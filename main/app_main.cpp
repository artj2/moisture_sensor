#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_openthread.h"
#include "esp_openthread_lock.h"
#include "esp_openthread_types.h"
#include <openthread/dataset.h>
#include <platform/ESP32/OpenthreadLauncher.h>
#include "esp_matter.h"
#include <iot_button.h>
#include <button_gpio.h>
#include <app_reset.h>
#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>
#include <app/InteractionModelEngine.h>
#include <app/icd/server/ICDNotifier.h>
#include <messaging/ExchangeMgr.h>
#include <app-common/zap-generated/attributes/Accessors.h>
#include <platform/PlatformManager.h>
#include <platform/ConnectivityManager.h>
#include <setup_payload/OnboardingCodesUtil.h>
#include <app/clusters/relative-humidity-measurement-server/RelativeHumidityMeasurementCluster.h>
#include <app/clusters/relative-humidity-measurement-server/CodegenIntegration.h>

// ---------------------------------------------------------------------------
// Hardware configuration
// ---------------------------------------------------------------------------

#define SENSOR_POWER_PIN GPIO_NUM_1
#define SENSOR_ADC_CHAN  ADC_CHANNEL_2 // GPIO2 is ADC1 Channel 2 on ESP32-C6
#define WS2812_DATA_PIN  GPIO_NUM_8    // Onboard WS2812B RGB LED data line
#define BOOT_BUTTON_PIN  GPIO_NUM_9    // Onboard BOOT button (hold 5s for factory reset)

// Capacitive sensor power-on RC stabilization and multisampling
#define SENSOR_SETTLE_MS   250
#define SENSOR_ADC_SAMPLES 16

// Calibration values in air (dry) and water (wet) at 12 dB ADC attenuation.
// When ADC_DRY_MV / ADC_WET_MV are 0, hardware_init() derives the calibrated
// millivolt endpoints from ADC_DRY_VAL / ADC_WET_VAL through this chip's factory
// eFuse curve-fitting calibration and logs the exact mV values on boot so they
// can optionally be locked in for cross-board portability.
#define ADC_DRY_MV  0
#define ADC_WET_MV  0
#define ADC_DRY_VAL 2572.0f
#define ADC_WET_VAL 886.0f

// ---------------------------------------------------------------------------
// Sleep policy
// ---------------------------------------------------------------------------

// Set to 1 for 2-minute wake cycles while verifying Home Assistant updates;
// set to 0 for moisture-based intervals.
#define TEST_SHORT_SLEEP 0
#define SLEEP_SECS_TEST  (2 * 60)

// Set to 1 for the 24-48h monitoring run (2h dry / 4h mid / 6h wet);
// set to 0 for final production battery intervals (2h dry / 6h mid / 12h wet).
#define MONITORING_MAX_6H 0

// Wake interval as a function of measured moisture. Wetter soil changes slowly,
// so it is checked less often.
#if MONITORING_MAX_6H
#define SLEEP_SECS_WET   (6 * 60 * 60)    // moisture > 40%  || 6 hours (monitoring cap)
#define SLEEP_SECS_MID   (4 * 60 * 60)    // moisture > 35%  || 4 hours (monitoring tier)
#define SLEEP_SECS_DRY   (2 * 60 * 60)    // moisture <= 35% || 2 hours
#else
#define SLEEP_SECS_WET   (12 * 60 * 60)   // moisture > 40%  || 12 hours
#define SLEEP_SECS_MID   (6 * 60 * 60)    // moisture > 35%  || 6 hours
#define SLEEP_SECS_DRY   (2 * 60 * 60)    // moisture <= 35% || 2 hours
#endif
#define SLEEP_SECS_ERROR (30 * 60)        // sensor read failed

// If the reading never made it out (Thread did not attach, or the publish
// failed), retry sooner than the moisture-based interval. Backs off on repeated
// failures so a dead border router does not drain the pack at 30 min per wake.
#define SLEEP_SECS_RETRY     (30 * 60)
#define SLEEP_SECS_RETRY_MAX (6 * 60 * 60)

// How long to wait for Thread to attach after a wake before giving up.
// OpenThread's first 4 MLE attach attempts complete within ~1-22 s; cutting off
// at 25 s avoids burning active radio power through the 8s/16s/32s backoff tail
// when the Border Router is unreachable.
#define THREAD_ATTACH_TIMEOUT_MS 25000

// Dynamic Matter report completion limits after Thread attach:
// - Wait up to SUBSCRIPTION_WAIT_MS for subscription resumption (or controller
//   re-subscribe / Check-In) to register an active Subscribe ReadHandler.
// - Keep waiting up to REPORT_MAX_WAIT_MS while any ReportData chunk or Matter
//   ExchangeContext (CASE, chunked IM:ReportData, StatusResponse) is in flight.
// - Require REPORT_QUIET_MS of uninterrupted idle (active subscription present,
//   zero reports in flight, zero open exchanges) so the final StandaloneAck frame
//   finishes transmitting over 802.15.4 before deep sleep cuts radio power.
#define SUBSCRIPTION_WAIT_MS 30000
#define REPORT_MAX_WAIT_MS   60000
#define REPORT_QUIET_MS      1500

// Longer grace period for the first cycle after commissioning. The controller
// walks the descriptor tree, registers ICD Check-In, and sets up subscriptions
// immediately after pairing; sleeping through that leaves Home Assistant with
// an incomplete device.
#define COMMISSION_SETTLE_MS 45000

// ---------------------------------------------------------------------------

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

static const char *TAG = "THREAD_MOISTURE";
static uint16_t humidity_endpoint_id = 0;
static adc_oneshot_unit_handle_t adc1_handle = NULL;
static adc_cali_handle_t adc1_cali_handle = NULL;
static int s_cal_dry_mv = 0;
static int s_cal_wet_mv = 0;
static float initial_moisture_percent = -1.0f;
static int initial_adc_raw = 0;
static int initial_adc_mv = -1;
static volatile bool s_commissioning_complete = false;

// Survives deep sleep; cleared on power-on and hard reset.
RTC_DATA_ATTR static uint32_t wake_count = 0;
RTC_DATA_ATTR static uint32_t consecutive_failures = 0;

// RAII wrapper around the CHIP stack lock.
// esp_matter::lock::chip_stack_lock() is not usable in esp-matter 1.6 (the
// declarations sit inside the ScopedChipStackLock class body rather than at
// namespace scope), so go straight to the platform manager instead.
class ScopedStackLock {
public:
    ScopedStackLock()  { chip::DeviceLayer::PlatformMgr().LockChipStack(); }
    ~ScopedStackLock() { chip::DeviceLayer::PlatformMgr().UnlockChipStack(); }
private:
    ScopedStackLock(const ScopedStackLock &) = delete;
    ScopedStackLock &operator=(const ScopedStackLock &) = delete;
};

static uint16_t percent_to_centi(float moisture_percent)
{
    if (moisture_percent <= 0.0f) {
        return 0;
    }
    if (moisture_percent >= 100.0f) {
        return 10000;
    }
    long v = lroundf(moisture_percent * 100.0f);
    if (v < 0) return 0;
    if (v > 10000) return 10000;
    return static_cast<uint16_t>(v);
}

// ---------------------------------------------------------------------------
// Matter callbacks
// ---------------------------------------------------------------------------

static esp_err_t app_attribute_update_cb(
    attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
    uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    return ESP_OK;
}

static esp_err_t app_identification_cb(
    identification::callback_type_t type, uint16_t endpoint_id,
    uint8_t effect_id, uint8_t effect_variant, void *priv_data)
{
    ESP_LOGI(TAG, "Identification triggered by Matter controller");
    return ESP_OK;
}

static void log_active_thread_dataset(TickType_t lock_wait_ticks)
{
    if (!esp_openthread_lock_acquire(lock_wait_ticks)) {
        return;
    }
    otInstance *instance = esp_openthread_get_instance();
    otOperationalDataset dataset = {};
    if (instance && otDatasetGetActive(instance, &dataset) == OT_ERROR_NONE) {
        const uint8_t *ep = dataset.mExtendedPanId.m8;
        ESP_LOGI(TAG,
                 "Active Thread dataset: name='%s', channel=%u, pan=0x%04x, "
                 "extpan=%02x%02x%02x%02x%02x%02x%02x%02x",
                 dataset.mComponents.mIsNetworkNamePresent ? dataset.mNetworkName.m8 : "<unset>",
                 dataset.mComponents.mIsChannelPresent ? dataset.mChannel : 0,
                 dataset.mComponents.mIsPanIdPresent ? dataset.mPanId : 0,
                 ep[0], ep[1], ep[2], ep[3], ep[4], ep[5], ep[6], ep[7]);
    }
    esp_openthread_lock_release();
}

static void app_event_cb(const chip::DeviceLayer::ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kThreadStateChange:
        ESP_LOGI(TAG, "Thread state changed");
        log_active_thread_dataset(0);
        break;
    case chip::DeviceLayer::DeviceEventType::kThreadConnectivityChange:
        ESP_LOGI(TAG, "Thread connectivity changed");
        break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        s_commissioning_complete = true;
        ESP_LOGI(TAG, "Commissioning complete");
        break;
    case chip::DeviceLayer::DeviceEventType::kFabricRemoved: {
        ESP_LOGI(TAG, "Fabric removed");
        if (chip::Server::GetInstance().GetFabricTable().FabricCount() == 0) {
            s_commissioning_complete = false;
            chip::CommissioningWindowManager &commissionMgr =
                chip::Server::GetInstance().GetCommissioningWindowManager();
            if (!commissionMgr.IsCommissioningWindowOpen()) {
                CHIP_ERROR err = commissionMgr.OpenBasicCommissioningWindow(
                    chip::System::Clock::Seconds16(300),
                    chip::CommissioningWindowAdvertisement::kAllSupported);
                if (err != CHIP_NO_ERROR) {
                    ESP_LOGE(TAG, "Failed to reopen commissioning window: %" CHIP_ERROR_FORMAT,
                             err.Format());
                }
            }
        }
        break;
    }
    default:
        break;
    }
}

// ---------------------------------------------------------------------------
// Sensor
// ---------------------------------------------------------------------------

static float read_moisture_percent(int *raw_out, int *mv_out)
{
    // Power up with full GPIO drive strength, let the sensor's onboard LDO,
    // 555 oscillator, and RC filter settle, multisample, then power down.
    // Cycling power prevents electrolytic corrosion of the probe and saves current.
    gpio_set_level(SENSOR_POWER_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(SENSOR_SETTLE_MS));

    int dummy = 0;
    (void)adc_oneshot_read(adc1_handle, SENSOR_ADC_CHAN, &dummy);

    int64_t raw_sum = 0;
    int64_t mv_sum = 0;
    int valid_samples = 0;
    int valid_mv_samples = 0;
    esp_err_t last_err = ESP_OK;
    for (int i = 0; i < SENSOR_ADC_SAMPLES; i++) {
        int sample = 0;
        esp_err_t err = adc_oneshot_read(adc1_handle, SENSOR_ADC_CHAN, &sample);
        if (err == ESP_OK) {
            raw_sum += sample;
            valid_samples++;
            if (adc1_cali_handle != NULL) {
                int sample_mv = 0;
                if (adc_cali_raw_to_voltage(adc1_cali_handle, sample, &sample_mv) == ESP_OK) {
                    mv_sum += sample_mv;
                    valid_mv_samples++;
                }
            }
        } else {
            last_err = err;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    gpio_set_level(SENSOR_POWER_PIN, 0);

    if (valid_samples == 0) {
        if (raw_out) {
            *raw_out = 0;
        }
        if (mv_out) {
            *mv_out = -1;
        }
        ESP_LOGE(TAG, "ADC read failed: %s", esp_err_to_name(last_err));
        return -1.0f;
    }

    int adc_raw = static_cast<int>((raw_sum + (valid_samples / 2)) / valid_samples);
    if (raw_out) {
        *raw_out = adc_raw;
    }

    if (valid_mv_samples > 0 && s_cal_dry_mv > s_cal_wet_mv) {
        int adc_mv = static_cast<int>((mv_sum + (valid_mv_samples / 2)) / valid_mv_samples);
        if (mv_out) {
            *mv_out = adc_mv;
        }
        // Capacitive sensors read HIGH voltage when dry, LOW voltage when wet.
        if (adc_mv >= s_cal_dry_mv) {
            return 0.0f;
        }
        if (adc_mv <= s_cal_wet_mv) {
            return 100.0f;
        }
        return 100.0f * static_cast<float>(s_cal_dry_mv - adc_mv) /
               static_cast<float>(s_cal_dry_mv - s_cal_wet_mv);
    }

    if (mv_out) {
        *mv_out = -1;
    }

    // Fallback to raw counts if factory eFuse calibration is unavailable.
    // Capacitive sensors read HIGH when dry, LOW when wet, so the mapping is inverted.
    if (adc_raw >= (int)ADC_DRY_VAL) {
        return 0.0f;
    }
    if (adc_raw <= (int)ADC_WET_VAL) {
        return 100.0f;
    }
    return 100.0f * (ADC_DRY_VAL - (float)adc_raw) / (ADC_DRY_VAL - ADC_WET_VAL);
}

// ---------------------------------------------------------------------------
// Matter publish
// ---------------------------------------------------------------------------

// RelativeHumidityMeasurement is a code-driven cluster: the cluster instance owns
// MeasuredValue, not the esp-matter attribute store. attribute::update() on it
// returns ESP_ERR_NOT_SUPPORTED (0x106), so use the cluster's own setter.
static esp_err_t publish_humidity(uint16_t centi_percent)
{
    ScopedStackLock guard;

    // Enter ICD Active Mode (fast Thread polling) so subscription resumption,
    // Check-In, and ReportData exchanges complete briskly before deep sleep.
    chip::app::ICDNotifier::GetInstance().NotifyNetworkActivityNotification();

    CHIP_ERROR chip_err = RelativeHumidityMeasurement::SetMeasuredValue(
        humidity_endpoint_id,
        chip::app::DataModel::MakeNullable<uint16_t>(centi_percent));

    if (chip_err != CHIP_NO_ERROR) {
        // ConstraintError means the value fell outside Min/MaxMeasuredValue.
        ESP_LOGE(TAG, "SetMeasuredValue(%u) failed: %" CHIP_ERROR_FORMAT,
                 centi_percent, chip_err.Format());
        return ESP_FAIL;
    }

    return ESP_OK;
}

static bool is_commissioned(void)
{
    ScopedStackLock guard;
    return chip::Server::GetInstance().GetFabricTable().FabricCount() > 0;
}

static void log_onboarding_codes(void)
{
    ScopedStackLock guard;
    PrintOnboardingCodes(chip::RendezvousInformationFlags(chip::RendezvousInformationFlag::kBLE));
}

static void ensure_commissioning_window_open(void)
{
    ScopedStackLock guard;
    if (chip::Server::GetInstance().GetFabricTable().FabricCount() > 0) {
        return;
    }
    chip::CommissioningWindowManager &commissionMgr =
        chip::Server::GetInstance().GetCommissioningWindowManager();
    if (!commissionMgr.IsCommissioningWindowOpen()) {
        ESP_LOGI(TAG, "Reopening BLE commissioning window");
        CHIP_ERROR err = commissionMgr.OpenBasicCommissioningWindow(
            chip::System::Clock::Seconds16(900),
            chip::CommissioningWindowAdvertisement::kAllSupported);
        if (err != CHIP_NO_ERROR) {
            ESP_LOGE(TAG, "Failed to reopen commissioning window: %" CHIP_ERROR_FORMAT,
                     err.Format());
        }
    }
}

static bool wait_for_thread_attached(uint32_t timeout_ms)
{
    const uint32_t poll_ms = 250;
    uint32_t waited = 0;

    while (waited < timeout_ms) {
        bool attached;
        {
            ScopedStackLock guard;
            attached = chip::DeviceLayer::ConnectivityMgr().IsThreadAttached();
        }
        if (attached) {
            ESP_LOGI(TAG, "Thread attached after %" PRIu32 " ms", waited);
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(poll_ms));
        waited += poll_ms;
    }

    ESP_LOGW(TAG, "Thread did not attach within %" PRIu32 " ms", timeout_ms);
    return false;
}

static bool wait_for_matter_report_complete(bool just_commissioned)
{
    const uint32_t poll_ms = 250;
    const uint32_t min_wait_ms = just_commissioned ? COMMISSION_SETTLE_MS : 0;
    uint32_t waited = 0;
    uint32_t sub_age_ms = 0;
    uint32_t quiet_ms = 0;
    uint32_t last_subs = 0;
    uint32_t last_reports = 0;
    uint32_t last_exchanges = 0;

    ESP_LOGI(TAG, "Waiting for Matter subscription report & ACKs (max %u ms)...",
             (unsigned)REPORT_MAX_WAIT_MS);

    while (waited < REPORT_MAX_WAIT_MS) {
        uint32_t subs = 0;
        uint32_t reports = 0;
        uint32_t exchanges = 0;
        {
            ScopedStackLock guard;
            auto *im = chip::app::InteractionModelEngine::GetInstance();
            if (im != nullptr) {
                subs = im->GetNumActiveReadHandlers(
                    chip::app::ReadHandler::InteractionType::Subscribe);
                reports = im->GetReportingEngine().GetNumReportsInFlight();
            }
            exchanges = static_cast<uint32_t>(
                chip::Server::GetInstance().GetExchangeManager().GetNumActiveExchanges());

            // Keep ICD in Active Mode (500 ms Thread polling) while waiting for
            // SRP/mDNS lookup and CASE resumption before an ExchangeContext opens.
            if (subs == 0 && (waited % 2500) == 0) {
                chip::app::ICDNotifier::GetInstance().NotifyNetworkActivityNotification();
            }
        }

        last_subs = subs;
        last_reports = reports;
        last_exchanges = exchanges;

        if (subs > 0) {
            sub_age_ms += poll_ms;
            // Give Reporting::Engine::ScheduleRun() at least 1s after ReadHandler
            // creation before counting idle ticks, then require REPORT_QUIET_MS of
            // zero in-flight reports and zero active exchanges.
            if (sub_age_ms >= 1000 && reports == 0 && exchanges == 0) {
                quiet_ms += poll_ms;
                if (quiet_ms >= REPORT_QUIET_MS && waited >= min_wait_ms) {
                    ESP_LOGI(TAG,
                             "Matter report & ACKs completed after %" PRIu32 " ms (subs=%" PRIu32 ")",
                             waited, subs);
                    return true;
                }
            } else {
                quiet_ms = 0;
            }
        } else {
            sub_age_ms = 0;
            quiet_ms = 0;
            if (!just_commissioned && waited >= SUBSCRIPTION_WAIT_MS && exchanges == 0) {
                break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(poll_ms));
        waited += poll_ms;
    }

    ESP_LOGW(TAG,
             "Matter report did not complete cleanly within %" PRIu32
             " ms (subs=%" PRIu32 ", reports=%" PRIu32 ", exchanges=%" PRIu32 ")",
             waited, last_subs, last_reports, last_exchanges);
    return false;
}

// ---------------------------------------------------------------------------
// Sleep
// ---------------------------------------------------------------------------

static uint32_t sleep_seconds_for(float moisture_percent)
{
#if TEST_SHORT_SLEEP
    (void)moisture_percent;
    return SLEEP_SECS_TEST;
#else
    if (moisture_percent < 0.0f) {
        return SLEEP_SECS_ERROR;
    }
    if (moisture_percent > 40.0f) {
        return SLEEP_SECS_WET;
    }
    if (moisture_percent > 35.0f) {
        return SLEEP_SECS_MID;
    }
    return SLEEP_SECS_DRY;
#endif
}

static void wait_if_boot_button_pressed(void)
{
    if (gpio_get_level(BOOT_BUTTON_PIN) == 0) {
        ESP_LOGI(TAG, "BOOT button held - delaying sleep until released (hold 5s for factory reset)");
        while (gpio_get_level(BOOT_BUTTON_PIN) == 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        // Allow iot_button's release callback (esp_matter::factory_reset) to run.
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}

static void enter_deep_sleep(uint32_t seconds)
{
    wait_if_boot_button_pressed();

    ESP_LOGI(TAG, "Sleeping for %" PRIu32 " s", seconds);

    if (adc1_cali_handle) {
        adc_cali_delete_scheme_curve_fitting(adc1_cali_handle);
        adc1_cali_handle = NULL;
    }
    if (adc1_handle) {
        adc_oneshot_del_unit(adc1_handle);
        adc1_handle = NULL;
    }

    // Make sure the probe and WS2812 data line are held LOW through deep sleep.
    gpio_set_level(SENSOR_POWER_PIN, 0);
    gpio_set_level(WS2812_DATA_PIN, 0);
    gpio_hold_en(SENSOR_POWER_PIN);
    gpio_hold_en(WS2812_DATA_PIN);

    esp_sleep_enable_timer_wakeup((uint64_t)seconds * 1000000ULL);

    // Give the log a moment to flush over USB-Serial/JTAG before the CPU stops.
    vTaskDelay(pdMS_TO_TICKS(50));

    esp_deep_sleep_start();
}

// ---------------------------------------------------------------------------
// Main measurement cycle
// ---------------------------------------------------------------------------

static void measure_and_sleep_task(void *pvParameters)
{
    int adc_raw = initial_adc_raw;
    int adc_mv = initial_adc_mv;
    float moisture_percent = initial_moisture_percent;
    if (adc_mv >= 0) {
        ESP_LOGI(TAG, "Wake #%" PRIu32 " - Raw ADC: %d (%d mV), Moisture: %.1f%%",
                 wake_count, adc_raw, adc_mv, moisture_percent);
    } else {
        ESP_LOGI(TAG, "Wake #%" PRIu32 " - Raw ADC: %d, Moisture: %.1f%%",
                 wake_count, adc_raw, moisture_percent);
    }

    if (is_commissioned()) {
        s_commissioning_complete = true;
    }

    for (;;) {
        // Never sleep while uncommissioned, or the commissioning window would close
        // every time and the device could never be added to Home Assistant.
        // Wait for kCommissioningComplete (not just FabricCount() > 0 during AddNOC)
        // so a failed ConnectNetwork rollback never sends the device to deep sleep.
        bool just_commissioned = false;
        if (!is_commissioned() || !s_commissioning_complete) {
            ESP_LOGW(TAG, "Not commissioned - staying awake for BLE commissioning");
            log_onboarding_codes();
            uint32_t wait_loops = 0;
            while (!s_commissioning_complete || !is_commissioned()) {
                vTaskDelay(pdMS_TO_TICKS(5000));
                wait_loops++;
                if ((wait_loops % 6) == 0) {
                    ensure_commissioning_window_open();
                    ESP_LOGI(TAG, "Waiting for Matter commissioning (passcode 20202021, discriminator 3840)...");
                    log_onboarding_codes();
                }
            }
            just_commissioned = true;
            ESP_LOGI(TAG, "Now commissioned - entering sleep cycle");

            // The reading above was taken before pairing and may be old by now.
            moisture_percent = read_moisture_percent(&adc_raw, &adc_mv);
            if (adc_mv >= 0) {
                ESP_LOGI(TAG, "Post-commissioning reading - Raw ADC: %d (%d mV), Moisture: %.1f%%",
                         adc_raw, adc_mv, moisture_percent);
            } else {
                ESP_LOGI(TAG, "Post-commissioning reading - Raw ADC: %d, Moisture: %.1f%%",
                         adc_raw, moisture_percent);
            }
        }

        log_active_thread_dataset(pdMS_TO_TICKS(100));

        bool published = false;
        if (wait_for_thread_attached(THREAD_ATTACH_TIMEOUT_MS) && moisture_percent >= 0.0f) {
            uint16_t matter_humidity = percent_to_centi(moisture_percent);
            if (publish_humidity(matter_humidity) == ESP_OK) {
                published = wait_for_matter_report_complete(just_commissioned);
            }
        }

        // If the fabric was removed while we were awake, stay awake for re-pairing.
        if (!is_commissioned()) {
            s_commissioning_complete = false;
            continue;
        }

        uint32_t sleep_secs;
        if (published) {
            consecutive_failures = 0;
            sleep_secs = sleep_seconds_for(moisture_percent);
        } else {
#if TEST_SHORT_SLEEP
            consecutive_failures++;
            sleep_secs = SLEEP_SECS_TEST;
            ESP_LOGW(TAG, "Reading not published (failure #%" PRIu32 ") - retrying in %" PRIu32 " s (test mode)",
                     consecutive_failures, sleep_secs);
#else
            // Exponential backoff, capped. 30 min, 1 h, 2 h, 4 h, 6 h, 6 h...
            uint32_t shift = consecutive_failures < 4 ? consecutive_failures : 4;
            sleep_secs = SLEEP_SECS_RETRY << shift;
            if (sleep_secs > SLEEP_SECS_RETRY_MAX) {
                sleep_secs = SLEEP_SECS_RETRY_MAX;
            }
            consecutive_failures++;
            ESP_LOGW(TAG, "Reading not published (failure #%" PRIu32 ") - retrying in %" PRIu32 " s",
                     consecutive_failures, sleep_secs);
#endif
        }

        enter_deep_sleep(sleep_secs);
    }
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

static void button_init(void)
{
    button_handle_t btn_handle = NULL;
    const button_config_t btn_cfg = {};
    const button_gpio_config_t btn_gpio_cfg = {
        .gpio_num = BOOT_BUTTON_PIN,
        .active_level = 0,
        .enable_power_save = false,
        .disable_pull = false,
    };

    esp_err_t err = iot_button_new_gpio_device(&btn_cfg, &btn_gpio_cfg, &btn_handle);
    if (err != ESP_OK || !btn_handle) {
        ESP_LOGE(TAG, "Failed to init BOOT button on GPIO%d: %s",
                 BOOT_BUTTON_PIN, esp_err_to_name(err));
        return;
    }

    err = app_reset_button_register(btn_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register factory reset on BOOT button: %s",
                 esp_err_to_name(err));
    }
}

static void hardware_init(void)
{
    // Release deep-sleep pad holds from the previous cycle before driving pins.
    gpio_hold_dis(SENSOR_POWER_PIN);
    gpio_hold_dis(WS2812_DATA_PIN);

    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << SENSOR_POWER_PIN) | (1ULL << WS2812_DATA_PIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_set_drive_capability(SENSOR_POWER_PIN, GPIO_DRIVE_CAP_3);
    gpio_set_level(SENSOR_POWER_PIN, 0);
    gpio_set_level(WS2812_DATA_PIN, 0);

    adc_oneshot_unit_init_cfg_t init_config1 = {};
    init_config1.unit_id = ADC_UNIT_1;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t chan_config = {};
    chan_config.atten = ADC_ATTEN_DB_12;
    chan_config.bitwidth = ADC_BITWIDTH_DEFAULT;
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, SENSOR_ADC_CHAN, &chan_config));

    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = SENSOR_ADC_CHAN,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    esp_err_t cali_err = adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_handle);
    if (cali_err == ESP_OK) {
        int dry_mv = ADC_DRY_MV;
        int wet_mv = ADC_WET_MV;
        if (dry_mv <= 0 || wet_mv <= 0) {
            esp_err_t e1 = adc_cali_raw_to_voltage(
                adc1_cali_handle, static_cast<int>(lroundf(ADC_DRY_VAL)), &dry_mv);
            esp_err_t e2 = adc_cali_raw_to_voltage(
                adc1_cali_handle, static_cast<int>(lroundf(ADC_WET_VAL)), &wet_mv);
            if (e1 != ESP_OK || e2 != ESP_OK) {
                dry_mv = 0;
                wet_mv = 0;
            }
        }
        if (dry_mv > wet_mv && wet_mv > 0) {
            s_cal_dry_mv = dry_mv;
            s_cal_wet_mv = wet_mv;
            ESP_LOGI(TAG,
                     "ADC eFuse curve-fitting calibration active: dry=%d mV (raw %.0f), wet=%d mV (raw %.0f)",
                     s_cal_dry_mv, ADC_DRY_VAL, s_cal_wet_mv, ADC_WET_VAL);
        } else {
            ESP_LOGW(TAG,
                     "Invalid calibrated mV endpoints (dry=%d, wet=%d) - falling back to raw ADC counts",
                     dry_mv, wet_mv);
            adc_cali_delete_scheme_curve_fitting(adc1_cali_handle);
            adc1_cali_handle = NULL;
        }
    } else {
        adc1_cali_handle = NULL;
        ESP_LOGW(TAG, "ADC eFuse calibration unavailable (%s) - falling back to raw ADC counts",
                 esp_err_to_name(cali_err));
    }

    button_init();
}

extern "C" void app_main(void)
{
    wake_count++;

    uint32_t causes = esp_sleep_get_wakeup_causes();
    ESP_LOGI(TAG, "Boot (wakeup causes: 0x%08" PRIx32 ", timer: %s, wake count: %" PRIu32 ")",
             causes,
             (causes & BIT(ESP_SLEEP_WAKEUP_TIMER)) ? "yes" : "no",
             wake_count);

    // 1. NVS (required for Thread and Matter credentials)
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // 2. Default event loop (required for Matter events)
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 3. OpenThread platform config: native 802.15.4 radio, no host UART/SPI link
    esp_openthread_platform_config_t ot_config = {};
    ot_config.radio_config.radio_mode = RADIO_MODE_NATIVE;
    ot_config.host_config.host_connection_mode = HOST_CONNECTION_MODE_NONE;
    ot_config.port_config.storage_partition_name = "nvs";
    ot_config.port_config.netif_queue_size = 10;
    ot_config.port_config.task_queue_size = 10;
    set_openthread_platform_config(&ot_config);

    // 4. ADC, GPIO, and factory-reset button (GPIO9, 5s hold)
    hardware_init();

    // Read the sensor before creating the Matter endpoint so MeasuredValue starts
    // with the actual moisture reading rather than 0% during subscription resumption.
    initial_moisture_percent = read_moisture_percent(&initial_adc_raw, &initial_adc_mv);
    uint16_t initial_centi = (initial_moisture_percent >= 0.0f)
                                 ? percent_to_centi(initial_moisture_percent)
                                 : 0;

    // 5. Matter node
    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    if (!node) {
        ESP_LOGE(TAG, "Failed to create Matter node");
        return;
    }

    // 6. Humidity sensor endpoint.
    // These config fields are nullable<uint16_t>, not plain uint16_t.
    humidity_sensor::config_t sensor_config;
    sensor_config.relative_humidity_measurement.measured_value =
        (initial_moisture_percent >= 0.0f) ? nullable<uint16_t>(initial_centi) : nullable<uint16_t>();
    sensor_config.relative_humidity_measurement.min_measured_value = nullable<uint16_t>(0);
    sensor_config.relative_humidity_measurement.max_measured_value = nullable<uint16_t>(10000);

    endpoint_t *sensor_endpoint =
        humidity_sensor::create(node, &sensor_config, ENDPOINT_FLAG_NONE, NULL);
    if (!sensor_endpoint) {
        ESP_LOGE(TAG, "Failed to create humidity sensor endpoint");
        return;
    }
    humidity_endpoint_id = endpoint::get_id(sensor_endpoint);
    ESP_LOGI(TAG, "Humidity sensor endpoint created, id = %d (initial = %u centi-%%)",
             humidity_endpoint_id, initial_centi);

    // 7. Start the Matter stack
    err = esp_matter::start(app_event_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start Matter: %s", esp_err_to_name(err));
        return;
    }

    // 8. One measurement cycle, then deep sleep. This task does not return.
    xTaskCreate(measure_and_sleep_task, "measure_and_sleep", 6144, NULL, 5, NULL);
}
