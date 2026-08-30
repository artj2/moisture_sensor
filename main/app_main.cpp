#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_openthread_types.h"
#include <platform/ESP32/OpenthreadLauncher.h>
#include "esp_matter.h"
#include <app/server/Server.h>
#include <app-common/zap-generated/attributes/Accessors.h>
#include <platform/PlatformManager.h>
#include <platform/ConnectivityManager.h>
#include <app/clusters/relative-humidity-measurement-server/RelativeHumidityMeasurementCluster.h>
#include <app/clusters/relative-humidity-measurement-server/CodegenIntegration.h>

// ---------------------------------------------------------------------------
// Hardware configuration
// ---------------------------------------------------------------------------

#define SENSOR_POWER_PIN GPIO_NUM_1
#define SENSOR_ADC_CHAN  ADC_CHANNEL_2 // GPIO2 is ADC1 Channel 2 on ESP32-C6

// Calibration values (raw ADC at 12 dB attenuation)
#define ADC_DRY_VAL 2572.0f
#define ADC_WET_VAL 886.0f

// ---------------------------------------------------------------------------
// Sleep policy
// ---------------------------------------------------------------------------

// Wake interval as a function of measured moisture. Wetter soil changes slowly,
// so it is checked less often.
#define SLEEP_SECS_WET     180   // moisture > 70%
#define SLEEP_SECS_MID     120   // moisture > 50%
#define SLEEP_SECS_DRY      60   // moisture <= 50%
#define SLEEP_SECS_ERROR    60   // sensor read failed

// How long to wait for Thread to attach after a wake before giving up.
#define THREAD_ATTACH_TIMEOUT_MS 30000

// Time to stay awake after publishing, so the report and its ack get out
// before the radio is cut. Tune this once you can see report timing in the log.
#define REPORT_SETTLE_MS 3000

// ---------------------------------------------------------------------------

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

static const char *TAG = "THREAD_MOISTURE";
static uint16_t humidity_endpoint_id = 0;
static adc_oneshot_unit_handle_t adc1_handle = NULL;

// Survives deep sleep; cleared on power-on and hard reset.
RTC_DATA_ATTR static uint32_t wake_count = 0;

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

static void app_event_cb(const chip::DeviceLayer::ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kThreadStateChange:
        ESP_LOGI(TAG, "Thread state changed");
        break;
    case chip::DeviceLayer::DeviceEventType::kThreadConnectivityChange:
        ESP_LOGI(TAG, "Thread connectivity changed");
        break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete");
        break;
    case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
        ESP_LOGI(TAG, "Fabric removed");
        break;
    default:
        break;
    }
}

// ---------------------------------------------------------------------------
// Sensor
// ---------------------------------------------------------------------------

static float read_moisture_percent(int *raw_out)
{
    int adc_raw = 0;

    // Power up, settle, read, power down.
    // Cycling power prevents electrolytic corrosion of the probe and saves current.
    gpio_set_level(SENSOR_POWER_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    esp_err_t err = adc_oneshot_read(adc1_handle, SENSOR_ADC_CHAN, &adc_raw);
    gpio_set_level(SENSOR_POWER_PIN, 0);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC read failed: %s", esp_err_to_name(err));
        return -1.0f;
    }

    if (raw_out) {
        *raw_out = adc_raw;
    }

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

// ---------------------------------------------------------------------------
// Sleep
// ---------------------------------------------------------------------------

static uint32_t sleep_seconds_for(float moisture_percent)
{
    if (moisture_percent < 0.0f) {
        return SLEEP_SECS_ERROR;
    }
    if (moisture_percent > 70.0f) {
        return SLEEP_SECS_WET;
    }
    if (moisture_percent > 50.0f) {
        return SLEEP_SECS_MID;
    }
    return SLEEP_SECS_DRY;
}

static void enter_deep_sleep(uint32_t seconds)
{
    ESP_LOGI(TAG, "Sleeping for %" PRIu32 " s", seconds);

    // Make sure the probe is not left powered through sleep.
    gpio_set_level(SENSOR_POWER_PIN, 0);

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
    // Read the sensor straight away. This does not need the radio, so if the
    // network never comes up the reading is still in the log.
    int adc_raw = 0;
    float moisture_percent = read_moisture_percent(&adc_raw);
    ESP_LOGI(TAG, "Wake #%" PRIu32 " - Raw ADC: %d, Moisture: %.1f%%",
             wake_count, adc_raw, moisture_percent);

    // Never sleep while uncommissioned, or the commissioning window would close
    // every time and the device could never be added to Home Assistant.
    if (!is_commissioned()) {
        ESP_LOGW(TAG, "Not commissioned - staying awake for commissioning");
        while (!is_commissioned()) {
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
        ESP_LOGI(TAG, "Now commissioned - entering sleep cycle");
    }

    if (wait_for_thread_attached(THREAD_ATTACH_TIMEOUT_MS) && moisture_percent >= 0.0f) {
        uint16_t matter_humidity = (uint16_t)(moisture_percent * 100.0f);
        if (publish_humidity(matter_humidity) == ESP_OK) {
            // Let the subscription report actually go out.
            vTaskDelay(pdMS_TO_TICKS(REPORT_SETTLE_MS));
        }
    }

    enter_deep_sleep(sleep_seconds_for(moisture_percent));
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

static void hardware_init(void)
{
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << SENSOR_POWER_PIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(SENSOR_POWER_PIN, 0);

    adc_oneshot_unit_init_cfg_t init_config1 = {};
    init_config1.unit_id = ADC_UNIT_1;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t chan_config = {};
    chan_config.atten = ADC_ATTEN_DB_12;
    chan_config.bitwidth = ADC_BITWIDTH_DEFAULT;
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, SENSOR_ADC_CHAN, &chan_config));
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

    // 4. ADC and GPIO
    hardware_init();

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
    sensor_config.relative_humidity_measurement.measured_value     = nullable<uint16_t>(0);
    sensor_config.relative_humidity_measurement.min_measured_value = nullable<uint16_t>(0);
    sensor_config.relative_humidity_measurement.max_measured_value = nullable<uint16_t>(10000);

    endpoint_t *sensor_endpoint =
        humidity_sensor::create(node, &sensor_config, ENDPOINT_FLAG_NONE, NULL);
    if (!sensor_endpoint) {
        ESP_LOGE(TAG, "Failed to create humidity sensor endpoint");
        return;
    }
    humidity_endpoint_id = endpoint::get_id(sensor_endpoint);
    ESP_LOGI(TAG, "Humidity sensor endpoint created, id = %d", humidity_endpoint_id);

    // 7. Start the Matter stack
    err = esp_matter::start(app_event_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start Matter: %s", esp_err_to_name(err));
        return;
    }

    // 8. One measurement cycle, then deep sleep. This task does not return.
    xTaskCreate(measure_and_sleep_task, "measure_and_sleep", 4096, NULL, 5, NULL);
}