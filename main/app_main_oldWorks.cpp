#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <esp_matter.h>
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <app/server/Server.h>
#include <app-common/zap-generated/ids/Clusters.h>
#include <app-common/zap-generated/ids/Attributes.h>
#include <platform/CHIPDeviceLayer.h>
#include <platform/CHIPDeviceEvent.h>

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <platform/ESP32/OpenthreadLauncher.h>
#include <esp_openthread_types.h>
#include <esp_openthread.h>

#ifndef ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG
#define ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG() \
    { .radio_mode = RADIO_MODE_NATIVE, }

#define ESP_OPENTHREAD_DEFAULT_HOST_CONFIG() \
    { .host_connection_mode = HOST_CONNECTION_MODE_NONE, }

#define ESP_OPENTHREAD_DEFAULT_PORT_CONFIG() \
    { .storage_partition_name = "nvs", .netif_queue_size = 10, .task_queue_size = 10, }
#endif
#endif

#include <app/server-cluster/ServerClusterInterface.h>

using namespace esp_matter;
using namespace esp_matter::cluster;
using namespace esp_matter::endpoint;

// Strong implementations of the cluster callbacks to resolve the linker error
void ESPMatterRelativeHumidityMeasurementClusterServerInitCallback(uint16_t endpoint) {}
void ESPMatterRelativeHumidityMeasurementClusterServerShutdownCallback(uint16_t endpoint, chip::app::ClusterShutdownType shutdownType) {}

static const char *TAG = "moisture_sensor";

// Hardware Pins
#define SENSOR_PWR_PIN   GPIO_NUM_1
#define SENSOR_ADC_CHAN  ADC_CHANNEL_2  // GPIO 1

// How often to sample + report in this always-on validation build
#define REPORT_INTERVAL_MS (30 * 1000)

// --- Retained for the future battery (sleepy) build; unused while always-on ---
// Thresholds (Lower ADC = Wet, Higher ADC = Dry)
#define THRESH_MEDIUM_DRY 2500
#define THRESH_VERY_DRY   3200
// -----------------------------------------------------------------------------

uint16_t moisture_endpoint_id = 0;

static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg) {
    if (event->Type == chip::DeviceLayer::DeviceEventType::kCommissioningComplete) {
        ESP_LOGI(TAG, "Commissioning complete! Device is now paired.");
    }
}

extern "C" void app_main()
{
    esp_err_t err = ESP_OK;

    nvs_flash_init();

    // --- Matter data model: one humidity_sensor endpoint (device type 0x0307) ---
    node::config_t node_config;
    node_t *node = node::create(&node_config, nullptr, nullptr);

    humidity_sensor::config_t sensor_config;
    sensor_config.relative_humidity_measurement.measured_value = static_cast<uint16_t>(0);

    endpoint_t *endpoint = humidity_sensor::create(node, &sensor_config, ENDPOINT_FLAG_NONE, nullptr);
    moisture_endpoint_id = endpoint::get_id(endpoint);

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    set_openthread_platform_config(&config);
#endif

    err = esp_matter::start(app_event_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start Matter, err:%d", err);
        return;
    }

    // --- Sensor hardware: configure once, reuse for every sample ---
    gpio_set_direction(SENSOR_PWR_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(SENSOR_PWR_PIN, 0);  // sensor off until we sample

    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    adc_oneshot_new_unit(&init_config, &adc1_handle);

    adc_oneshot_chan_cfg_t config_adc = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    adc_oneshot_config_channel(adc1_handle, SENSOR_ADC_CHAN, &config_adc);

    ESP_LOGI(TAG, "Always-on mode: sampling every %d s (no deep sleep)", REPORT_INTERVAL_MS / 1000);

    // --- Periodic sample + report loop. Never sleeps, so the device stays a
    //     live Thread router and Home Assistant can interview it and keep it
    //     online. When uncommissioned, esp_matter auto-advertises for pairing. ---
    while (true) {
        // Power the sensor, let it settle, read, power it back down
        gpio_set_level(SENSOR_PWR_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(100));

        int raw_moisture = 0;
        adc_oneshot_read(adc1_handle, SENSOR_ADC_CHAN, &raw_moisture);

        gpio_set_level(SENSOR_PWR_PIN, 0);

        ESP_LOGI(TAG, "Moisture Reading (raw ADC): %d", raw_moisture);

        // NOTE: RelativeHumidityMeasurement::MeasuredValue is expressed in 0.01%
        // units (valid range 0..10000), so a raw ADC value (0..4095) shows up as a
        // small percentage in HA. That's fine for validating the pipeline end to
        // end; scale/convert it to a real % once you've confirmed it works.
        esp_matter_attr_val_t val =
            esp_matter_nullable_uint16(nullable<uint16_t>(static_cast<uint16_t>(raw_moisture)));
        attribute::update(moisture_endpoint_id,
                          chip::app::Clusters::RelativeHumidityMeasurement::Id,
                          chip::app::Clusters::RelativeHumidityMeasurement::Attributes::MeasuredValue::Id,
                          &val);

        vTaskDelay(pdMS_TO_TICKS(REPORT_INTERVAL_MS));
    }
}