# Matter-over-Thread Soil Moisture Sensor (ESP32-C6)

Low-power capacitive soil moisture sensor built on the **ESP32-C6** using **ESP-IDF 6.0.2** and **esp-matter 1.6**. The device operates as a Matter Long Idle Time Intermittently Connected Device (**LIT ICD**) over **Thread** (OpenThread Sleepy End Device) and reports soil moisture (`0–100%`, `0.01%` resolution) via the standard Matter `RelativeHumidityMeasurement` cluster (`0x0405`) to **Home Assistant**.

---

## Hardware & Wiring

| Component / Signal | ESP32-C6 Pin | Notes |
| :--- | :--- | :--- |
| **MCU Board** | Waveshare / The Pi Hut ESP32-C6 Mini | `ESP32-C6FH8`, native USB-Serial/JTAG (`/dev/ttyACM0`) |
| **Sensor `VCC`** | `GPIO1` (`SENSOR_POWER_PIN`) | Driven HIGH (`GPIO_DRIVE_CAP_3`) for `250 ms` + 16 ADC samples per wake, held LOW in deep sleep to prevent probe electrolysis and save power |
| **Sensor `AOUT`** | `GPIO2` (`ADC1_CH2`) | 12 dB ADC attenuation with ESP32-C6 factory eFuse curve-fitting millivolt calibration (`ADC_DRY_MV` / `ADC_WET_MV`, derived on-chip from `ADC_DRY_VAL = 2572.0f` / `ADC_WET_VAL = 886.0f` if unset) |
| **Sensor `GND`** | `GND` | Common ground |
| **Onboard `WS2812B` LED** | `GPIO8` (`WS2812_DATA_PIN`) | Data line held LOW in deep sleep (**disconnect/desolder LED on battery power** — see below) |
| **Factory Reset Button** | `GPIO9` (`BOOT` button) | Hold for `5 s` while awake to clear Matter/Thread NVS credentials and reopen BLE pairing |

### `3 × AA` Battery Power Note
Even with `GPIO8` held LOW in deep sleep, the onboard `WS2812B` smart RGB LED's internal controller IC draws **~300–600 µA** continuously from the `3V3` rail. **Desolder or disconnect the onboard `WS2812B` LED before deploying on `3 × AA` batteries** to reach the ~`10–25 µA` deep-sleep floor required for ≥ 1 year battery life.

---

## Adaptive Sleep & Reliable Delivery Policy

Configured in [`main/app_main.cpp`](main/app_main.cpp):

| Soil Moisture | Tier | Deep-Sleep Interval | Macro |
| :--- | :--- | :--- | :--- |
| **`<= 35.0%`** | Dry | **`2 hours`** (`7,200 s`) | `SLEEP_SECS_DRY` |
| **`35.0% – 40.0%`** | Mid | **`6 hours`** (`21,600 s`) | `SLEEP_SECS_MID` |
| **`> 40.0%`** | Wet | **`12 hours`** (`43,200 s`) | `SLEEP_SECS_WET` |
| Sensor read error | Error | `30 minutes` (`1,800 s`) | `SLEEP_SECS_ERROR` |

### Dynamic Matter Report Completion & Self-Healing Retries
- **Event/State-Driven Sleep Gate (`wait_for_matter_report_complete`):** Instead of a fixed post-publish delay, the sensor polls `InteractionModelEngine` and `ExchangeManager` after Thread attach—keeping ICD in `500 ms` Active Mode during mDNS/SRP lookup, waiting until an active `Subscribe` `ReadHandler` is registered, and requiring **`1.5 s` of uninterrupted zero in-flight `ReportData` chunks and zero open Matter exchanges** (up to a `60 s` ceiling) before calling `esp_deep_sleep_start()`. Normal wakes finish in **~12.5–14 s**, while wakes experiencing 802.15.4 retransmissions or subscription renewals stay awake until every `StatusResponse (0x00)` and `StandaloneAck` completes.
- **Exponential Retry Backoff:** If Thread fails to attach within `25 s` or the Matter report does not complete cleanly, the reading is marked unpublished and retried with capped exponential backoff (`30 min → 1 h → 2 h → 4 h → 6 h`) via `SLEEP_SECS_RETRY` / `SLEEP_SECS_RETRY_MAX` rather than sleeping for the full `6h`/`12h` interval.
- **Test / Monitoring Toggles:**
  - `TEST_SHORT_SLEEP` (`0` in production): set to `1` for `2-minute` test cycles.
  - `MONITORING_MAX_6H` (`0` in production): set to `1` for `2h / 4h / 6h` capped monitoring intervals.

---

## Build & Flash

### 1. Build
```bash
source ~/esp/esp-idf/export.sh
source ~/esp/esp-matter/export.sh
export IDF_CCACHE_ENABLE=1
idf.py build
```

### 2. Firmware-Only Update (Preserves Home Assistant Pairing / NVS)
`nvs` lives at `0x10000` (`0xC000` bytes) and `ota_0` lives at `0x20000` (see [`partitions.csv`](partitions.csv)). To flash code updates **without** erasing Matter fabric or Thread credentials, flash only `0x20000` while the chip is awake (or right after tapping `RST`):

```bash
python -m esptool --chip esp32c6 -p /dev/ttyACM0 -b 460800 \
  --before default-reset --after hard-reset write-flash \
  --flash-mode dio --flash-freq 80m --flash-size 4MB \
  0x20000 build/light.bin
```

### 3. Full Initial Flash (New Board)
```bash
idf.py -p /dev/ttyACM0 flash monitor
```

---

## Commissioning with Home Assistant

1. **Test DAC Setting:** Because the firmware uses Espressif's development Matter test Vendor ID (`0xFFF1`), enable **"Enable test-net DCL usage"** in the Home Assistant **Matter Server** add-on configuration before pairing.
2. **BLE Pairing:** While uncommissioned, the device stays awake continuously and advertises over BLE (`passcode: 20202021`, `discriminator: 3840`; QR/manual code printed on USB serial). After commissioning completes, it stays awake at least `45 s` (`COMMISSION_SETTLE_MS`) so Home Assistant finishes interviewing descriptors and establishing its subscription before the first deep-sleep cycle.
3. **Factory Reset:** Wake the board (tap `RST`) and immediately hold `BOOT` (`GPIO9`) for **5 seconds**, then release. [`wait_if_boot_button_pressed()`](main/app_main.cpp) prevents deep sleep while `BOOT` is held so `esp_matter::factory_reset()` can erase NVS and restart into BLE commissioning mode.
