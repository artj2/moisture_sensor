# Matter-over-Thread Soil Moisture Sensor (ESP32-C6)

Low-power capacitive soil moisture sensor built on the **ESP32-C6** using **ESP-IDF 6.0.2** and **esp-matter 1.6**. The device operates as a Matter Long Idle Time Intermittently Connected Device (**LIT ICD**, `43,200 s` idle interval) over **Thread** (OpenThread Minimal Thread Device / Sleepy End Device) and reports soil moisture (`0–100%`, `0.01%` resolution) via the standard Matter `RelativeHumidityMeasurement` cluster (`0x0405`) to **Home Assistant**.

---

## 1. Hardware & Wiring

| Component / Signal | ESP32-C6 Pin | Notes |
| :--- | :--- | :--- |
| **MCU Board** | Waveshare ESP32-C6-Zero | `ESP32-C6` (4 MB flash), native USB-Serial/JTAG (`/dev/ttyACM0`) |
| **Sensor `VCC`** | `GPIO1` (`SENSOR_POWER_PIN`) | Capacitive Soil Moisture Sensor v2.0 — driven HIGH (`GPIO_DRIVE_CAP_3`) for `250 ms` + `16` ADC samples (interquartile trimmed mean of middle 8) per wake, then held LOW through deep sleep to prevent probe electrolysis |
| **Sensor `AOUT`** | `GPIO2` (`ADC1_CH2`) | `12 dB` ADC attenuation (`ADC_ATTEN_DB_12`) with ESP32-C6 factory eFuse curve-fitting millivolt calibration (`ADC_DRY_MV = 2553`, `ADC_WET_MV = 881`), or raw 12-bit fallback counts (`ADC_DRY_VAL`, `ADC_WET_VAL`) |
| **Sensor `GND`** | `GND` | Common ground |
| **Onboard `WS2812B` LED** | `GPIO8` (`WS2812_DATA_PIN`) | Data line held LOW in deep sleep (`gpio_hold_en`) |
| **Factory Reset Button** | `GPIO9` (`BOOT_BUTTON_PIN`) | Hold `BOOT` for `5 s` while awake to erase Matter/Thread NVS credentials and reopen BLE commissioning |

### Power Mode Note (Mains Today / Optional Battery Later)
The sensor currently runs on **USB / mains power** (deep sleep cycles still run normally, and `/dev/ttyACM0` disconnects between wakes).

If you later deploy on **batteries** (for example `3 × AA`):
- **Desolder or disconnect the onboard `WS2812B` RGB LED (`GPIO8`)**: even with `GPIO8` held LOW, the smart LED controller IC continuously draws **~300–600 µA** from the `3V3` rail, preventing the ~`10–25 µA` deep-sleep floor.
- Optionally coat exposed probe PCB edges (and pot/heat-shrink top SMD components) to protect against moisture wicking in soil.

---

## 2. Home Assistant Prerequisites (Thread, Matter & Chrony)

Before flashing the sensor, prepare these three components in Home Assistant:

### A. OpenThread Border Router (OTBR)
1. Ensure you have a working **Thread** network and **OpenThread Border Router** set up in Home Assistant.
2. **Thread Channel Recommendation:** If your Thread network is on **Channel 15** (`2425 MHz`) and you observe slow Thread attaches (`>15 s`), retransmissions, or missed/failed reports, consider switching to **Thread Channel 25** (`2475 MHz`) to avoid 2.4 GHz Wi-Fi overlap. On a clean Channel 25 link, wakes complete in ~`10.7 s` total (`4.75–6.00 s` Matter report completion).

### B. Matter Server Add-on (`Enable test-net DCL usage`)
Because [`sdkconfig`](sdkconfig) uses Espressif's development test Vendor ID (`0xFFF1`) and Product ID (`0x8000`), open the Home Assistant **Matter Server** add-on configuration and enable **"Enable test-net DCL usage"** before pairing.

### C. Add Chrony (Local NTP Server over Thread)
> [!IMPORTANT]
> **Why Chrony Is Required:**
> The firmware disables IPv4 (`CONFIG_DISABLE_IPV4=y`, `CONFIG_LWIP_IPV4=n`) and operates as an isolated Thread Sleepy End Device, so it cannot query public IPv4 NTP servers or rely on public DNS. Instead, it syncs UTC directly with Home Assistant's local Chrony server over the Thread IPv6 mesh (`HA_NTP_SERVER_ULA`).

1. In Home Assistant, go to **Settings → Add-ons → Add-on Store**, search for the standard **Chrony** add-on, install it, and start it using its **default configuration**. (Enable **Start on boot** so time sync stays available after Home Assistant restarts.)
2. Find your Home Assistant host's Thread **Unique Local Address (ULA)** — an IPv6 address starting with `fd...` matching your Thread network prefix:
   - **Option 1 (UI):** Go to **Settings → System → Network → Network Adapter** and look at the IPv6 addresses assigned to the host interface.
   - **Option 2 (Terminal & SSH add-on):** Run:
     ```bash
     ip -6 addr show
     ```
     Look for the `fd...` address with `/64` `scope global` on your main host network interface.

---

## 3. Firmware Configuration (`main/app_main.cpp`)

Before building, inspect the configuration macros at the top of [`main/app_main.cpp`](main/app_main.cpp):

| Macro | Current Default | Required? | Purpose |
| :--- | :--- | :--- | :--- |
| `HA_NTP_SERVER_ULA` | `"fdea:6164:84c:1:8daf:5cbf:b421:c513"` | **Yes** | Your Home Assistant host's Thread ULA (`fd...`) for SNTP queries to Chrony |
| `DEVICE_VENDOR_NAME_STR` | `"Artur"` | Optional | Vendor name in Home Assistant Device Info (max 32 chars) |
| `DEVICE_PRODUCT_NAME_STR` | `"Plant Soil Sensor"` | Optional | Product name / node label in Home Assistant (max 32 chars) |
| `DEVICE_HARDWARE_VER_STR` | `"ESP32-C6-Zero & Capacitive Soil Moisture Sensor v2.0"` | Optional | Hardware version string in Device Info (max 64 chars) |
| `ADC_DRY_MV` / `ADC_WET_MV` | `2553` / `881` | Calibrate per probe | eFuse-calibrated dry-in-air (`0%` physical) and wet-in-water (`100%` physical) millivolts (set either to `0` to derive mV on-chip from `ADC_DRY_VAL` / `ADC_WET_VAL`) |
| `ADC_DRY_VAL` / `ADC_WET_VAL` | `2572.0f` / `886.0f` | Fallback / derive mV | Raw 12-bit ADC endpoints used if eFuse calibration is unavailable (or when `*_MV` is `0`) |
| `SOIL_SCALE_MIN_RAW_PCT` / `SOIL_SCALE_MAX_RAW_PCT` | `24.0f` / `84.0f` | Optional | Rescales the usable potting-soil physical `%` range onto `0%–100%` reported in Home Assistant (`MAX > MIN`) |
| `TEST_SHORT_SLEEP` | `0` | Debug only | Set to `1` for `2-minute` test wake cycles (`SLEEP_SECS_TEST`) |
| `MONITORING_MAX_6H` | `0` | Debug only | Set to `1` for `6h / 4h / 2h` capped monitoring intervals (`12h / 6h / 2h` when `0`) |
| `s_rtc_scale` | `1.0129f` | Optional | Initial empirical RTC wake-timer scale factor (`RTC_DATA_ATTR`), refined automatically on each SNTP-synced wake |

### How to Measure Your Probe's `ADC_DRY_MV` and `ADC_WET_MV`
Capacitive moisture probes output **higher** voltage when dry and **lower** voltage when wet:
1. Build and flash with defaults, then open `idf.py monitor` (or power-cycle/tap `RST` to watch a wake log).
2. Hold the probe completely dry in air and note the logged line:
   `Raw ADC: <raw> (<mv> mV, ...)` → use `<mv>` as `ADC_DRY_MV` (and `<raw>` as `ADC_DRY_VAL`).
3. Submerge the probe's capacitive sensing blade up to the marked line in water (keep top electronics dry) and note `<mv>` → use as `ADC_WET_MV` (and `<raw>` as `ADC_WET_VAL`).
4. Update [`main/app_main.cpp`](main/app_main.cpp), rebuild, and flash.

---

## 4. Build & Flash

### Prerequisites
- **ESP-IDF 6.0.2** and **esp-matter 1.6** installed (default paths used below: `~/esp/esp-idf` and `~/esp/esp-matter`).
- On Linux, ensure your user has serial port access (log out and back in after adding):
  ```bash
  sudo usermod -aG dialout "$USER"
  ```

### 1. Build
```bash
source ~/esp/esp-idf/export.sh
source ~/esp/esp-matter/export.sh
export IDF_CCACHE_ENABLE=1
idf.py build
```
([`CMakeLists.txt`](CMakeLists.txt) defaults `IDF_TARGET` to `esp32c6`. Run `idf.py set-target esp32c6` only if regenerating configuration from scratch.)

### 2. Full Initial Flash (New Board)
```bash
idf.py -p /dev/ttyACM0 flash monitor
```
Wait for the bootloader/partition table (`partitions.csv`) and application (`ota_0` at `0x20000`) to flash; serial logs will then start printing commissioning details.

### 3. Firmware-Only Update (Preserves Home Assistant Pairing / NVS)
In [`partitions.csv`](partitions.csv), `nvs` sits at `0x10000` (`0xC000` bytes) and `ota_0` sits at `0x20000` (`0x1E0000` bytes). To update firmware **without** erasing Matter fabrics or Thread credentials, flash only `0x20000` (**never** write to `0x0` or `0x10000` during updates):

```bash
python -m esptool --chip esp32c6 -p /dev/ttyACM0 -b 460800 \
  --before default-reset --after hard-reset write-flash \
  --flash-mode dio --flash-freq 80m --flash-size 4MB \
  0x20000 build/moisture_sensor.bin
```

> [!TIP]
> **Entering Download Mode When Sleeping:**
> Because native USB-Serial/JTAG (`/dev/ttyACM0`) disconnects during deep sleep, either run `esptool` right after tapping `RST` / during a scheduled wake, or force bootloader download mode manually by **holding `BOOT` (`GPIO9`), tapping `RST`, then releasing `BOOT`**.

---

## 5. Commissioning with Home Assistant

1. **Confirm Test DCL Is Enabled:** Double-check **"Enable test-net DCL usage"** is enabled in the Home Assistant **Matter Server** add-on.
2. **Ensure Phone Has Thread Credentials:** Verify the Home Assistant Companion App on your phone has your Thread network credentials synced (**Settings → Devices & services → Thread**).
3. **BLE Pairing:**
   - While uncommissioned, the sensor stays awake continuously and advertises over BLE (`passcode: 20202021`, `discriminator: 3840`), logging the QR code and manual pairing code periodically over USB serial.
   - In the Home Assistant Companion App, open **Settings → Devices & services → Add integration → Add Matter device**, choose **No, it's new**, and scan the printed QR code (or enter the printed manual pairing code / passcode `20202021`).
   - Leave the board powered for at least `45 s` (`COMMISSION_SETTLE_MS`) after pairing completes so Home Assistant finishes interviewing endpoints and establishing its ICD subscription before the first deep sleep.
4. **Device & Entity in Home Assistant:**
   - The device appears as **`Plant Soil Sensor`** (by `Artur`) exposing a **Humidity** entity (`0–100%`) mapped from `RelativeHumidityMeasurement` (`0x0405`). You can optionally rename the entity or customize its icon/unit display in Home Assistant.
5. **Factory Reset (Re-Pairing):**
   - Wake the board (tap `RST`) and immediately hold `BOOT` (`GPIO9`) for **5 seconds**, then release. `wait_if_boot_button_pressed()` holds off deep sleep while `BOOT` is pressed so `esp_matter::factory_reset()` clears NVS and re-enters BLE commissioning mode.

### Optional: Next Check-In Template Helper in Home Assistant
If you want a countdown or expected next wake timestamp on your dashboard, create a **Template Sensor** helper (`device_class: timestamp`, replacing the entity ID if you renamed it):

```jinja2
{% set m = states('sensor.plant_soil_sensor_humidity') | float(0) %}
{% set hours = 12 if m > 35 else (6 if m > 20 else 2) %}
{% set last = as_datetime(states.sensor.plant_soil_sensor_humidity.last_reported) %}
{% set next_time = last + timedelta(hours=hours) %}
{{ next_time.replace(minute=0, second=0, microsecond=0).isoformat() }}
```

---

## 6. Adaptive Sleep, Report Gate & Time Synchronization Details

### Soil Rescale & Adaptive Sleep Policy
- **Potting-Soil Rescale (`24% – 84%` physical → `0% – 100%` reported):** Stretches the usable potting-soil range onto `0–100%` (`100%` = freshly watered peak, `~67–77%` = settled field capacity, `10%` = watering point, `0%` = bone-dry floor).

| Reported Soil Moisture | Tier | Deep-Sleep Interval | Macro |
| :--- | :--- | :--- | :--- |
| **`> 35.0%`** | Wet (settled & healthy) | **`12 hours`** (`43,200 s`) | `SLEEP_SECS_WET` |
| **`20.0% – 35.0%`** | Mid (drying down) | **`6 hours`** (`21,600 s`) | `SLEEP_SECS_MID` |
| **`<= 20.0%`** | Dry (approaching watering point) | **`2 hours`** (`7,200 s`) | `SLEEP_SECS_DRY` |
| Sensor read error | Error | `30 minutes` (`1,800 s`) | `SLEEP_SECS_ERROR` |

### Dynamic Report Completion & Self-Healing Retries
- **Event/State-Driven Sleep Gate (`wait_for_matter_report_complete`):** Keeps ICD in `500 ms` Active Mode while Thread attaches (`25 s` timeout via `THREAD_ATTACH_TIMEOUT_MS`) and a `Subscribe` `ReadHandler` registers (`30 s` `SUBSCRIPTION_WAIT_MS`), then waits until **`1.5 s` (`REPORT_QUIET_MS`)** of zero in-flight `ReportData` chunks and zero open Matter exchanges (up to `60 s` `REPORT_MAX_WAIT_MS`) before sleeping.
- **Exponential Retry Backoff:** If Thread attach fails or the report does not complete cleanly, the sensor retries with capped exponential backoff (`30 min → 1 h → 2 h → 4 h → 6 h`) via `SLEEP_SECS_RETRY` / `SLEEP_SECS_RETRY_MAX`.

### SNTP Time Sync & Top-of-Hour Alignment
- **Concurrent SNTP + Quiet-Window Fallback:** Starts SNTP polling (`esp_sntp_setserver` with pre-parsed binary `ip_addr_t` from `HA_NTP_SERVER_ULA`) when Thread attaches; if Matter reporting finishes before SNTP responds, it triggers one quiet-window `esp_sntp_restart()` retry (up to `1,000 ms`).
- **Top-of-Hour Snapping + Early-Wake Tolerance:** `calculate_aligned_sleep_seconds()` aligns wakeups to `((now / interval) + 1) * interval` UTC. If waking slightly early within `30 minutes` before the boundary (`early_tolerance_s = 1800`, or `interval / 4` for intervals under `2 h`), it treats that check-in as satisfying the boundary and sleeps to the following slot.
- **SNTP-Gated RTC Drift Calibration:** Starting from empirical baseline `s_rtc_scale = 1.0129f`, `log_and_calibrate_clock()` updates `s_rtc_scale = 0.80 * s_rtc_scale + 0.20 * measured_scale` (accepting `0.92..1.10`) **only** when `s_sntp_synced` succeeded on that wake.
