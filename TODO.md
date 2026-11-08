# Moisture Sensor — Improvements & Backlog

Tracked roadmap of reliability, battery-life, accuracy, and hardware improvements.
Current active queue: **`#3` → `#5` → `#8`**.

---

## 1. High-Impact Reliability & Battery Life

- [ ] **1. Boost 802.15.4 Thread TX Power (+ Optional Thread Channel Move)**
  - Set ESP32-C6 802.15.4 transmit power to `+20 dBm` (`otPlatRadioSetTransmitPower(instance, 20)`) during OpenThread initialization in [`main/app_main.cpp`](main/app_main.cpp) to add ~10–15 dB of link margin against `error:NoAck` timeouts.
  - Optional Home Assistant network tweak: move Thread from Channel `15` (`2425 MHz`, overlaps 2.4 GHz Wi-Fi Ch 3–4) to Channel `25` (`2475 MHz`).

- [ ] **2. Remove Onboard `WS2812B` RGB LED (`GPIO8`) Before `3 × AA` Deployment**
  - Hardware step: desolder/disconnect the onboard `WS2812B` LED on the ESP32-C6 Mini board to eliminate its ~`300–600 µA` quiescent `3V3` draw during deep sleep.

- [x] **3. Shorten Failed-Attach Timeout (`60 s` → `25 s`)**
  - Lower `THREAD_ATTACH_TIMEOUT_MS` from `60000` to `25000` in [`main/app_main.cpp`](main/app_main.cpp). Covers OpenThread's first 4 attach attempts (`~1–22 s` observed in 5-day telemetry) while cutting wasted active battery drain by ~57% when the Border Router is unreachable.

---

## 2. Functional & Accuracy Upgrades

- [ ] **4. Add Battery % & Voltage Reporting (`PowerSource` Cluster)**
  - Add a switched high-impedance voltage divider to a spare ADC1 pin (e.g., `GPIO3`) and enable `CONFIG_SUPPORT_POWER_SOURCE_CLUSTER=y` in [`sdkconfig.defaults`](sdkconfig.defaults) to expose `BatPercentRemaining` and `BatVoltage` in Home Assistant.

- [ ] **5. Use Factory eFuse Millivolt ADC Calibration (`esp_adc/adc_cali_scheme.h`)** *(Up next)*
  - Initialize ESP32-C6 curve-fitting ADC calibration (`adc_cali_create_scheme_curve_fitting`) so `read_moisture_percent()` maps calibrated millivolts (`mV`) rather than uncalibrated raw 12-bit counts, compensating for die-temperature and chip-to-chip ADC drift.

- [ ] **6. Skip Radio Wake When Moisture Is Unchanged (Optional Power Saver)**
  - Track last reported moisture and elapsed sleep seconds in `RTC_DATA_ATTR`; if moisture changed by `< 0.5%` and `< 12 h` has elapsed since the last report, go straight back to deep sleep after the `250 ms` ADC sample without powering up Thread.

---

## 3. Hardware Longevity & Polish

- [ ] **7. Seal Capacitive Probe Edges & Top Electronics**
  - Coat exposed FR4 PCB edges with thin epoxy / clear varnish and pot or heat-shrink the top 555-timer SMD components to prevent moisture wicking and corrosion over multi-year soil deployment.

- [ ] **8. Clean Up Stale Fabric `0x1` & Rename `light` Project/Device Metadata** *(Queued after #5)*
  - Rename build target from `project(light)` (`build/light.bin`) to `project(moisture_sensor)` in [`CMakeLists.txt`](CMakeLists.txt) and set proper Matter `BasicInformation` device metadata (`ProductName` / `NodeLabel`).
  - Optional Home Assistant step: remove unused secondary `Fabric index 0x1` (`VendorId 0x6006`) via Home Assistant's *Manage Fabrics* dialog.
