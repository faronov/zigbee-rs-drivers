# zigbee-scd4x

Sensirion SCD40 / SCD41 / SCD43 photoacoustic CO2 sensors with integrated
temperature and humidity, over I2C. Full datasheet command set: periodic,
low-power periodic and single-shot (incl. RH/T-only) measurement,
data-ready, temperature offset, altitude, ambient pressure, ASC (enable,
target, initial/standard period), forced recalibration, persist settings,
serial number, self test, factory reset, reinit, sensor variant probe and
power down / wake up (`LowPower`).

| | |
|---|---|
| Address | 0x62 (fixed) |
| Conversion | single shot 5 s (RH/T only 50 ms); periodic 5 s, low-power periodic 30 s intervals |
| Average current (3.3 V) | periodic 15 mA, low-power periodic 3.2 mA, single shot every 5 min 0.45 mA (SCD41/43) |
| Peak current | 175 mA typ. at 3.3 V — needs a stiff supply |
| Power down | `power_down` / `wake_up` (wake-up is not acknowledged; 30 ms); current not specified in the datasheet |
| Output | CO2 ppm, 0.01 °C, 0.01 %RH |
| Source | Sensirion Datasheet SCD4x v1.7 (April 2025) |
| Status | mock-tested |

`probe()` reads the sensor variant; SCD41/SCD43 then default to single-shot
mode through the `Sensor` trait (override with `with_mode`). In periodic
modes the first `start_measurement` starts measuring and the next read is
`NotReady`; call `start_periodic_measurement()` at init to avoid that.
Single-shot results are fetched with bounded I2C acknowledge polling (no
delay). Getters take a delay for the 1 ms execution time; FRC (400 ms) and
self test (10 s) are split into `start_*` / `read_*`. Settings are volatile
until `persist_settings` (EEPROM, limited endurance).

Wiring: VDD 2.4–5.5 V, pull-ups on SDA/SCL, ≤ 30 mV supply ripple.
