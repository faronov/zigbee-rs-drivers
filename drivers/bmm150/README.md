# zigbee-bmm150

Bosch BMM150 3-axis geomagnetic sensor. Use cases: door/window contact
with a magnet (threshold interrupt), utility meter dial reading, compass
heading.

| | |
|---|---|
| Bus | I2C 0x10 (0x11–0x13 via SDO/CSB), up to 400 kHz; SPI 4-wire mode 0/3, up to 10 MHz (minimal `Interface` trait, blocking + async) |
| ID | `CHIP_ID` (0x40) = 0x32, readable only after `power_up()` |
| Output | magnetic field in nT (`Vector3`), 1/16 µT resolution; ±1300 µT X/Y, ±2500 µT Z; temperature compensated on the host with the factory trim registers |
| Default | Regular preset (nXY 9, nZ 15, 0.6 µT noise), forced (single) measurements |
| Sampling | one forced measurement per `start_measurement()`; `read_measurement()` returns `NotReady` until `DRDY`, `InvalidData` on overflow; optional normal mode 2–30 Hz; presets Low power / Regular / Enhanced / High accuracy or custom repetitions |
| Conversion | 145 µs·nXY + 500 µs·nZ + 980 µs: 2.9 / 9.8 / 16.7 / 49.3 ms for the presets |
| Offset | none on chip; trim-based temperature/sensitivity compensation (integer port of Bosch SensorAPI) |
| Threshold interrupt | low/high threshold per axis on the INT pin (raw output vs. register × 16, ≈6 µT/step), latched or not; DRDY pin optional |
| Current | 1 µA typ. / 3 µA max suspend; 0.17 / 0.5 / 0.8 mA at 10 Hz (Low power / Regular / Enhanced), 4.9 mA at 20 Hz (High accuracy); 18 mA peak while measuring |
| Sleep | `LowPower::sleep` = suspend (registers lost); `wake()` = power-up (3 ms), configuration rewritten on the next `start_measurement()` |
| Source | Bosch Sensortec BMM150 datasheet BST-BMM150-DS001-05 rev. 1.4 (April 2020); BMM150 SensorAPI v2.0.0 (compensation) |
| Status | mock-tested; compensation matches the SensorAPI fixed-point code |

Notes

* The part starts in suspend mode where only register 0x4B is
  accessible: call `power_up()`, wait 3 ms, then `probe()`.
  `configure()` (or the first `start_measurement()`) reads and caches the
  trim registers.
* `src/compensation.rs` is derived from Bosch's BMM150 SensorAPI
  (BSD-3-Clause) and carries its notice. The final `/ 16` of SensorAPI
  v2.0.0 is left out to keep the 1/16 µT resolution.
* Preset repetitions follow the datasheet (nXY = 1 + 2·REPXY,
  nZ = 1 + REPZ, Table 3): REPZ = 2 / 14 / 26 / 82. SensorAPI v2.0.0 uses
  smaller REPZ values for its presets.
* Between forced measurements the chip stays in sleep mode, whose current
  the datasheet does not specify; call `sleep()` for the 1 µA suspend
  mode at the cost of a 3 ms wake-up and a configuration rewrite.
* The threshold interrupt compares uncompensated data and is only
  evaluated when a measurement runs, so use normal mode for an
  always-on contact sensor. The datasheet does not say whether the
  threshold registers are signed.
* Normal-mode presets slower than the ODR return `Unsupported`.
* The advanced (coil) self-test and the data-overrun flag are not used;
  `start_self_test()` / `read_self_test()` run the normal self-test.
