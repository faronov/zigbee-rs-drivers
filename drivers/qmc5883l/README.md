# zigbee-qmc5883l

QST QMC5883L and QMC5883P 3-axis AMR magnetometers. The two parts have
different addresses, IDs and register maps; choose `Variant` in
`Qmc5883::new()`. Use cases: door/window contact with a magnet, utility
meter dial reading, compass heading.

| | |
|---|---|
| Bus | I2C, fixed address: L 0x0D, P 0x2C; up to 400 kHz |
| ID | L: register 0x0D = 0xFF; P: register 0x00 = 0x80 |
| Output | magnetic field in nT (`Vector3`), 16-bit; L ±2 / ±8 gauss (12000 / 3000 LSB/gauss); P ±2 / ±8 / ±12 / ±30 gauss (15000 / 3750 / 2500 / 1000 LSB/gauss) |
| Default | ±8 gauss, highest over-sampling, on-request measurements |
| Sampling | one measurement per `start_measurement()`; `read_measurement()` returns `NotReady` until `DRDY`, `InvalidData` on overflow; optional periodic mode 10/50/100/200 Hz |
| Conversion | L: 5 ms (one 200 Hz period, emulated single shot); P: 0.7 ms × OSR1 × OSR2 estimate (5.6 ms by default) |
| Offset | internal offset cancellation and temperature compensation; P set/reset mode (on / set only / off) |
| Threshold interrupt | none; L has a DRDY pin (`Config::drdy_pin`), P has no interrupt pin |
| Current | L: 3 µA standby, 75 / 100 µA at 10 Hz (OSR 64 / 512); P: 22 µA suspend, 35–78 µA at 10 Hz (OSR1 1–8) |
| Sleep | `LowPower::sleep` = standby (L) / suspend (P); `wake()` restarts periodic mode |
| Source | QST QMC5883L datasheet Doc 13-52-04 Rev A; QST QMC5883P datasheet Doc 13-52-19 Rev C |
| Status | mock-tested |

Notes

* The QMC5883L has no single-measurement mode: `start_measurement()`
  switches it to continuous mode at 200 Hz and `read_measurement()` puts
  it back to standby. Keep the time in between short.
* The QMC5883L chip ID register reads 0xFF, so `probe()` mostly checks
  that something answers at 0x0D. Many "HMC5883L" (GY-271) boards carry
  a QMC5883L.
* The QMC5883P datasheet gives no conversion time; the 0.7 ms per
  OSR1 × OSR2 step is derived from the 1500 Hz maximum rate and the 5 ms
  self-test wait. `read_measurement()` returns `NotReady` if it is short.
* QMC5883P register 0x29 (axis sign) is written with 0x06 as in all
  datasheet examples; it is otherwise undocumented.
* Temperature output (offset not calibrated) and self-test are not used;
  the QMC5883P high-rate "continuous" mode (up to 1500 Hz) is not exposed.
