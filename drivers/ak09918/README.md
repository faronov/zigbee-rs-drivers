# zigbee-ak09918

AKM AK09918 3-axis Hall-effect magnetometer (as used on Grove IMU
boards). Use cases: door/window contact with a magnet, utility meter dial
reading, compass heading.

| | |
|---|---|
| Bus | I2C 0x0C (fixed), up to 400 kHz; 1.65–1.95 V supply |
| ID | `WIA1` (0x00) = 0x48 (AKM), `WIA2` (0x01) = 0x0C |
| Output | magnetic field in nT (`Vector3`), 16-bit, 0.15 µT/LSB, ±4912 µT |
| Default | single measurements, chip in power-down in between |
| Sampling | single measurement per `start_measurement()`; `read_measurement()` returns `NotReady` until `ST1.DRDY`; optional continuous mode 10/20/50/100 Hz |
| Conversion | 7.2 ms typ., 8.2 ms max (driver waits 8.2 ms) |
| Offset | factory sensitivity adjustment on chip; no offset cancellation (initial offset up to ±2000 LSB, calibrate in the application) |
| Threshold interrupt | none (no interrupt pin) |
| Current | 1 µA typ. / 3 µA max power-down; 1.5 mA while measuring; 1.1 mA average at 100 Hz |
| Sleep | `LowPower::sleep` = power-down; single mode powers down by itself after each measurement; `wake()` restarts continuous mode at the next `start_measurement()` |
| Source | AKM AK09918 datasheet 016014242-E-00 (2016/11) |
| Status | mock-tested |

Notes

* Mode changes must go through power-down and wait 100 µs (`Twait`).
  `configure()` writes power-down and returns 100 µs; the next
  `start_measurement()` starts continuous mode. Do not start a new single
  measurement before the previous one finished (8.2 ms).
* `read_measurement()` reads `ST1` first and only then the data and
  `ST2` (reading `ST2` releases the data protection), so a measurement
  that completes in between is not lost.
* Magnetic sensor overflow (`ST2.HOFL`, |X| + |Y| + |Z| ≥ 4912 µT)
  returns `InvalidData`.
* `start_self_test()` / `read_self_test()` run the internal-field
  self-test with the datasheet limits; the chip must be in power-down.
* The temperature register (`TMPS`) is a dummy on this part.
