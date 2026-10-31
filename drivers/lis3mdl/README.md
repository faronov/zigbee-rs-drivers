# zigbee-lis3mdl

ST LIS3MDL 3-axis magnetometer. Use cases: door/window contact with a
magnet, utility meter dial reading, orientation.

| | |
|---|---|
| Bus | I2C 0x1C (SDO/SA1 low) / 0x1E (SDO/SA1 high); 4-wire SPI mode 3 |
| ID | `WHO_AM_I` 0x3D |
| Output | magnetic field in nT (`Vector3`, 1 gauss = 100 000 nT) |
| Default | ±4 gauss, ultra-high performance X/Y/Z, power-down between single conversions |
| Sampling | single conversion per `start_measurement()`; `read_measurement()` returns `NotReady` until `STATUS.ZYXDA` |
| Conversion | LP 1.2 ms, MP 1.91 ms, HP 3.48 ms, UHP 6.65 ms (AN4602 turn-on time) |
| Threshold interrupt | `INT_THS` 15-bit absolute threshold (±, same LSB as output) on selected axes, INT pin active high/low, latched until `INT_SRC` read; runs the part continuously at 0.625–80 Hz |
| Current | 1 µA power-down; continuous 6 µA (LP, 0.625 Hz) – 270 µA (UHP, 20 Hz) |
| Sleep | `LowPower::sleep` = power-down (`MD` = 11); `wake()` restarts continuous mode if a threshold is enabled |
| Source | LIS3MDL datasheet DocID024204 Rev 4; AN4602 DocID027014 Rev 1 |
| Status | mock-tested |

Notes

* `configure()` writes `CTRL_REG1/2/4/5`, the threshold registers and
  `CTRL_REG3` (power-down, or continuous with a threshold); `BDU` is set so
  the six output bytes always belong to one sample. The part leaves
  single-conversion mode for power-down by itself.
* I2C burst reads set bit 7 of the sub-address and SPI reads set bits 7
  (read) and 6 (auto-increment). Tie SDO/SA1 and CS (high) for I2C.
* `FAST_ODR`, the temperature sensor (no offset specified in the
  datasheet), self-test and the `CTRL_REG3.LP` shortcut are not used.
* For a contact sensor, either poll single conversions on a timer and
  compare against a calibrated baseline, or enable the threshold interrupt
  at a low continuous rate and wake on INT.
