# zigbee-hmc5883l

Honeywell HMC5883L 3-axis AMR magnetometer. Use cases: door/window
contact with a magnet, utility meter dial reading, compass heading.

| | |
|---|---|
| Bus | I2C 0x1E (fixed), up to 400 kHz |
| ID | identification registers A–C (0x0A–0x0C) = ASCII "H43" |
| Output | magnetic field in nT (`Vector3`), 12-bit; gain 1370–230 LSB/gauss for ±0.88 to ±8.1 gauss |
| Default | ±1.3 gauss (1090 LSB/gauss), no averaging, single measurements |
| Sampling | single measurement per `start_measurement()`; `read_measurement()` returns `NotReady` until status `RDY`; optional continuous mode 0.75–75 Hz; 1/2/4/8-sample averaging |
| Conversion | 6 ms (single measurement, no averaging); the driver waits 6 ms per averaged sample |
| Offset | automatic: the ASIC does set/reset per measurement, output = (Mset − Mreset)/2 |
| Threshold interrupt | none; DRDY pin (active low) signals data ready |
| Current | 2 µA idle; 100 µA at 7.5 Hz continuous without averaging |
| Sleep | `LowPower::sleep` = idle mode; single mode returns to idle by itself; `wake()` restarts continuous mode |
| Source | Honeywell HMC5883L datasheet Form #900405 Rev E (Feb 2013) |
| Status | mock-tested |

Notes

* A gain change only applies from the second measurement. In single mode
  `configure()` starts a dummy measurement and returns its duration; the
  next `start_measurement()` reads out and drops that sample. In
  continuous mode `configure()` returns 3 periods (first sample after
  2/f, second with the new gain).
* An axis overflow (output -4096) makes `read_measurement()` return
  `InvalidData`; choose a larger range when a magnet sits close by.
* The HMC5883L is discontinued. Many boards sold as "HMC5883L" carry a
  QMC5883L (address 0x0D, different registers); `probe()` fails on them —
  use `zigbee-qmc5883l`.
* The datasheet gives 6 ms only without averaging; scaling the wait by
  the sample count is a conservative assumption.
* Positive/negative bias self-test and high-speed I2C are not used.
