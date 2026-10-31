# zigbee-lsm6dso

ST 6-axis IMUs (accelerometer + gyroscope) over I2C or SPI:
LSM6DS3, LSM6DS33 (`WHO_AM_I` 0x69), LSM6DSL, LSM6DS3TR-C (0x6A),
LSM6DSO, LSM6DSOX (0x6C). Use cases: vibration, tilt/orientation and
wake-on-motion for sleepy end devices.

| | |
|---|---|
| Bus | I2C 0x6A (SA0 low) / 0x6B (SA0 high); SPI mode 0/3, 4-wire |
| Output | acceleration in milli-g, angular rate in milli-dps (`Vector3`) |
| Default | accelerometer ±2 g, 26 Hz low-power; gyroscope off |
| Sampling | free-running; `read_measurement()` returns `NotReady` until STATUS_REG.XLDA |
| Start-up | 2 accelerometer ODR periods; gyroscope 70 ms (conservative) |
| Motion interrupt | wake-up (slope) threshold, FS/64 per LSB, INT1 or INT2, optional latch |
| Sleep | `LowPower::sleep` = power-down (~3 µA typ.), `wake` re-applies the config |
| Source | ST LSM6DS3 DocID026899 Rev 4, LSM6DS33 DocID027423 Rev 4, LSM6DSL/LSM6DSO/LSM6DSOX datasheets (registers cross-checked with ST register headers) |
| Status | mock-tested |

Notes

* LSM6DSO and LSM6DSOX (and LSM6DSL and LSM6DS3TR-C) share a `WHO_AM_I`
  value; only the register subset common to them is used, so the driver
  does not need to tell them apart.
* The wake-up interrupt needs the accelerometer running. For a motion
  triggered sleepy end device keep the accelerometer at 12.5–26 Hz in
  low-power mode instead of calling `sleep()`, which also stops motion
  detection.
* Sensitivities are the datasheet nominal values (0.061 mg/LSB at ±2 g,
  70 mdps/LSB at ±2000 dps), so the gyroscope reports up to ±2293 dps.
* The 1.6 Hz accelerometer ODR exists only on LSM6DSL/LSM6DSO/LSM6DSOX in
  low-power mode; other combinations return `Error::Unsupported`.
* SPI 3-wire mode is not supported. Keep CS high at power-up for I2C.
