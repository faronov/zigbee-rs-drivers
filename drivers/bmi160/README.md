# zigbee-bmi160

Bosch BMI160 accelerometer and gyroscope. Use cases: vibration, tilt and
wake-on-motion.

| | |
|---|---|
| Bus | I2C 0x68 (SDO low) / 0x69 (SDO high); SPI mode 0/3 |
| ID | `CHIP_ID` 0xD1 |
| Output | acceleration in milli-g, angular rate in milli-dps (`Vector3`) |
| Default | ±2 g, 25 Hz, accelerometer normal mode, gyroscope suspended |
| Sampling | free-running; `read_measurement()` returns `NotReady` until `STATUS.drdy_acc` |
| Start-up | stepped: soft reset 1 ms → accelerometer normal 4.1 ms → gyroscope normal 80 ms (if enabled) → registers |
| Low-power accel | under-sampling from 0.78 Hz, 1–128× averaging (3 µA at 0.78 Hz, 4 µA at 1.56 Hz, 14 µA at 25 Hz, no averaging) |
| Motion interrupt | any-motion on INT1, all axes, full scale / 512 per LSB (3.91 mg at ±2 g), 1–4 samples, latched or not |
| Sleep | `LowPower::sleep` = soft reset, both sensors suspended (3 µA); accelerometer normal 180 µA, both normal 925 µA |
| Source | BST-BMI160-DS000-09 Rev 1.0 (Nov 2020) |
| Status | mock-tested |

Notes

* Commands occupy `CMD` for up to 3.8 ms (accelerometer) / 80 ms
  (gyroscope) and later commands are dropped meanwhile; while all sensors
  are suspended or in low-power mode each write also needs ~400 µs before
  the next. `configure()` therefore starts from a soft reset and returns a
  wait; call `continue_configure()` (or `start_measurement()`) after each
  wait until it returns 0. Registers are written while the accelerometer is
  in normal mode and the low-power command is written last.
* `enable_wake_on_motion()` / `disable_wake_on_motion()` restart the same
  sequence. While any-motion is enabled `read_measurement()` skips the
  data-ready check. In latched mode `motion_detected()` releases INT1 with
  the `int_reset` command; allow 400 µs before the next write in low-power
  mode.
* `sleep()` is a soft reset and loses all settings; allow 1 ms before the
  next access. Over SPI the driver performs the dummy read of 0x7F that
  re-selects SPI after power-up and reset (use `new_spi`).
* Accelerometer low-power mode with the gyroscope on is not supported by
  the part (`Unsupported`), nor are normal-mode ODRs below 12.5 Hz.
* Gyroscope sensitivities are the datasheet values 16.4/32.8/65.6/131.2/262.4
  LSB/dps.
