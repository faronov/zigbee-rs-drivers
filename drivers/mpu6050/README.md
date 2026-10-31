# zigbee-mpu6050

InvenSense MPU-6050 / MPU-6000, MPU-6500 and MPU-9250 accelerometer and
gyroscope (the MPU-9250's AK8963 magnetometer is not used). Use cases:
vibration, tilt and wake-on-motion.

| | |
|---|---|
| Bus | I2C 0x68 (AD0 low) / 0x69 (AD0 high); SPI mode 0/3 on MPU-6000/6500/9250 |
| ID | `WHO_AM_I` 0x68 (6050/6000), 0x70 (6500), 0x71 (9250) |
| Output | acceleration in milli-g, angular rate in milli-dps (`Vector3`) |
| Default | ±2 g, 25 Hz (1 kHz / 40), 44 Hz DLPF, gyroscope in standby |
| Sampling | free-running; `read_measurement()` returns `NotReady` until `INT_STATUS.DATA_RDY` |
| Start-up | accelerometer 20 ms, gyroscope 35 ms from sleep; 100 ms after reset |
| Low-power accel | cycle mode: MPU-6050 1.25/5/20/40 Hz (10/20/60/110 µA); MPU-6500/9250 0.24–500 Hz (8.4 µA at 0.98 Hz, 19.8 µA at 31.25 Hz) |
| Motion interrupt | MPU-6050 motion detection (1 mg/LSB, 5 Hz HPF, `MOT_DUR` ms); MPU-6500/9250 Wake-on-Motion (4 mg/LSB) |
| Sleep | `LowPower::sleep` = SLEEP bit (5 µA MPU-6050, 8 µA MPU-9250) |
| Source | PS-MPU-6000A-00 Rev 3.1, RM-MPU-6000A-00 Rev 4.0, PS-MPU-9250A-01 Rev 1.0, RM-MPU-9250A-00 Rev 1.4 |
| Status | mock-tested |

Notes

* The data-ready interrupt is enabled so the status bit can be polled; INT
  therefore pulses once per sample unless wake-on-motion is enabled, in
  which case only motion asserts INT and `read_measurement()` returns the
  latest sample without a ready check. `motion_detected()` reports motion
  flags even if `read_measurement()` cleared `INT_STATUS` first.
* MPU-6050 `MOT_DUR` (0x20) and `ACCEL_HPF` (`ACCEL_CONFIG[2:0]`) are
  described in the product specification but dropped from register-map
  Rev 4.0; the HPF only feeds the motion detector.
* For SPI call `disable_i2c()` once after reset. The MPU-6050 itself is
  I2C-only.
* Gyroscope sensitivities are the datasheet nominal values (131, 65.5, 32.8,
  16.4 LSB/dps).
