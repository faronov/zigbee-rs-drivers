# zigbee-bmi270

Bosch BMI270 accelerometer and gyroscope. Use cases: vibration, tilt and
wake-on-motion.

| | |
|---|---|
| Bus | I2C 0x68 (SDO low) / 0x69 (SDO high); SPI mode 0/3 (one dummy byte per read) |
| ID | `CHIP_ID` 0x24 |
| Output | acceleration in milli-g, angular rate in milli-dps (`Vector3`) |
| Default | ±2 g, 25 Hz, accelerometer normal (performance filter), gyroscope off |
| Sampling | free-running; `read_measurement()` returns `NotReady` until `STATUS.drdy_acc` |
| Start-up | first `configure()`: soft reset 2 ms → APS off 450 µs → 8 KB upload → 20 ms init; then accelerometer 2 ms / gyroscope 45 ms |
| Low-power accel | `acc_filter_perf` = 0 + advanced power save, 0.78–1600 Hz, 1–128× averaging, down to 4 µA |
| Motion interrupt | any-motion feature on INT1, all axes, 1 g / 2048 per LSB (0.49 mg), 20 ms per duration LSB; needs ODR ≥ 50 Hz in low-power mode |
| Sleep | `LowPower::sleep` = suspend (sensors off, advanced power save), 3.5 µA; configuration file kept. Accelerometer normal 210 µA, both 685 µA |
| Source | BST-BMI270-DS000-08; Bosch BMI270 SensorAPI v2.86.1 (configuration file, feature defaults) |
| Status | mock-tested |

Notes

* The configuration file (`bmi270_config_file`, 8192 bytes) is copied from
  Bosch's BSD-3-Clause SensorAPI into `src/config_file.rs` with the original
  notice and costs 8 KB of flash. It is behind the default cargo feature
  `config-blob`; with `default-features = false` pass your own copy (e.g.
  from external flash mapped memory) to `with_config_file()`. Binaries that
  include it must reproduce the BSD-3-Clause notice.
* The file is uploaded in bursts of `with_burst_len()` bytes (default 256,
  using `INIT_ADDR_0/1`), so the interface trait has an extra `write_regs`
  (burst write); I2C uses a two-write `transaction`, which embedded-hal
  sends without a repeated start.
* Configuration is stepped: `configure()` returns a wait, then call
  `continue_configure()` (or `start_measurement()`) after each wait until it
  returns 0. The upload happens only once after reset; later
  `configure()`/`wake()` calls only switch advanced power save off (450 µs)
  and rewrite the registers. `Error::InvalidData` if `INTERNAL_STATUS` does
  not report `init_ok`.
* With advanced power save on (after reset, low-power and suspend) writes
  need 450 µs gaps; the driver only writes registers with it off and enables
  it last.
* `motion_detected()` reads `INT_STATUS_0`, which clears on read and
  releases a latched INT1.
* Gyroscope sensitivity is exactly full scale / 32768 (16.384 LSB/dps at
  ±2000 dps).
