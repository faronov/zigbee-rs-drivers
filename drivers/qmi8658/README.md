# zigbee-qmi8658

QST QMI8658A and QMI8658C accelerometer and gyroscope. Use cases:
vibration, tilt and wake-on-motion.

| | |
|---|---|
| Bus | I2C 0x6A (SA0 high/open) / 0x6B (SA0 low); 4-wire SPI mode 0/3 |
| ID | `WHO_AM_I` 0x05 (both variants; `REVISION_ID` 0x7C on the A) |
| Output | acceleration in milli-g, angular rate in milli-dps (`Vector3`) |
| Default | QMI8658A, ±2 g, 31.25 Hz, accelerometer normal mode, gyroscope off |
| Sampling | free-running; `read_measurement()` returns `NotReady` until `STATUS0.aDA` |
| Start-up | first `configure()`: soft reset 15 ms (A) / 1.75 s (C); then accelerometer 3 ms + 3/ODR, gyroscope 150 ms (A) / 60 ms (C) + 3/ODR |
| Low-power accel | `aODR` 128/21/11/3 Hz duty-cycled (55/42/35/30 µA), gyroscope off; normal accelerometer 132–182 µA, 6-axis ≈ 750 µA |
| Motion interrupt | wake-on-motion via `CTRL9` command, threshold 1 mg/LSB, INT1 or INT2 with selectable initial level, 0–63 blanking samples; pin toggles per event |
| Sleep | `LowPower::sleep` = sensors off, slow clock (8 µA, fast restart); `power_down()` = oscillator off (6 µA, next `configure()` soft-resets) |
| Source | QMI8658A Datasheet Rev A (13-52-25, 2022); QMI8658C Datasheet Rev 0.6 (Jan 2021) |
| Status | mock-tested |

Notes

* The variants cannot be told apart by `WHO_AM_I`; pick one with
  `with_variant()`. They differ in the `CTRL9` handshake (A: `STATUSINT`
  bit 7 + `CTRL_CMD_ACK`, selected via `CTRL8`; C: `STATUS1` bit 0, cleared
  on read), INT pin enables (A: `CTRL1` bits 3/4, pins high-Z by default)
  and turn-on times.
* The datasheets advise against resetting except to reboot the part, so only
  the first `configure()` (and the first after `power_down()`) resets;
  later calls disable the sensors, rewrite `CTRL1/2/3(/8)` and re-enable
  them. `CTRL1` selects auto-increment and little-endian reads.
* Changing the wake-on-motion setting runs `CTRL_CMD_WRITE_WOM_SETTING`;
  `continue_configure()` polls completion every 1 ms and returns `Timeout`
  after 50 polls. Disabling writes threshold 0 as the datasheet requires.
* In wake-on-motion mode the part produces no samples, so
  `read_measurement()` returns `NotReady`; WoM needs the gyroscope off
  (`Unsupported` otherwise) and is normally combined with a low-power rate.
  `motion_detected()` reads `STATUS1`, which clears the flag and resets the
  pin to its initial level. `sleep()` stops the accelerometer and therefore
  WoM.
* With the gyroscope on, the QMI8658A derives all rates from the gyroscope
  drive frequency (e.g. 28.025 Hz for the 31.25 Hz setting), and 8/4/2 kHz
  accelerometer-only rates are not available on it.
* The attitude engine (C), motion engines (A), FIFO, temperature and the
  external magnetometer master are not used.
