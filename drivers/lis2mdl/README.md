# zigbee-lis2mdl

ST LIS2MDL and IIS2MDC 3-axis magnetometers (same register map). Use
cases: door/window contact with a magnet, utility meter dial reading,
orientation.

| | |
|---|---|
| Bus | I2C 0x1E (fixed); SPI mode 3 — the part starts in 3-wire mode, `new_spi()` enables 4-wire (`CFG_REG_C.4WSPI`) before the first read |
| ID | `WHO_AM_I` 0x40 (both parts) |
| Output | magnetic field in nT (`Vector3`), 1.5 mgauss = 150 nT/LSB, ±50 gauss |
| Default | high resolution, idle between single measurements, temperature compensation on (required), BDU |
| Sampling | single measurement per `start_measurement()`; `read_measurement()` returns `NotReady` until `STATUS_REG.ZYXDA` |
| Conversion | 9.4 ms high resolution / 6.4 ms low power; + 1/ODR with offset cancellation (Table 12) |
| Offset | optional offset cancellation (`OFF_CANC` + `OFF_CANC_ONE_SHOT` in single mode); hard-iron offset registers via `set_hard_iron_offset()` |
| Threshold interrupt | 16-bit unsigned threshold (±, 150 nT/LSB) on selected axes, raw or hard-iron corrected data, INT pin push-pull, latched or pulsed; runs the part continuously at 10–100 Hz |
| Current | 1.5 µA idle; continuous 25 µA (LP) / 100 µA (HR) at 10 Hz, ×2.4–1.2 with offset cancellation; single mode below 10 Hz scales linearly |
| Sleep | `LowPower::sleep` = idle (`MD` = 11); `wake()` restarts continuous mode if a threshold is enabled |
| Source | LIS2MDL datasheet DocID030621 Rev 5 (Nov 2018); IIS2MDC registers cross-checked with ST `iis2mdc_reg.h` |
| Status | mock-tested |

Notes

* For I2C tie CS high. Burst reads set bit 7 of the sub-address; SPI reads
  set bit 7 of the address and auto-increment.
* In 4-wire SPI mode pin 7 is SDO, so data-ready and the threshold
  interrupt are only visible through `STATUS_REG` / `INT_SOURCE_REG`.
* Single mode is independent of the ODR, but offset cancellation adds
  1/ODR to the measurement, so pick 100 Hz for the shortest wait.
  A user report on the ST community (with datasheet Rev 6) says single
  measurements never complete with `OFF_CANC` + `OFF_CANC_ONE_SHOT`; the
  driver follows the datasheet but leaves offset cancellation off by
  default — if `read_measurement()` keeps returning `NotReady`, disable it.
* The temperature output (8 LSB/°C, offset not specified in the
  datasheet) and self-test are not used.
