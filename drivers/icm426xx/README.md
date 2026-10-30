# zigbee-icm426xx

TDK InvenSense ICM-42605, ICM-42688-P and ICM-42670-P 6-axis IMUs. Use
cases: vibration, orientation, wake-on-motion.

| | |
|---|---|
| Bus | I2C 0x68 (AP_AD0 low) / 0x69; SPI mode 0/3 (4-wire) |
| ID | `WHO_AM_I` (0x75): 0x42 ICM-42605, 0x47 ICM-42688-P, 0x67 ICM-42670-P |
| Output | acceleration in milli-g, angular rate in milli-dps (`Vector3`) |
| Default | ±2 g, 25 Hz, accelerometer low-power mode (wake-up oscillator), gyroscope off |
| Rates | ICM-42605/42688-P 1.5625 Hz–8 kHz (≤ 6.25 Hz LP only, ≥ 1 kHz LN only); ICM-42670-P 1.5625 Hz–1.6 kHz (≤ 6.25 Hz LP only, ≥ 800 Hz LN only) |
| Sampling | free-running; `read_measurement()` returns `NotReady` until the data-ready status bit |
| Start-up | accelerometer 10 ms, gyroscope 30 ms (typical); 1 ms after soft reset |
| Wake-on-Motion | 1 g/256 (~3.9 mg) per LSB on all three axes (OR), compare to previous or initial sample |
| Sleep | `LowPower::sleep` = sensors and RC oscillator off: 7.5 µA ICM-42605/42688-P, 3.5 µA ICM-42670-P |
| Source | TDK DS-000292 Rev 1.2 (ICM-42605), DS-000347 Rev 1.8 (ICM-42688-P), DS-000451 Rev 1.0 (ICM-42670-P) |
| Status | mock-tested |

Notes

* Register maps: the ICM-42605 and ICM-42688-P share one map and keep the WoM
  thresholds in bank 4 (`REG_BANK_SEL`). The ICM-42670-P uses a different
  bank-0 layout and reaches them through the indirect MREG1 window. Each
  MREG access needs 10 µs of bus silence, `MCLK_RDY` set, and the RC
  oscillator on (`PWR_MGMT0.IDLE`) while the accelerometer runs from the
  wake-up oscillator.
* The WoM flows contain waits: 1 ms and 50 ms on ICM-42605/42688-P, and
  10 µs per MREG write on ICM-42670-P. Arming is therefore split into
  steps. Call `enable_wake_on_motion()`, then wait the returned time and
  call `continue_wake_on_motion()` until it returns 0.
* Datasheet errata in §8.7: the step "SMD_MODE to 1 (Register 0x56h)"
  means `SMD_CONFIG` 0x57, and `SMD_MODE` = 01 ("WOM mode" in the
  ICM-42605 register description). The ICM-42688-P outputs register
  (0x7D) is `INT_STATUS2` 0x37.
* On ICM-42605/42688-P `configure()` clears `INT_CONFIG1.INT_ASYNC_RESET`
  as the datasheet requires, and sets the 8 µs pulse / no-deassert bits for
  4 kHz and 8 kHz ODRs. INT1 is driven push-pull, active high. Data-ready
  is routed to INT1 unless WoM is enabled; while WoM is enabled,
  `read_measurement()` skips the ready check.
* The ±125 dps range exists only on the ICM-42605/42688-P. The 16/32 kHz
  ICM-42688-P rates are not exposed.
* Gyroscope sensitivities are the datasheet nominal values (16.4–262
  LSB/dps). Accelerometer counts are scaled as full scale / 32768.
