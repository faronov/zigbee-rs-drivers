# zigbee-kxtj3

Kionix KXTJ3-1057 three-axis accelerometer with an 8-bit low-power mode and
a wake-up (motion detect) function for sleepy devices. Blocking and async
drivers over I2C.

| | |
|---|---|
| Parts | KXTJ3-1057 (WHO_AM_I `0x0F` = `0x35`) |
| Bus | I2C at `0x0E` (ADDR to GND) or `0x0F` (ADDR to IO_VDD) |
| Ranges | ±2/4/8/16 g; 8-bit low power, 12-bit high resolution, 14-bit at ±8/16 g |
| Modes | Operating (continuous, 0.781–1600 Hz; low power only up to 200 Hz), stand-by; no single-shot mode |
| Current | ~10 µA low power, 155 µA high resolution, 0.9 µA stand-by |
| Output | milli-g (`Quantity::Acceleration`, truncated toward zero) |
| Interrupts | Wake-up: 3.9 mg threshold, own 0.781–100 Hz rate, motion and non-activity counters, per-direction enables; latched, pulsed or unlatched; single push-pull INT pin, active high or low |
| Sources | Kionix KXTJ3-1057 Specifications Rev. 4.0 (05-Oct-2018) |
| Status | mock-tested |

## Usage on a sleepy device

```rust,ignore
let mut acc = Kxtj3::new(i2c, ADDRESS_ADDR_LOW);
acc.probe()?;
acc.configure(Config { rate: DataRate::Hz0_781, ..Default::default() })?;
acc.configure_wake_up(&WakeUpConfig { threshold_mg: 100, ..Default::default() })?;
// ... MCU sleeps until INT (active high, latched by default) ...
let src = acc.read_interrupt_source()?;   // reads INT_REL, releasing INT
```

## Notes

* The wake-up function only runs in operating mode (`PC1 = 1`).
  `LowPower::sleep` (stand-by) stops it as well, so a motion-wake device
  keeps the part operating at a low ODR in low-power mode.
* Control registers are writable only with `PC1 = 0`. While operating,
  `configure_wake_up`/`disable_wake_up` drop to stand-by and return to
  operating mode afterwards.
* The wake-up threshold is range-independent (256 counts/g, 12 bits).
  `WAKEUP_COUNTER` and `NA_COUNTER` count wake-up periods and must be at
  least 1; zero gives `Error::Unsupported`.
* `Config::data_ready` enables DRDYE so reads can return `NotReady`. With
  the INT pin enabled this also signals every sample on the same pin, so
  it is off by default.
* The turn-on time figure in the datasheet is unclear (about 1/ODR); the
  driver waits 1/ODR + 1 ms. The part does not answer I2C for 12–30 ms
  after power-up.
* Self-test (DCST, SELF_TEST) and software reset are not exposed.
