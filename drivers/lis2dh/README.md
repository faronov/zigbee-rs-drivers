# zigbee-lis2dh

ST LIS2DH12 / LIS2DH / LIS3DH and the Silan SC7A20 clone: three-axis
accelerometers with programmable motion (wake-up) and 6D/4D orientation
interrupts. Blocking and async drivers over I2C or SPI.

| | |
|---|---|
| Parts | LIS2DH12, LIS2DH, LIS3DH (WHO_AM_I `0x33`), SC7A20 (WHO_AM_I `0x11`) |
| Bus | I2C `0x18` (SA0 low) / `0x19` (SA0 high, internal pull-up); SPI mode 0/3, 4-wire, ≤ 10 MHz |
| Ranges | ±2/4/8/16 g |
| Modes | low-power 8-bit, normal 10-bit, high-resolution 12-bit; 1 Hz … 5.376 kHz |
| Conversion | free-running; turn-on 1/ODR + 1 ms (LP), + 1.6 ms (normal), 7/ODR (HR) |
| Current | 2 µA @ 1 Hz LP, 3 µA @ 10 Hz LP, 0.5 µA power-down (LIS2DH12) |
| Output | milli-g (`Quantity::Acceleration`) |
| Interrupts | wake-up (threshold mg, duration in 1/ODR, INT1/INT2, latch, high-pass), 6D/4D orientation |
| Sources | ST LIS2DH12 DocID025056 Rev 6; ST LIS3DH Doc ID 17530 Rev 1; Silan SC7A20 datasheet v0.4 |
| Status | mock-tested |

## Usage on a sleepy device

```rust,ignore
let mut acc = Lis2dh::new_i2c(i2c, ADDRESS_SA0_LOW);
acc.probe()?;                                   // selects ST / SC7A20 scaling
acc.configure(Config { mode: Mode::LowPower, rate: DataRate::Hz10, ..Default::default() })?;
acc.configure_wake_up(&WakeUpConfig { threshold_mg: 80, duration: 1, ..Default::default() })?;
// MCU sleeps; INT1 (active high, latched) wakes it.
let src = acc.read_interrupt_source(IntPin::Int1)?; // clears the latch
```

`LowPower::sleep()` is a full power-down; the wake-up detector only runs
while the part is converting, so keep it in low-power mode at 1–10 Hz for
wake-on-motion.

## Notes

* WHO_AM_I cannot tell LIS2DH12 from LIS3DH: both use the ST tables. The
  SC7A20 differs at ±16 g (8 mg/digit instead of 12, threshold step 128 mg
  instead of 186 mg) and in the CTRL_REG6 bit that routes generator 2 to
  INT2 (bit 6 instead of bit 5). Call `probe()` or `with_variant()`.
* Each pin uses its own interrupt generator (INT1 → IA1, INT2 → IA2): put
  the wake-up interrupt on one pin and the orientation interrupt on the
  other to use both.
* Wiring: tie CS high for I2C. INT pins are push-pull; polarity selectable
  with `Config::int_active_low`.
