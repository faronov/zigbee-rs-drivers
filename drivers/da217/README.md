# zigbee-da217

MiraMEMS DA217 (and DA218) 14-bit three-axis accelerometer with an active
(slope) interrupt and auto-sleep for motion wake-up. Blocking and async
drivers over I2C or SPI.

| | |
|---|---|
| Parts | DA217 (CHIP_ID `0x01` = `0x13`); DA218 assumed register-compatible |
| Bus | I2C at `0x26` (SA0 low) or `0x27` (SA0 high), CSB tied high; SPI with 16-bit instruction, one byte per transfer, switched to 4-wire by `probe` |
| Ranges | ±2/4/8/16 g; 14/12/10/8-bit (4096 LSB/g at ±2 g, 14-bit) |
| Modes | Normal (continuous, 1–500 Hz), auto-sleep (12.5 Hz while inactive), suspend; no single-shot mode |
| Current | 95 µA normal at 125 Hz, 1 µA suspend |
| Output | milli-g (`Quantity::Acceleration`, truncated toward zero) |
| Interrupts | Active: slope threshold 3.91 mg/LSB at ±2 g, 1–16 sample duration, per-axis enables, INT1 or INT2, non-latched/temporary/latched, push-pull or open-drain |
| Sources | MiraMEMS DA217 datasheet Rev 0.1 (27-Mar-2017) |
| Status | mock-tested |

## Usage on a sleepy device

```rust,ignore
let mut acc = Da217::new_i2c(i2c, ADDRESS_SA0_LOW);
acc.probe()?;
acc.configure(Config { rate: DataRate::Hz7_81, ..Default::default() })?;
acc.configure_motion(&MotionConfig { threshold_mg: 100, ..Default::default() })?;
// ... MCU sleeps until INT1 (active high, push-pull, latched by default) ...
let src = acc.read_interrupt_source()?;   // reads flags, then resets latched ints
```

## Notes

* Suspend mode stops all interrupts. A motion-wake device keeps the part in
  normal mode at a low ODR, or uses `Config::auto_sleep`. The datasheet
  only gives current at 125 Hz.
* The active threshold scales with the range. Reconfigure motion after
  changing the range.
* The datasheet contradicts itself on INT polarity: section 4.3.3 says
  `int_lvl = 0` is active low, while the INT_CONFIG table says active high.
  The driver follows the register table (`active_low: false` writes 0).
* SPI: the datasheet says SPI starts in 3-wire mode but gives SPI_CONFIG a
  reset value of `0x81` (4-wire). `probe` writes `0x81`. It is unclear
  whether multi-byte transfers count addresses up or down, so the driver
  reads one register per transfer. Without a burst read, the MSB and LSB of
  one axis can come from different samples; use a low ODR or `data_ready`.
* NEWDATA_FLAG clears itself before the next sample. When polling with
  `Config::data_ready`, a late read can return `NotReady`.
* The DA218 register map is not public. The driver assumes it matches the
  DA217 (same CHIP_ID).
* Undocumented MODE_BW bits 4 and 3 are written as their reset value (1).
  The I2C watchdog stays enabled with a 1 ms timeout.
* Tap, orientation, freefall, step counter, tilt and FIFO are not exposed.
