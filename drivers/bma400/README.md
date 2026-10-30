# zigbee-bma400

Bosch BMA400 ultra-low-power three-axis accelerometer with wake-up
interrupt, two generic activity/inactivity interrupts and automatic
low-power/normal mode switching. Blocking and async drivers over I2C or SPI.

| | |
|---|---|
| Parts | BMA400 (CHIPID `0x90`) |
| Bus | I2C at `0x14` (SDO low) or `0x15` (SDO high); SPI mode 0/3, read `reg \| 0x80` + 1 dummy byte |
| Ranges | ±2/4/8/16 g, 12-bit (1024/512/256/128 LSB/g) |
| Modes | low-power (fixed 25 Hz), normal 12.5–800 Hz, sleep; no single-shot |
| Current | 0.85–1.35 µA low-power, 3.5–14.5 µA normal, 160 nA sleep |
| Output | milli-g (`Quantity::Acceleration`, truncated toward zero) |
| Interrupts | wake-up (low-power mode, per-axis threshold, 1–8 samples, one-time/every-time reference), gen1/gen2 activity/inactivity (normal mode, 8 mg/LSB, hysteresis, 16-bit duration), auto wake-up / auto low-power; INT1/INT2, latched or not, active high/low |
| Sources | Bosch Sensortec BMA400 Data sheet rev. 2.3 (BST-BMA400-DS000-14) |
| Status | mock-tested |

## Usage on a sleepy device

```rust,ignore
let mut acc = Bma400::new_i2c(i2c, ADDRESS_SDO_LOW);
acc.probe()?;
// ~1 µA at 25 Hz, INT1 on motion > 125 mg.
acc.configure(Config { mode: PowerMode::LowPower, ..Default::default() })?;
acc.configure_wake_up(&WakeUpConfig { threshold_mg: 125, ..Default::default() })?;
// ... MCU sleeps until INT1 ...
let src = acc.read_interrupt_source()?;   // reading INT_STAT0..2 clears latched bits
```

## Notes

* The wake-up interrupt is evaluated only in low-power mode, and the
  generic interrupts only in normal mode. Leaving normal mode clears the
  interrupt status.
* The part starts in I2C mode; the first SPI access only switches the
  interface. `probe()` does a dummy CHIPID read over SPI first.
* The wake-up threshold LSB is range-dependent (2^(2+range)/256 g), so call
  `configure_wake_up` after `configure` when changing the range.
* Turn-on wait is 2/ODR (80 ms in low-power mode). The datasheet does not
  give a separate start-up time.
* Auto timeouts use 2.5 ms/LSB (max 10.2 s).
* FIFO, step counter, orientation, tap, activity-change and self-test are
  not exposed.
