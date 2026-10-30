# zigbee-mc3419

MEMSIC MC3419 / MC3479 16-bit three-axis accelerometers with an AnyMotion
interrupt for motion wake-up. Blocking and async drivers over I2C or SPI.

| | |
|---|---|
| Parts | MC3419, MC3479 (CHIP_ID `0x18` = `0xA4`; select the part with `Config::part`) |
| Bus | I2C at `0x4C` (DOUT_A6 low) or `0x6C` (DOUT_A6 high); SPI 4-wire, read `0x80 \| reg` + one zero byte |
| Ranges | ±2/4/8/12/16 g, 16-bit (16384/8192/4096/2730/2048 LSB/g) |
| Modes | WAKE (continuous; MC3419 25–1000 Hz, MC3479 50–2000 Hz), STANDBY; no single-shot or low-power sampling mode |
| Current | 77 µA in WAKE at 100 Hz, 4 µA STANDBY |
| Output | milli-g (`Quantity::Acceleration`, truncated toward zero) |
| Interrupts | AnyMotion (15-bit threshold, debounce, latched or auto-clear); INTN1 or INTN2 (request swap), open-drain/push-pull, active low/high |
| Sources | MEMSIC MC3419 APS-048-0071 v1.1; MEMSIC MC3479 APS-048-0072 v1.12 |
| Status | mock-tested |

## Usage on a sleepy device

```rust,ignore
let mut acc = Mc3419::new_i2c(i2c, ADDRESS_A6_LOW);
acc.probe()?;
acc.configure(Config { rate: DataRate::Hz25, ..Default::default() })?;
acc.configure_motion(&MotionConfig { threshold_mg: 100, ..Default::default() })?;
// ... MCU sleeps until INTN1 (active low, open-drain by default) ...
let src = acc.read_interrupt_source()?;   // reads and clears INTR_STAT
```

## Notes

* These are not low-power parts: motion detection needs WAKE state, and
  WAKE draws tens of µA. STANDBY (4 µA) stops all detection.
* Only CHIP_ID `0xA4` in the MC3479 datasheet is documented; the MC3419
  datasheet marks 0x18 reserved. The driver checks `0xA4` for both and
  cannot tell them apart, so the part must be given in `Config`.
* All registers except MODE and INTR_STAT are writable only in STANDBY.
  While sampling, `configure_motion`/`disable_motion` drop to STANDBY and
  return to WAKE afterwards.
* The datasheets give no unit for the AnyMotion threshold. The driver
  assumes output-data LSBs at the configured range, so reconfigure motion
  after changing the range. The debounce register is passed through as
  raw samples.
* MC3419 sample-rate codes are "Selection A" (I2C, or SPI at 4 MHz or
  less). Keep MC3419 SPI at 4 MHz or less.
* Both INTN pins get the same drive/polarity. Moving the motion interrupt
  to INTN2 swaps the request lines, so FIFO interrupts move to INTN1.
* Clearing interrupts works only in WAKE state.
* Tilt/flip, shake, tilt-35, FIFO, decimation and offset/gain trim are not
  exposed.
