# zigbee-adxl367

Analog Devices ADXL367 nanopower three-axis accelerometer with
ultra-low-power wake-up mode, referenced activity/inactivity detection,
linked/loop modes, autosleep and single/double tap. Blocking and async
drivers over I2C or SPI.

| | |
|---|---|
| Parts | ADXL367 (DEVID_AD `0xAD`, DEVID_MST `0x1D`, PART_ID `0xF7`) |
| Bus | I2C at `0x53` (ASEL high) or `0x1D` (ASEL low); SPI mode 0, 1–8 MHz, commands `0x0A` write / `0x0B` read |
| Ranges | ±2/4/8 g, 14-bit (0.25/0.5/1 mg/LSB) |
| Modes | measurement 12.5–400 Hz (normal / low-noise), wake-up mode at 80/160/320/640 ms intervals; no single-shot |
| Current | 0.89 µA @ 100 Hz normal, 180 nA wake-up mode (640 ms), 40 nA standby |
| Output | milli-g (`Quantity::Acceleration`, truncated toward zero) |
| Interrupts | activity (13-bit threshold, TIME_ACT samples), inactivity (16-bit sample count), absolute or referenced, default/linked/loop, autosleep, AWAKE level output, single/double tap; INT1/INT2, active high/low |
| Sources | Analog Devices ADXL367 Data Sheet Rev. A |
| Status | mock-tested |

## Usage on a sleepy device

```rust,ignore
let mut acc = Adxl367::new_i2c(i2c, ADDRESS_ASEL_HIGH);
acc.probe()?;
// ~180 nA: one sample every 640 ms, interrupt on referenced motion > 250 mg.
acc.configure(Config {
    mode: PowerMode::WakeUp,
    wake_up_rate: WakeUpRate::Ms640,
    ..Default::default()
})?;
acc.configure_wake_up(&WakeUpConfig { threshold_mg: 250, ..Default::default() })?;
// ... MCU sleeps until INT1 ...
let src = acc.read_interrupt_source()?;   // reading STATUS clears ACT/INACT
```

## Notes

* Registers 0x00–0x2D must be written in standby. While measuring, the
  driver writes POWER_CTL to standby before reconfiguring and restores it
  afterwards. Tap, timer and upper interrupt-map registers are written
  directly.
* `read_measurement` reads STATUS in the same burst as the data to check
  DATA_READY. That read also clears latched ACT/INACT events, so the driver
  stores them and merges them into the next `read_interrupt_source`.
* The turn-on wait is the 100 ms settling time from "Operating Modes"
  (Table 1 gives 31 ms typical). In wake-up mode the driver waits at least
  one interval plus 10 ms for the first sample.
* In wake-up mode the activity timer (`duration`) is ignored, and low-noise
  mode cannot be combined with wake-up mode.
* The datasheet's motion-switch routine labels `0x0F/0xA0` as 62.5 mg; at
  ±2 g that code is 250 mg. The driver follows the 0.25 mg/LSB scale.
* FILTER_CTL.I2C_HS is left at its reset value (1).
* FIFO, temperature/ADC, keep-alive timer, external clock/trigger and
  self-test are not exposed.
