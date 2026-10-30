# zigbee-adxl362

Analog Devices ADXL362 micropower three-axis accelerometer with
ultra-low-power wake-up mode, referenced activity/inactivity detection,
linked/loop modes and autosleep. Blocking and async drivers over SPI.

| | |
|---|---|
| Parts | ADXL362 (DEVID_AD `0xAD`, DEVID_MST `0x1D`, PARTID `0xF2`) |
| Bus | SPI mode 0 (CPOL = CPHA = 0), 1–5 MHz; commands `0x0A` write / `0x0B` read |
| Ranges | ±2/4/8 g, 12-bit (1/2/4 mg/LSB) |
| Modes | measurement 12.5–400 Hz (normal / low-noise / ultra-low-noise), wake-up mode ≈ 6 Hz; no single-shot |
| Current | 1.8 µA @ 100 Hz normal, 270 nA wake-up mode, 10 nA standby |
| Output | milli-g (`Quantity::Acceleration`) |
| Interrupts | activity (11-bit threshold, TIME_ACT samples), inactivity (16-bit sample count), absolute or referenced, default/linked/loop, autosleep, AWAKE level output; INT1/INT2, active high/low |
| Sources | Analog Devices ADXL362 Data Sheet Rev. B (Chinese edition) |
| Status | mock-tested |

## Usage on a sleepy device

```rust,ignore
let mut acc = Adxl362::new(spi_device);
acc.probe()?;
// 270 nA: sample ~6×/s, interrupt on referenced motion above 250 mg.
acc.configure(Config { mode: PowerMode::WakeUp, ..Default::default() })?;
acc.configure_wake_up(&WakeUpConfig { threshold_mg: 250, ..Default::default() })?;
// ... MCU sleeps until INT1 ...
let src = acc.read_interrupt_source()?;   // reading STATUS clears ACT/INACT
```

## Notes

* Registers 0x00–0x2C must be written in standby. While measuring, the
  driver writes POWER_CTL to standby before reconfiguring and restores it
  afterwards.
* `read_measurement` reads STATUS in the same burst as the data to check
  DATA_READY. That read also clears latched ACT/INACT events, so the driver
  stores them and merges them into the next `read_interrupt_source`.
* The datasheet gives 4/ODR from the measure command to valid data. In
  wake-up mode the driver waits 4 × 1/6 s; that figure is an assumption.
* In wake-up mode the activity timer (`duration`) is ignored.
* The English datasheet could not be downloaded, so the driver was written
  from the Chinese edition of Rev. B.
* FIFO, temperature, external clock/trigger and self-test are not exposed.
