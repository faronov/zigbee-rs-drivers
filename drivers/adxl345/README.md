# zigbee-adxl345

Analog Devices ADXL343 / ADXL345 three-axis accelerometer with activity,
inactivity, link/auto-sleep and tap interrupts. Blocking and async drivers
over I2C or SPI.

| | |
|---|---|
| Parts | ADXL345, ADXL343 (DEVID `0xE5`, identical register map) |
| Bus | I2C `0x53` (ALT ADDRESS low) / `0x1D` (high); SPI 4-wire, mode 3, ≤ 5 MHz |
| Ranges | ±2/4/8/16 g, full resolution (256 LSB/g) |
| Modes | continuous 0.10 Hz–3.2 kHz, reduced-power mode at 12.5–400 Hz, auto-sleep at 8/4/2/1 Hz; no single-shot |
| Current | 23 µA ≤ 0.78 Hz, 34 µA @ 12.5 Hz reduced power; 0.1 µA standby |
| Output | milli-g (`Quantity::Acceleration`) |
| Interrupts | activity (62.5 mg/LSB, DC or AC/referenced), inactivity (threshold + 1–255 s), link, auto-sleep, single/double tap; INT1/INT2 routing, active high/low; always latched |
| Sources | Analog Devices ADXL345 Data Sheet Rev. E; ADXL343 Data Sheet Rev. 0 |
| Status | mock-tested |

## Usage on a sleepy device

```rust,ignore
let mut acc = Adxl345::new_i2c(i2c, ADDRESS_ALT_LOW);
acc.probe()?;
acc.configure(Config::default())?;            // 12.5 Hz reduced power, ±2 g
acc.configure_wake_up(&WakeUpConfig {
    threshold_mg: 250,
    inactivity: Some(InactivityConfig { threshold_mg: 125, time_s: 10, referenced: true }),
    auto_sleep: Some(SleepRate::Hz1),           // 1 Hz while still
    ..Default::default()
})?;
// ... MCU sleeps until INT1 ...
let src = acc.read_interrupt_source()?;       // reading INT_SOURCE clears the latch
```

## Notes

* ADXL343 and ADXL345 report the same DEVID and cannot be told apart.
* Interrupts are always latched; they are released by reading
  `INT_SOURCE`. `read_measurement` reads `INT_SOURCE` in the same burst
  as the data (to check `DATA_READY`), so motion events seen there are
  stored and merged into the next `read_interrupt_source`. The pin is
  already released at that point.
* While auto-sleep has the part asleep `DATA_READY` is suppressed and
  `read_measurement` returns `NotReady`.
* Activity has no duration filter; one sample above the threshold
  triggers.
* `LowPower::sleep()` is standby (no detection). For wake-on-motion keep
  the part measuring at a low rate instead.
* FIFO, free-fall, offsets and self-test are not exposed.
