# zigbee-lis2du12

ST LIS2DU12 ultra-low-power three-axis accelerometer with anti-aliasing
filter, one-shot conversion, wake-up, activity/inactivity and 6D interrupts.
Blocking and async drivers over I2C or SPI.

| | |
|---|---|
| Parts | LIS2DU12 (WHO_AM_I `0x45`) |
| Bus | I2C `0x18` (SA0 low) / `0x19` (SA0 high), up to 1 MHz; SPI mode 0/3, 4-wire |
| Ranges | ±2/4/8/16 g, 12-bit |
| Modes | ultralow-power 1.6/3/6 Hz; normal (anti-aliasing) 6–800 Hz; one-shot |
| Conversion | one-shot via `CTRL4.SOC` (default) or continuous |
| Current | 0.2 µA one-shot, 0.47 µA @ 1.6 Hz ULP, 6.1 µA @ 800 Hz; 20 nA power-down |
| Output | milli-g (`Quantity::Acceleration`) |
| Interrupts | wake-up (threshold mg, FS/256 or FS/64 steps chosen automatically, duration 0–3 /ODR, INT1 or INT2, latch), activity/inactivity with inactive ODR, 6D/4D |
| Sources | ST LIS2DU12 datasheet DS12422 Rev 4 (November 2022) |
| Status | mock-tested |

## Usage on a sleepy device

```rust,ignore
let mut acc = Lis2du12::new_i2c(i2c, ADDRESS_SA0_LOW);
acc.probe()?;                                 // also enables IF_ADD_INC
// Periodic reading only: one-shot, 0.2 µA between samples.
acc.configure(Config::default())?;
let m = acc.measure(&mut delay)?;

// Wake on motion: continuous ULP at 1.6 Hz (0.47 µA) + wake-up interrupt.
acc.configure(Config { conversion: Conversion::Continuous, ..Default::default() })?;
acc.configure_wake_up(&WakeUpConfig { threshold_mg: 50, ..Default::default() })?;
let src = acc.read_interrupt_source()?;       // reading ALL_INT_SRC clears latches
```

## Notes

* DS12422 does not give the one-shot conversion time or the power-up time
  in continuous mode. The driver returns `Config::one_shot_wait_us`
  (default 5 ms) for one-shot and two sample periods for continuous start;
  `read_measurement` returns `NotReady` until `STATUS.DRDY` is set. Tune
  `one_shot_wait_us` on hardware.
* `CTRL1.IF_ADD_INC` is 0 after reset; `probe()` and `configure()` set it
  (burst reads are wrong until one of them ran).
* Latch (`LIR` + `INT_SHORT_EN`) is shared by every interrupt function.
* Interrupt functions need continuous conversion. `LowPower::sleep()`
  writes ODR = 0000.
