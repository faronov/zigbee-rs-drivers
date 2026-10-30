# zigbee-lis2dw12

ST LIS2DW12 ultra-low-power three-axis accelerometer with single data
conversion on demand, wake-up, activity/inactivity and 6D interrupts.
Blocking and async drivers over I2C or SPI.

| | |
|---|---|
| Parts | LIS2DW12 (WHO_AM_I `0x44`) |
| Bus | I2C `0x18` (SA0 low) / `0x19` (SA0 high); SPI mode 0/3, 4-wire, ≤ 10 MHz |
| Ranges | ±2/4/8/16 g |
| Modes | low-power 1 (12-bit), low-power 2–4 (14-bit), 1.6–200 Hz; high-performance 14-bit, 12.5 Hz–1.6 kHz |
| Conversion | on demand (bus trigger, T_on 1.2/1.7/2.3/3.55 ms for LP1–4) or continuous |
| Current | 0.38 µA @ 1.6 Hz, 1 µA @ 12.5 Hz (LP1); 50 nA power-down |
| Output | milli-g (`Quantity::Acceleration`) |
| Interrupts | wake-up (threshold mg in FS/64 steps, duration 0–3 /ODR, INT1, latch), activity/inactivity auto-sleep (sleep-change on INT2), 6D/4D on INT1 |
| Sources | ST LIS2DW12 datasheet DocID029682 Rev 4 |
| Status | mock-tested |

## Usage on a sleepy device

```rust,ignore
let mut acc = Lis2dw12::new_i2c(i2c, ADDRESS_SA0_LOW);
acc.probe()?;
// Periodic reading only: on-demand conversion, part is powered down between samples.
acc.configure(Config::default())?;
let m = acc.measure(&mut delay)?;            // start → wait T_on → read

// Wake on motion: continuous LP1 at 12.5 Hz (1 µA) + INT1 wake-up.
acc.configure(Config { conversion: Conversion::Continuous, ..Default::default() })?;
acc.configure_wake_up(&WakeUpConfig { threshold_mg: 63, ..Default::default() })?;
let src = acc.read_interrupt_source()?;       // reading ALL_INT_SRC clears latches
```

## Notes

* The wake-up event can only be routed to INT1. INT2 carries the
  activity/inactivity *change* (`INT2_SLEEP_CHG`) and therefore needs
  `WakeUpConfig::auto_sleep`.
* `CTRL3.LIR` (latch) is shared by every interrupt function.
* Interrupt functions need continuous conversion; in on-demand mode the part
  is powered down between triggers. `LowPower::sleep()` writes ODR = 0.
* The datasheet gives no power-up time for continuous mode; the driver waits
  T_on + 1/ODR (low-power) or 2/ODR (high-performance, Table 14), and
  `read_measurement` returns `NotReady` until `STATUS.DRDY` is set.
* The driver keeps `CTRL2.IF_ADD_INC` set (its reset value) for burst reads.
