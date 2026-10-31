# zigbee-max44009

Analog Devices (Maxim) MAX44009 ambient light sensor, 0.045 lx to 188 klx,
over I2C.

| | |
|---|---|
| Address | 0x4A (A0 = GND) / 0x4B (A0 = VCC) |
| ID | none; `probe()` is a presence read of `CONFIG` |
| Mode | always measuring (continuous only). Default: auto-range, one conversion per 800 ms. `with_continuous(true)` converts back-to-back; `with_manual(Some(it))` fixes the integration time. `start_measurement` writes `CONFIG` once and returns one period; later calls return 0 |
| Gain / IT | automatic, or manual 6.25–800 ms |
| Lux | `2^E · M · 0.045` (4-bit exponent, 8-bit mantissa); E = 15 means overrange |
| Wait | 800 ms (auto) or the manual integration time, + 10 %; only after a (re)configuration |
| Standby current | no shutdown; `sleep()` restores the default 800 ms mode (0.65 µA typ.) |
| Output | milli-lux, raw exponent and mantissa, overrange flag |
| Interrupt | upper/lower lux thresholds (exponent + upper mantissa nibble) and a 100 ms-step threshold timer (`set_interrupt()`, `take_interrupt()`) |
| Helper | `threshold_byte()`, `threshold_millilux()`, `to_millilux()` |
| Source | Maxim MAX44009 datasheet register map, cross-checked against Linux `iio/light/max44009.c` and ESPHome `max44009` |
| Status | mock-tested |

Notes:

* The datasheet PDF could not be fetched while writing this crate. Register
  addresses, bit positions, the threshold encoding (upper threshold low
  nibble reads as 0xF) and the integration-time table match the Linux and
  ESPHome drivers.
* The part has no data-ready flag. `read_measurement` returns the most
  recent completed conversion.
* The two lux bytes are read in one I2C transaction with a repeated START,
  as Linux does, so the exponent and mantissa come from the same
  conversion.
* The `CDR` (current division) bit is left at 0.
