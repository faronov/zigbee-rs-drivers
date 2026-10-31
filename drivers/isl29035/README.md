# zigbee-isl29035

Renesas (Intersil) ISL29035 ambient light sensor with a 16-bit ADC and
four ranges, over I2C.

| | |
|---|---|
| Address | 0x44 (fixed) |
| ID | `ID` bits 5:3 = 0b101; `probe()` also clears the brown-out (`BOUT`) flag |
| Mode | continuous ALS → wait → read `DATA` → power down (`with_continuous(true)` keeps measuring) |
| Range / IT | 1k/4k/16k/64k lux; 16 bit (105 ms), 12 bit (6.5 ms), 8 bit (0.41 ms), 4 bit (25.6 µs) |
| Lux | `Range / 2^n · DATA` (Eq. 3) |
| Wait | integration time + 10 % |
| Standby current | 0.24 µA typ. (power-down) |
| Output | milli-lux, raw count, saturation flag |
| Interrupt | 16-bit window, persistence 1/4/8/16 (`set_interrupt()`); `take_interrupt()` reads `COMMAND-I`, which clears the flag |
| Helper | `suggest_range()` (auto-range), `to_millilux()` |
| Source | Renesas FN8371 datasheet |
| Status | mock-tested |

Notes:

* "ALS once" (OP = 001) is not used, because the datasheet does not
  recommend it (Table 7, note 11). The driver uses continuous mode and
  powers the part down after the read. There is no data-ready flag.
* The IR channel and the IR-compensated EV formula (Eq. 5) are not
  exposed, because its β constant depends on the light source.
* The INT pin is always active. `set_interrupt(None, …)` only restores
  the reset window.
