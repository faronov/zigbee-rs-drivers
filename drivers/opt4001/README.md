# zigbee-opt4001

TI OPT4001 (PicoStar and SOT-5X3) and OPT4003-Q1 (visible + near-IR,
USON) high-precision ambient light sensors over I2C.

| | |
|---|---|
| Address | 0x44 (ADDR=GND), 0x45 (VDD), 0x46 (SDA), 0x47 (SCL); PicoStar is fixed at 0x45 |
| ID | `DIDH` 0x121 (OPT4001), 0x221 (OPT4003-Q1) (`probe()`) |
| Mode | forced auto-range one-shot; the part returns to standby by itself |
| Conversion | 0.6–800 ms per channel (OPT4003-Q1 converts two channels); driver waits typ. + 10 % + 0.5 ms Tss + 0.5 ms auto-range reset |
| Standby current | 1.6 µA typ. (OPT4001), 2 µA (OPT4003-Q1) |
| Output | milli-lux (312.5 / 437.5 / 535 µlx per code) + linear codes, IR codes (OPT4003-Q1), counter, overload and threshold flags; every result is CRC-checked |
| Interrupt | low/high threshold in milli-lux, latch, polarity, fault count |
| Source | TI SBOS993A (OPT4001), SBASA69E (OPT4003-Q1) |
| Status | mock-tested |

Pick the `Variant` that matches the package: the lux scale differs.
Datasheet notes: the OPT4001 address table lists ADDR=SCL as 1000101b,
a duplicate of VDD; the OPT4003-Q1 table gives 1000111b (0x47), which the
driver uses. The OPT4003-Q1 register summary shows a 0x121 reset value for
register 0x11, but the field description gives `DIDH` = 0x221.
Thresholds apply to the visible channel only (`THRESHOLD_CH_SEL` left at
its default).
