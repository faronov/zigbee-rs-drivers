# zigbee-ltr390

Lite-On LTR-390UV-01: ambient light and UV (280–430 nm) sensor with a
20-bit ADC, over I2C.

| | |
|---|---|
| Address | 0x53 (fixed) |
| ID | `PART_ID[7:4]` = 0xB (`probe()` returns the revision) |
| Mode | ALS **or** UV per sample (`set_mode()`); enable → wait → check "new data" → burst-read 0x07–0x12 → standby; `with_continuous(true)` keeps measuring |
| Gain / resolution | ×1 / ×3 / ×6 / ×9 / ×18; 13–20 bit (12.5–400 ms) |
| Lux | `0.6 · ALS / (GAIN · INT) · WFAC` (datasheet §7.1) |
| UV index | `UV / (2300 · GAIN/18 · INT/4) · WFAC`, reported as `Quantity::UvIndex` (0.01) |
| Wait | 10 ms wake-up + conversion + 10 % |
| Standby current | 1 µA typ. |
| Output | milli-lux or UVI × 100, raw count, saturation flag |
| Interrupt | 20-bit window on the active channel, persistence 1–16 (`set_interrupt()`) |
| Source | Lite-On LTR-390UV-01 datasheet (BNS-OD-FC002/A4) |
| Status | mock-tested |

The datasheet gives the UV sensitivity only at ×18 / 20 bit (2300
counts/UVI); other settings are scaled linearly with gain and integration
time, which is how Lite-On's formula is normally applied. The 13-bit
setting uses INT = 0.125 (12.5 ms / 100 ms), extrapolated from the table.
