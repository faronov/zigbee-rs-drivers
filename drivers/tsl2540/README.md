# zigbee-tsl2540

ams-OSRAM TSL2540: 1.8 V ambient light sensor with photopic (visible) and
IR channels, 2.81 ms – 721 ms integration and ×½ – ×128 gain.

| | |
|---|---|
| Address | 0x39 (fixed) |
| ID | `ID[7:2]` = 0xE4 (`init()`) |
| Mode | one-shot via sleep-after-interrupt: `APERS = 0`, `AIEN`, `SAI` → `PON | AEN` → oscillator stops after one cycle, `AINT` = data ready → read → `PON = 0`; or `with_continuous(true)` |
| Gain / IT | ×½, ×1, ×4, ×16, ×64 (67), ×128 (140); ATIME 0–255 → 2.81–721 ms (full scale 1024 per step, max 65535) |
| Lux | `VIS · DF / (t_ms · gain)`, DF = 53 by default (`with_device_factor()` for cover glass) |
| Wait | 3 ms auto-zero allowance + (ATIME + 1) × 2.90 ms |
| Sleep current | 0.7 µA typ. |
| Output | milli-lux, raw visible / IR, saturation flag (ASAT or full scale) |
| Interrupt | continuous mode: visible-channel window + persistence (`set_thresholds()`), flags cleared on read |
| Helper | `suggest_gain()`, `atime_for_ms()` |
| Source | ams-OSRAM DS000564 v2-00 (2022-12-05); DF = 53 from the Zephyr `tsl2540` driver |
| Status | mock-tested |

The datasheet publishes no lux equation ("custom lux equations can be
created") and no data-valid flag; the auto-zero duration before the first
cycle is not specified (3 ms is allowed; `read_measurement` returns
`NotReady` until `AINT` is set). In one-shot mode `INT` asserts once per
sample until the read clears it.
