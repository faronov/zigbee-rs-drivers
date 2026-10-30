# zigbee-si1145

Silicon Labs Si1145/46/47 UV index, ambient light (visible + IR) and
proximity sensor with irLED drivers, over I2C.

| | |
|---|---|
| Address | 0x60 (default) |
| ID | `PART_ID` 0x45 / 0x46 / 0x47 (`probe()` returns `Part`) |
| Mode | forced conversion (`MEAS_RATE = 0`): `NOP` → `ALS_FORCE` / `PS_FORCE` / `PSALS_FORCE` → wait → check `RESPONSE` → burst-read 0x22–0x2D |
| Gain / IT | ALS VIS and IR: `ADC_GAIN` 0–7 (25.6 µs × 2^gain), high range ÷14.5; PS1: gain 0–5, LED1 current code 1–15, high range |
| Lux | estimate: `(VIS − 256) / (0.282 counts/lx · 2^gain)`, × 14.5 in high range; no IR compensation (`LuxCal` adjustable) |
| UV | `AUX_DATA` = 100 × UVI with datasheet `UCOEF` 0x29/0x89/0x02/0x00 → `Quantity::UvIndex` |
| Wait | (285 µs ALS group + 155 µs PS1 + 25.6 µs × (2^gain − 1) per channel) + 10 % + 1 ms |
| Standby current | 150 nA typ. at 1.8 V (sequencer sleeps between forced commands) |
| Output | milli-lux estimate, UV index (0.01), raw VIS/IR, raw PS1 (`Quantity::Proximity`), saturation flag |
| Interrupt | conversion-complete INT (`with_interrupt()`); `take_interrupt()` reads and clears `IRQ_STATUS` (no thresholds on this part) |
| Helper | `suggest_adc()` (auto-range), `to_millilux()`, `set_param()`, `set_visible()` / `set_ir()` |
| Source | Silicon Labs Si1145/46/47 datasheet Rev. 1.1 |
| Status | mock-tested |

Notes:

* Call `reset()`, wait the returned 25 ms and then `init()` after power-up.
  `init()` writes `HW_KEY` and the parameter RAM. `CHLIST` must be
  non-zero before any forced conversion.
* Commands use the datasheet handshake: `NOP`, poll `RESPONSE` until it
  reads 0, send the command, poll until it is non-zero. Polls are bounded
  (`with_poll_limit`, default 200) because the driver has no delay, and
  return `Timeout` when the bound is exceeded. `INVALID_SETTING` (0x80)
  returns `InvalidData`. ADC overflow codes (0x88–0x8E) set `saturated`
  and are cleared by the next `NOP`.
* The datasheet gives no lux formula, so illuminance is an uncalibrated
  estimate based on the sunlight sensitivity. The 256-count dark offset is
  the typical Si114x ADC offset and is not stated in this datasheet. Set
  `LuxCal` per design.
* The datasheet `UCOEF` defaults are used. The Linux driver uses different
  values (0x7B/0x6B/0x01/0x00). Factory calibration (`GET_CAL`, per-part
  `UCOEF` correction) is not implemented.
* `MEAS_RATE0` is also cleared at 0x0A for parts with `SEQ_ID = 0x01`
  (datasheet errata). Autonomous mode, PS2/PS3, LED2/LED3 and `BUSADDR`
  are not used. `LowPower::sleep` is a no-op because the part is already
  in standby between forced commands.
