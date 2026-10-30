# zigbee-vcnl4040

Vishay VCNL4040 ambient light (ALS + white channel) and proximity sensor
with an integrated IRED, over I2C.

| | |
|---|---|
| Address | 0x60 (fixed) |
| ID | `ID` word low byte 0x86 (`probe()` returns the full word; high byte = version) |
| Mode | one-shot: ALS on + PS active force mode with `PS_TRIG` → wait → read → `ALS_SD`/`PS_SD`; `with_continuous(true)` uses the PS duty cycle and keeps running |
| Gain / IT | ALS 80/160/320/640 ms; PS 1T…8T, 1/2/4/8 pulses, 50–200 mA, 12/16-bit |
| Lux | `counts · 0.1 lx / 2^ALS_IT` (Table 14) |
| Wait | ALS IT + 10 %; PS IT × pulses (+ duty period when continuous) + 5 ms |
| Standby current | 0.2 µA typ. (both shut down) |
| Output | milli-lux, raw ALS/white, raw proximity (`Quantity::Proximity`), saturation flag |
| Interrupt | ALS and PS 16-bit windows + persistence (`set_interrupt()`); `take_interrupt()` reads and clears `INT_Flag` |
| Helper | `suggest_integration()` (auto-range), `to_millilux()`, `set_proximity_cancellation()` |
| Source | Vishay 84274 datasheet (Rev. 2.0); PS T ≈ 100 µs from Linux `vcnl4000.c` |
| Status | mock-tested |

Notes:

* The datasheet gives no PS conversion time. T = 100 µs comes from the
  Linux driver, and the extra 5 ms margin is an estimate.
* There is no data-ready flag, so honour the returned wait time.
* Interrupts only fire while the blocks are running, so use them with
  `with_continuous(true)`. The PS interrupt is set to trigger on both
  close and away. Logic-output mode (`PS_MS`) and sunlight cancellation
  are not used.
* The white channel is always enabled (reset default).
