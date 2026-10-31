# zigbee-xbr818

Phosense XBR818 10.525 GHz radar presence-detector SoC, e.g. on the
Ai-Thinker Rd-04 module.

| | |
|---|---|
| Interface | I2C 0x71 (configuration only, gated by I2C_EN) + IO_VAL presence pin (`InputPin`; async `Wait` for edge waits) |
| Conversion | autonomous detector; `start_measurement` returns 0, `read_measurement` samples IO_VAL |
| Output | `Quantity::Occupancy` |
| Config | RF power 0–7, sample divider, trigger/noise thresholds, delay (hold) and lock times (ms → 32 kHz ticks, max ≈ 524 s), IO_VAL polarity |
| Source | Phosense XBR818 datasheet Rev 2.3; Ai-Thinker Rd-04 module manual V1.0.0 §6 (register map) |
| Status | mock-tested |

Notes:

* There is no ID register, so the driver has no `probe()`.
* Drive I2C_EN high and wait `I2C_ENABLE_WAIT_US` (10 µs) before
  `configure`; pull it low afterwards to save power.
* Registers are volatile: call `configure` after every power-up. Detection is
  disabled (0x13 = 0) while writing and re-enabled last.
* Timer values assume the nominal 32 kHz RC clock and are approximate.
* The read-only detection data registers are not fully documented and are
  not used.
