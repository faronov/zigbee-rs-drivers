# zigbee-max17048

Maxim (Analog Devices) MAX17043 / MAX17044 / MAX17048 / MAX17049 ModelGauge
fuel gauges for 1-cell (43/48) and 2-cell (44/49) Li-ion packs, over I2C.

| | |
|---|---|
| Address | 0x36 (fixed) |
| Conversion | autonomous: VCELL every 250 ms (MAX17048 active), 45 s (hibernate), 500 ms (MAX17043) |
| Power-down current | sleep 0.5 µA typ (2 µA max); hibernate 3–4 µA, active 23 µA (MAX17048); MAX17043 active 50 µA |
| Output | pack voltage mV, SOC in 0.5 % units, CRATE in 0.001 %/h (MAX17048/49) |
| Calibration | RCOMP (`with_rcomp`, `set_rcomp`, `compensated_rcomp`), empty alert 1–32 % |
| Source | Maxim 19-6171 Rev 5 (MAX17048/49), 19-4703 Rev 5 (MAX17043/44) |
| Status | mock-tested |

Notes:

- The gauge runs by itself: `start_measurement` returns 0 and
  `read_measurement` reads VCELL, SOC and (MAX17048/49) CRATE.
- `init` writes CONFIG (RCOMP, ALSC, ATHD, clears ALRT) and, on the
  MAX17048/49, clears STATUS.RI. It returns `true` when RI was set, i.e.
  the gauge was reset and a custom model (if any) must be reloaded. Model
  table loading (0x40–0x7F, unlock 0x3F/0x3E) is left to the application
  via `write_register`.
- `reset` sends the POR command (0x5400 on MAX17048/49, 0x0054 on
  MAX17043/44). The IC does not ACK it, so bus errors are ignored; wait the
  returned 1 s / 250 ms before reading SOC.
- No ID register, so there is no `probe`; `version` reads VERSION.
- `LowPower::sleep` sets MODE.EnSleep (MAX17048/49) and CONFIG.SLEEP. SOC
  is not tracked while asleep. `wake` returns one VCELL update period.
- `compensated_rcomp` implements the data-sheet temperature compensation
  (defaults RCOMP0 0x97, TempCoUp −0.5, TempCoDown −5.0) in integer math.
- MAX17048/49 extras: `set_hibernate` (HIBRT, `(0, 0)` disables
  hibernate), `set_voltage_alert` (VALRT, 20 mV steps), `status`,
  `clear_alert`. These return `Unsupported` on MAX17043/44.
