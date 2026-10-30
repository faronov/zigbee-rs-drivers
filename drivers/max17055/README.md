# zigbee-max17055

Maxim (Analog Devices) MAX17055 1-cell ModelGauge m5 EZ fuel gauge over I2C.

| | |
|---|---|
| Address | 0x36 (fixed) |
| Conversion | autonomous: 175.8 ms task period (active), 5.625 s (hibernate) |
| Power-down current | shutdown 0.5 µA typ (state lost); hibernate 7 µA; active 18 µA |
| Output | cell voltage mV, current / average current µA, temperature 0.01 °C, RepSOC 0.5 %, RepCap / FullCapRep µAh, TTE s |
| Calibration | sense resistor µΩ (`with_sense_resistor`, default 10 mΩ); `EzConfig` (design capacity, termination current, VEmpty, VChg, model) |
| Source | Maxim MAX17055 data sheet 19-8758 Rev 3; UG6365 Software Implementation Guide Rev 2 |
| Status | mock-tested |

Notes:

- The gauge runs by itself: `start_measurement` returns 0, and
  `read_measurement` uses three auto-increment reads (0x08–0x0B,
  0x05–0x06, 0x10–0x11). Registers are LSB first.
- POR handling: configuration is RAM only. When `por()` is true, call
  `start_ez_config(&EzConfig)` (`NotReady` while FStat.DNR), wait, then poll
  `finish_ez_config()` every 10 ms until it stops returning `NotReady`
  (ModelCfg.Refresh). It restores HibCfg and clears Status.POR. This is
  UG6365 option 1 (EZ, no INI); custom INI models can be loaded with
  `write_register`.
- `dPAcc = dQAcc × 44138 / DesignCap` (51200 and ModelCfg.VChg for charge
  voltage above 4.275 V), using the integer dQAcc as UG6365 requires.
- `LowPower::sleep` writes the saved HibCfg (automatic hibernate, the gauge
  keeps tracking). `wake` forces active mode (HibCfg = 0, soft-wakeup
  0x0090, clear) and returns one task period. `shutdown()` is the 0.5 µA
  mode; it takes effect after ShdnTimer (45 s default), any SDA/SCL edge
  wakes the IC, and the next boot needs the EZ configuration again.
- `probe` checks DevName = 0x4010.
- RepCap, FullCapRep and TTE are reported in the struct only. Core
  `Quantity` has no capacity or time variant. `Readings` yields voltage,
  current, battery percent and temperature.
