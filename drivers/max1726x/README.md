# zigbee-max1726x

Maxim (Analog Devices) MAX17260 / MAX17261 / MAX17262 / MAX17263
ModelGauge m5 EZ fuel gauges over I2C.

| | |
|---|---|
| Address | 0x36 (fixed) |
| Conversion | autonomous: 175.8 ms task period (active), 5.625 s (hibernate) |
| Power-down current | shutdown 0.5 µA typ (state lost); hibernate 5.1 µA (60/61), 5.2 µA (62), 8.2 µA (63); active 15–16 µA |
| Output | pack voltage mV, current / average current µA, temperature 0.01 °C, RepSOC 0.5 %, RepCap / FullCapRep µAh, TTE / TTF s |
| Calibration | sense resistor µΩ (default 10 mΩ; MAX17262R/H use their 10 / 30 mΩ internal equivalent); cells; `EzConfig` (design capacity, termination current, VEmpty, VChg, model, R100) |
| Source | ADI MAX17260/61/62/63 data sheets; UG6597 MAX1726x ModelGauge m5 EZ User Guide Rev 3 |
| Status | mock-tested |

Notes:

- The gauge runs by itself: `start_measurement` returns 0, and
  `read_measurement` uses three auto-increment reads (0x08–0x0B,
  0x05–0x06, 0x10–0x11). Registers are LSB first.
- VCell is the per-cell voltage (MAX17261/63 read the CELLX divider).
  `with_cells(n)` reports VCell × n as the pack voltage.
- POR handling: when `por()` is true, call `start_ez_config(&EzConfig)`
  (`NotReady` while FStat.DNR), wait, then poll `finish_ez_config()` every
  10 ms until it stops returning `NotReady`. It restores HibCfg and clears
  Status.POR. The sequence is the same as the MAX17055 EZ flow:
  dQAcc = DesignCap / 32, dPAcc = dQAcc × 44138 (51200 with VChg) /
  DesignCap.
- ModelCfg is written read-modify-write. CSEL/VSEL (bits 3..2, detected by
  the IC at boot on MAX17260/63) are kept, and the bits the data sheet
  requires to be 1 are set: bit 3 on the MAX17261, bit 2 on the MAX17262.
- `LowPower::sleep` writes the saved HibCfg (automatic hibernate, the gauge
  keeps tracking). `wake` forces active mode (HibCfg = 0, soft-wakeup
  0x0090, clear) and returns one task period. `shutdown()` is the 0.5 µA
  mode; it takes effect after ShdnTimer, any SDA/SCL edge wakes the IC, and
  fuel-gauge state is lost.
- `probe` checks DevName: 0x4031 (60), 0x4033 (61), 0x4039 (62), 0x4037 (63).
- MAX17262 Power/AvgPower use a fixed 1.6 mW LSB. The driver does not read
  them.
- `Readings` yields voltage, current, battery percent, temperature,
  `Charge` (RepCap), `Capacity` (FullCapRep) and `TimeToEmpty` /
  `TimeToFull` when the gauge reports them (0xFFFF = not discharging /
  not charging is omitted). Average current is in the struct only.
