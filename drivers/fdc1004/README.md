# zigbee-fdc1004

Texas Instruments FDC1004 4-channel capacitance-to-digital converter, for
liquid-level, proximity and capacitive soil probes.

| | |
|---|---|
| Address | 0x50 (fixed) |
| Output | `Quantity::Capacitance` (fF) per measured slot; optional `Quantity::SoilMoisture` (0.01 %) or `Quantity::Distance` (liquid level, mm) through `Mapping` |
| Range | ±15 pF input, plus a CAPDAC offset of up to 96.875 pF (3.125 pF steps) |
| Rate | 100/200/400 S/s (10/5/2.5 ms per slot) |
| Current | 750 µA converting, 29 µA standby |
| Source | TI datasheet SNOSCY5C |
| Status | mock-tested |

Notes:

* `Readings` yields one `Capacitance` per measured slot (M1..M4 order),
  then the mapped quantity. The same values are in
  `Measurement::capacitance_ff`.
* `configure(slot, MeasConfig)` enables a slot. The negative side is ground,
  a CAPDAC offset or a higher-numbered input (differential, where CAPDAC has
  no effect). Invalid combinations return `InvalidArgument`, as does
  starting a measurement with no slot enabled.
* The device can only trigger one single-shot slot at a time. With one
  slot enabled, the driver uses single-shot mode; the datasheet example
  `0x0540` sets REPEAT by mistake, so the correct value is `0x0440`. With
  several slots, it starts repeat mode and stops it after reading all
  results. `NotReady` until every DONE flag is set.
* The conversion time is not specified. The driver returns one sample
  period per slot plus 10 %.
* `set_offset` (5.11 fixed-point pF) and `set_gain` (code / 16384) are
  volatile. Reset and power-up restore offset 0 and the factory gain.
* The liquid level uses the datasheet's three-electrode formula
  `h_ref × (C_lev − C_lev(0)) / (C_RL − C_RE)`. TI gives no numeric
  example for it.
