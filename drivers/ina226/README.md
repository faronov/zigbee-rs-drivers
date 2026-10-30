# zigbee-ina226

TI INA226 / INA230 / INA231 bidirectional current, bus voltage and power
monitors (16-bit) over I2C.

| | |
|---|---|
| Address | 0x40..=0x4F (A0/A1 to GND/VS/SDA/SCL) |
| Conversion | 140 µs … 8.244 ms per channel × 1…1024 averages; shunt + bus add up (driver adds the 10 % max tolerance) |
| Power-down current | 0.5 µA typ, 2 µA max (INA226); 40 µs recovery |
| Output | bus voltage mV, current µA, power mW (shunt voltage µV in the struct) |
| Calibration | `Calibration::new(shunt_µΩ, current_LSB_nA)` or `Calibration::for_max_current(shunt_µΩ, I_max_mA)` |
| Source | TI SBOS547C (INA226), SBOS601B (INA230), SBOS644D (INA231) |
| Status | mock-tested |

Notes:

- Triggered mode (default): `start_measurement` writes a single-shot
  shunt + bus conversion, the part returns to power-down when done.
  `read_measurement` polls CVRF in Mask/Enable (reading it clears the flag)
  and returns `NotReady` until it is set. Mask/Enable OVF → `InvalidData`.
- `probe` checks the INA226 manufacturer (5449h) and die ID (226xh). The
  INA230 die-ID register has no documented value and the INA231 has no ID
  registers, so do not call `probe` for those parts.
- Bus range: INA226 0–36 V on the separate VBUS pin; INA230/INA231 0–28 V.
  The ADC full scale is 40.96 V in every case.
- Alert/limit features are available through `write_register` with the
  `reg::MASK_ENABLE` / `reg::ALERT_LIMIT` addresses; the driver only uses
  the conversion-ready and overflow flags.
- Scaling: the CAL value is truncated as the device requires; µA and mW per
  LSB are derived from the written value (Q24 multiply, no runtime
  division).
