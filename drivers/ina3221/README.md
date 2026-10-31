# zigbee-ina3221

TI INA3221 triple-channel high-side shunt and bus voltage monitor, I2C.

| | |
|---|---|
| Address | 0x40..=0x43 (A0 to GND/VS/SDA/SCL) |
| Conversion | 140 µs … 8.244 ms per signal × 1…1024 averages; one pass = enabled channels × (shunt + bus) (driver adds the 10 % max tolerance) |
| Power-down current | 0.5 µA typ, 2 µA max; 40 µs recovery |
| Output | per enabled channel: bus voltage mV (8 mV LSB), current µA, power mW; shunt voltage µV (40 µV LSB) in the struct |
| Calibration | `Shunts::new([R1_µΩ, R2_µΩ, R3_µΩ])` |
| Source | TI SBOS576C |
| Status | mock-tested |

Notes:

- The INA3221 has no current/power registers. Current = V_shunt / R uses a
  Q24 factor precomputed per channel; power = mV × µA uses a `× 4295 >> 32`
  approximation of `/ 1e6` (7.6 ppm error, no 64-bit division).
- `Readings` yields Voltage, Current, Power for each enabled channel in
  channel order; map them to endpoints in that order.
- Triggered mode (default): one single-shot pass over all enabled channels,
  after which the part powers down. `read_measurement` returns `NotReady`
  until CVRF (Mask/Enable bit 0) is set. Reading Mask/Enable also clears the
  critical/warning alert flags.
- Bus inputs: 0–26 V even though the register full scale is 32.76 V.
- Shunt resistance must be ≥ 77 µΩ so the 163.8 mV full-scale current fits
  `i32` µA.
