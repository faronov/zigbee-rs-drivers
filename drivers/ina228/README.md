# zigbee-ina228

TI INA228 (I2C) / INA229 (SPI) 85 V, 20-bit current, voltage, power,
energy and charge monitors with die temperature sensor.

| | |
|---|---|
| Interface | INA228: I2C 0x40..=0x4F (A0/A1); INA229: SPI mode 1, ≤ 10 MHz |
| Conversion | 50 µs … 4.12 ms per channel (bus, shunt, temperature) × 1…1024 averages; driver adds 10 % margin |
| Shutdown current | 2.8 µA typ, 5 µA max; 60 µs wake-up, 300 µs after reset |
| Output | bus voltage mV, current µA, power mW, die temperature 0.01 °C, energy mWh (continuous mode); shunt voltage nV and charge µAh in the struct |
| Calibration | `Calibration::for_max_current(shunt_µΩ, I_max_mA, range)`, `Calibration::new(shunt_µΩ, current_LSB_nA, range)` or `Calibration::from_register` |
| Source | TI SLYS021A (INA228), SLYS023A (INA229) |
| Status | mock-tested (datasheet section 8.2.2 example values) |

Notes:

- `Ina228::new_i2c` / `Ina228::new_spi`; the same type drives both parts.
  `probe` returns `Chip::Ina228` or `Chip::Ina229` from the die ID.
- Triggered mode (default): `start_measurement` writes MODE = 7
  (single-shot bus + shunt + temperature); the ADC shuts down afterwards.
  `read_measurement` reads DIAG_ALRT (which clears CNVRF) and returns
  `NotReady` until CNVRF is set.
- Continuous mode (`with_mode(Mode::Continuous)`): ENERGY and CHARGE are
  only valid here (the device integrates over its internal time base);
  `reset_accumulators` sets RSTACC. `sleep` stops the ADC and the
  accumulators.
- MATHOF or a trim-memory checksum error (MEMSTAT = 0) → `InvalidData`.
- Scaling uses the SHUNT_CAL value actually written: Q24 multipliers for
  current/power, Q40 for energy/charge (20-bit split multiply, no 128-bit
  math or runtime division). Current LSB is limited to ≤ 4.096 mA so the
  full scale fits `i32` µA.
- Charge has no `Quantity` variant, so it is not yielded by `Readings`.
- Alert limits, SHUNT_TEMPCO and CONVDLY are reachable through
  `write_register` with the `reg::*` addresses.
