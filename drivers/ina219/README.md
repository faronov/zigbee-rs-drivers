# zigbee-ina219

TI INA219 bidirectional current / power monitor (shunt + bus voltage, 12-bit)
over I2C.

| | |
|---|---|
| Address | 0x40..=0x4F (A0/A1 pins; 0x40 with both at GND) |
| Conversion | 84 µs (9-bit) … 532 µs (12-bit) per channel, up to 68.1 ms with 128× averaging; shunt + bus conversions add up (driver adds 10 % margin) |
| Power-down current | 6 µA typ, 15 µA max; 40 µs recovery |
| Output | bus voltage mV, current µA, power mW (shunt voltage µV in the struct) |
| Calibration | `Calibration::new(shunt_µΩ, current_LSB_nA)` or `Calibration::for_max_current(shunt_µΩ, I_max_mA)` (equations 1–2) |
| Source | TI SBOS448G (December 2015) |
| Status | mock-tested |

Notes:

- Default mode is triggered shunt + bus conversion; the device is in
  power-down between samples. `read_measurement` returns `NotReady` until
  CNVR is set. `Mode::Continuous` reads the latest free-running result.
- Calibration register bit 0 is void; the driver clears it and derives the
  µA / mW scale factors from the value actually written, so decoded values
  match the device's own current/power math. Decoding is a 64-bit multiply
  and shift (Q24), no division at run time.
- Bus voltage is measured at IN− (high-side: load voltage). The bus input is
  limited to 26 V even with the 32 V range selected.
- An OVF flag (current/power math overflow) is reported as `InvalidData`;
  choose a larger current LSB or a wider shunt range.
- Wiring: shunt between IN+ and IN−, 0.1 µF on VS; SDA/SCL need pull-ups.
  There is no ID register, so there is no `probe`.
