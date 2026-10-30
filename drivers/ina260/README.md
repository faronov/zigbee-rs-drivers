# zigbee-ina260

TI INA260 current, bus voltage and power monitor with an integrated,
factory-trimmed 2 mΩ shunt (±15 A continuous), I2C.

| | |
|---|---|
| Address | 0x40..=0x4F (A0/A1 to GND/VS/SDA/SCL) |
| Conversion | 140 µs … 8.244 ms per channel × 1…1024 averages; current + bus add up (driver adds the 10 % max tolerance) |
| Power-down current | 0.5 µA typ, 2 µA max; 40 µs recovery |
| Output | bus voltage mV (1.25 mV LSB), current µA (1.25 mA LSB), power mW (10 mW LSB, magnitude) |
| Source | TI SBOS656C (December 2016) |
| Status | mock-tested |

Notes:

- No calibration: LSBs are fixed by the integrated shunt.
- Triggered mode (default) with power-down between samples;
  `read_measurement` returns `NotReady` until CVRF is set (reading
  Mask/Enable clears it). OVF → `InvalidData`.
- The power register is always positive; take the sign from the current.
- `probe` checks manufacturer 5449h and die ID 227xh.
- The VBUS pin is separate from IN+/IN−; tie it to IN− for high-side load
  voltage.
