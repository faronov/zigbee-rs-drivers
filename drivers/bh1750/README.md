# zigbee-bh1750

ROHM BH1750FVI (and the common GY-30 / GY-302 modules): 16-bit ambient
light sensor over I2C.

| | |
|---|---|
| Address | 0x23 (ADDR low), 0x5C (ADDR high) |
| Modes | one-time H-res (1 lx), H-res 2 (0.5 lx), L-res (4 lx); auto power-down after each sample |
| Conversion | H / H2: 120 ms typ., 180 ms max; L: 16 ms typ., 24 ms max (at MTreg 69, scales with MTreg) |
| Range | 1–65 535 lx at MTreg 69; 0.11 lx resolution (H2, MTreg 254) up to ~120 klx (MTreg 31) |
| Power-down current | 0.01 µA typ., 1 µA max |
| Output | milli-lux (`Quantity::Illuminance`) + raw count |
| Source | ROHM BH1750FVI datasheet Rev. B (2011.11) |
| Status | mock-tested |

Wiring: DVI must be held low ≥ 1 µs after VCC rises (most modules tie it
to VCC through an RC). The driver never waits internally: it powers on,
sends the one-time opcode and returns the conversion time.
