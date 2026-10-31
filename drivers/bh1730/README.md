# zigbee-bh1730

ROHM BH1730FVC: 16-bit two-channel (visible + IR) ambient light sensor
over I2C, 0.001–100 000 lx.

| | |
|---|---|
| Address | 0x29 (fixed) |
| ID | `PART_ID[7:4]` = 0x7 (`probe()`) |
| Mode | one-time conversion of both channels, ADC powers down by itself |
| Conversion | `ITIME_ms = 2.8 µs × 964 × (256 − ITIME)` typ.; default 0xDA: 102.6 ms typ., 150 ms max |
| Gain | ×1, ×2, ×64, ×128; `suggest_gain()` auto-range helper |
| Power-down current | 0.85 µA typ., 1.5 µA max |
| Output | milli-lux (datasheet section 6 formula, no optical window) + DATA0/DATA1 raw counts |
| Interrupt | DATA0 low/high window with persistence on the open-drain INT pin |
| Source | ROHM BH1730FVC datasheet Rev.001 (8 Jun 2016) |
| Status | mock-tested |

Wiring: INT is open drain (pull-up needed, or leave open/tie to GND when
unused). DVI is the I/O supply (1.65 V – VCC). Allow 2 ms after VCC rises
before the first access.

The lux coefficients assume no optical window or a spectrally flat one. With
a tinted window, scale the result or lengthen `ITIME` as described in
datasheet section 9.
