# zigbee-shtc3

Sensirion SHTC3 and SHTC1 temperature and relative humidity sensors over
I2C. Normal and low-power measurement modes, ID-register probe (detects the
part), soft reset, and sleep/wake-up (`LowPower`, SHTC3 only).

| | |
|---|---|
| Address | 0x70 (fixed) |
| Conversion | SHTC3: normal 12.1 ms, low-power 0.8 ms; SHTC1: 14.4 ms / 0.94 ms (max) |
| Wake-up / soft reset | 240 µs |
| Sleep current | SHTC3 0.3 µA (idle without sleep: 45 µA); SHTC1 idles at 0.7 µA, no sleep command |
| Output | 0.01 °C, 0.01 %RH |
| Source | Sensirion Datasheet SHTC3 v4 (Dec 2022), Datasheet SHTC1 v5 (Dec 2022), AN Low Power Measurement Mode SHTC1 v4 (May 2016) |
| Status | mock-tested |

Use the cycle wake → wait 240 µs → `start_measurement` → wait → `read_measurement`
→ `sleep` on an SHTC3; otherwise it draws 45 µA between samples. SHTC1 VDD
is 1.62–1.98 V; SHTC3 is 1.62–3.6 V. Keep the bus quiet during low-power
conversions (repeatability, AN section 1).
