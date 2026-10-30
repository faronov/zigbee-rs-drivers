# zigbee-sts4x

Sensirion STS40 / STS45 digital temperature sensors over I2C (SHT4x command
set without humidity and heater): three precision modes, serial number,
soft reset.

| | |
|---|---|
| Address | 0x44 (-A), 0x45 (-B), 0x46 (-C) — fixed by order code |
| Conversion | high 8.3 ms, medium 4.5 ms, low 1.6 ms (max) |
| Idle current | 80 nA (no sleep command needed) |
| Output | 0.01 °C |
| Source | Sensirion Datasheet STS4x v3.1 (February 2025) |
| Status | mock-tested |

Wiring: VDD 1.08–3.6 V, pull-ups on SDA/SCL. Keep ≥ 1 ms between commands
(t_W). `serial_number()` takes a delay (10 ms margin).
