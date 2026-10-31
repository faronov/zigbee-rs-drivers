# zigbee-si705x

Silicon Labs Si7050 / Si7051 / Si7053 / Si7054 / Si7055 digital
temperature sensors over I2C (accuracy ±4 °C down to ±0.1 °C).

| | |
|---|---|
| Address | 0x40 (fixed) |
| Conversion (max) | 14 bit 10.8 ms, 13 bit 6.2 ms, 12 bit 3.8 ms, 11 bit 2.4 ms |
| Standby current | 60 nA typ., automatic after each conversion |
| Power-up | 80 ms max over temperature |
| Output | 0.01 °C |
| Features | resolution (read-modify-write), electronic ID with device-code check (0x32..0x37), firmware revision, soft reset |
| Source | Si7050/1/3/4/5-A20 datasheet Rev 1.2 (not re-verified, Silicon Labs download blocked); cross-checked with Si7021-A20 Rev 1.2 |
| Status | mock-tested |

Wiring: SDA/SCL pull-ups required. No-hold-master mode only: the part
NACKs reads while converting (reported as `Error::Bus`).
