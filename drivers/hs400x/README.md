# zigbee-hs400x

Renesas HS40xx: HS4001 / HS4003 (and HS4011 / HS4013 with membrane)
temperature and relative humidity over I2C with CRC.

| | |
|---|---|
| Address | 0x54, fixed |
| Conversion (typ.) | RH + T 0.64 / 0.80 / 1.04 / 1.70 ms, T only 0.37 / 0.45 / 0.60 / 0.91 ms (8 / 10 / 12 / 14 bit); driver adds 50 % |
| Not-ready handling | address NACK while converting → `Error::Bus` |
| Sleep current | 10 nA typ., automatic after each read |
| Reset | soft reset 0xFE, then 500 ms |
| Output | 0.01 °C, 0.01 %RH |
| Features | no-hold RH + T and T-only measurements, CRC-8 (0x1D / 0xFF), resolution register, 32-bit sensor ID, register read / write, stop periodic mode, soft reset |
| Source | Renesas R36DS0034EU HS40xx Datasheet rev. 1.4 (Jun 14, 2024) |
| Status | mock-tested |

The datasheet's ordering table lists HS4001, HS4003, HS4011 and HS4013
only; there is no HS4002 / HS4004. No power-up time is specified (the
500 ms soft-reset wait is the only start-up figure). Periodic measurement
and alert threshold registers are reachable through `read_register` /
`write_register`; the driver itself only uses on-demand measurements.
The sensor ID has no CRC and no fixed value, so there is no `verify_id`.
