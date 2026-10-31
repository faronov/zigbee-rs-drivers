# zigbee-ens210

ScioSense ENS210: temperature and relative humidity over I2C with
automatic low-power standby and a CRC-7 on every result.

| | |
|---|---|
| Address | 0x43 (fixed) |
| Conversion | single-shot T + RH 130 ms max (booting included); booting 1.2 ms max |
| Standby current | 0.04 µA at 1.8 V (0.5 µA at 3.3 V), automatic after each measurement |
| Output | 0.01 °C, 0.01 %RH (clipped to 100 %RH) |
| Features | single-shot T + RH, CRC-7 (x^7 + x^3 + 1, init 0x7F) and valid-flag checks, PART_ID / DIE_REV / 64-bit UID, low-power control, soft reset |
| Source | ScioSense SC-000897-DS-7 ENS210 Datasheet, Revision 5 (2022-05-11) |
| Status | mock-tested |

PART_ID, DIE_REV and UID are readable only in the active state, so
`device_info` / `verify_id` clear LOW_POWER, wait the booting time on a
caller-supplied delay, read the IDs and set LOW_POWER again.

`LowPower::wake` clears LOW_POWER (the part then stays active; the only
active-state figure given is the continuous-run current,
58 µA typ at 1.8 V); `sleep` restores the power-on default of automatic
standby. Measurements work in either state. The reset time is not
specified; the driver returns the booting time. After reflow the RH
reading may be offset by −2 %RH until it recovers (about a week at room
conditions, or 12 h at 25 °C / 75 %RH).
