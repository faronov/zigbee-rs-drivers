# zigbee-hdc302x

Texas Instruments HDC3020 / HDC3021 / HDC3022: temperature and relative
humidity over I2C, trigger-on-demand mode with CRC-8 on every word.

| | |
|---|---|
| Address | 0x44 / 0x45 / 0x46 / 0x47 (ADDR, ADDR1 pins) |
| Conversion (max) | LPM0 14.1 ms, LPM1 8.4 ms, LPM2 5.7 ms, LPM3 4.2 ms |
| Sleep current | 0.36 µA typ. (trigger-on-demand), automatic after each conversion |
| Start-up / reset | Sensor PUR 5 ms max |
| Output | 0.01 °C, 0.01 %RH |
| Features | low-power modes, heater on/off and power level, NVM RH/T offset programming with encoding helpers (out of range → `InvalidArgument`), status register, manufacturer ID check (0x3000), 48-bit NIST ID, soft reset |
| Source | TI SNAS778D (Jul 2024) |
| Status | mock-tested |

HDC3021 (removable tape) and HDC3022 (IP67 PTFE membrane) differ from
HDC3020 only mechanically. A read before the conversion finishes is
NACKed and returns `Error::Bus`.

Offset programming writes EEPROM: keep the part in trigger-on-demand mode
and do not access the bus for 77 ms afterwards. Note: the datasheet's
Figure 7-22 example (`0xAA 0x33 0xAC` for +8.20 %RH / −7.17 °C) does not
match its own tables; −7.17 °C encodes as `0x2A` (Table 7-13) and the CRC of
`0xAA 0x2A` is `0xCD`.

Wiring: SDA/SCL pull-ups required; ALERT and nRESET are not used.
