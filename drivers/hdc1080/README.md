# zigbee-hdc1080

Texas Instruments HDC1010 / HDC1080: temperature and relative humidity
over I2C, sequential T + RH acquisition with one trigger.

| | |
|---|---|
| Address | HDC1080 0x40 (fixed); HDC1010 0x40..0x43 (ADR1/ADR0) |
| Conversion (typ.) | T 6.35 ms (14 bit) / 3.65 ms (11 bit); RH 6.50 / 3.85 / 2.50 ms (14 / 11 / 8 bit); driver waits sum + 10 % |
| Sleep current | 100 nA typ., automatic after each conversion |
| Start-up | 15 ms max after power-on |
| Output | 0.01 °C, 0.01 %RH |
| Features | resolution, heater, battery-low flag (< 2.8 V), soft reset, manufacturer (0x5449) / device ID check (HDC1080 0x1050, HDC1010 0x1000), 41-bit serial |
| Source | TI SNAS672A (HDC1080, Jan 2016), SNAS685A (HDC1010, Aug 2016) |
| Status | mock-tested |

Wiring: SDA/SCL pull-ups required; supply 2.7..5.5 V. Call `configure`
once after power-up (15 ms) to select sequential mode explicitly. The part
NACKs the result read while still converting (reported as `Error::Bus`).
