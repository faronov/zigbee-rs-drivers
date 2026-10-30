# zigbee-hdc2080

Texas Instruments HDC2010 / HDC2021 / HDC2022 / HDC2080: temperature and
relative humidity over I2C, on-demand (trigger) mode.

| | |
|---|---|
| Address | 0x40 (ADDR low / open), 0x41 (ADDR high) |
| Conversion (typ.) | RH 660 / 400 / 275 µs, T 610 / 350 / 225 µs (14 / 11 / 9 bit); driver waits sum + 50 % and checks MEAS_TRIG self-clear |
| Sleep current | 50 nA typ. (HDC2080 max 1 µA), automatic after each conversion |
| Start-up / reset | 3 ms (HDC2010), t_POR 3.5 ms (HDC2021/22) |
| Output | 0.01 °C, 0.01 %RH |
| Features | resolution, heater (90 mA), temperature / humidity offset registers with conversion helpers, manufacturer / device ID check (0x5449 / 0x07D0), soft reset |
| Source | TI SNAS678C (HDC2080, Jul 2021), SNAS693D (HDC2010, Feb 2021), SNAS773A / SNAS774A (HDC2021 / HDC2022, Jun 2020) |
| Status | mock-tested |

The temperature formula differs per datasheet: HDC2080 Rev C uses −40.5 °C
("to match more closely to the silicon"), HDC2010/2021/2022 use −40 °C.
Pick the matching `Variant`; the device ID cannot tell the parts apart.

Wiring: SDA/SCL pull-ups required; DRDY/INT is not used by the driver
(leave unconnected or use it as a wake-up source after enabling it).
