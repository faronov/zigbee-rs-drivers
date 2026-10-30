# zigbee-cht8305

Sensylink CHT8305 / CHT8305C: temperature and relative humidity over I2C
with automatic standby after each conversion.

| | |
|---|---|
| Address | 0x40 (AD0 GND / open), 0x44 (VCC), 0x48 (SDA), 0x4C (SCL) |
| Conversion | 11 ms typ. (14-bit T + RH); reference code waits > 20 ms, driver waits 21 ms |
| Standby current | 0.05 µA typ., 0.3 µA max |
| Output | 0.01 °C, 0.01 %RH |
| Features | pointer-write trigger, T / RH resolution (14 / 11 / 8 bit), heater, soft reset, manufacturer and version ID |
| Source | Sensylink CHT8305C Advanced Datasheet Ver 2.5 (2021-10-26) |
| Status | mock-tested |

The part NACKs reads until the conversion ends, so reading too early
returns `Error::Bus`. There is no CRC. Conversion times for the 11-bit
and 8-bit settings are not published; the driver always waits 21 ms.

The manufacturer ID (0x5959) is also used by the CHT8310 / CHT8315, so
`verify_id` cannot tell them apart; it returns the version ID register
(documented only as 0x8XXX). The soft-reset duration is not specified;
the SRST bit reads 0 once it completes.
