# zigbee-hs300x

Renesas HS3001 / HS3002 / HS3003 / HS3004: temperature and relative
humidity over I2C (one protocol; the parts differ in accuracy grade only).

| | |
|---|---|
| Address | 0x44 (custom addresses on request) |
| Conversion (typ.) | wake-up 0.1 ms + RH and T 0.55 / 1.31 / 4.50 / 16.90 ms each (8 / 10 / 12 / 14 bit); 33.9 ms at the 14-bit default; driver adds 25 % |
| Not-ready handling | stale status bits (`01`) → `NotReady` |
| Sleep current | automatic sleep after each measurement |
| Output | 0.01 °C, 0.01 %RH |
| Features | Measurement Request / Data Fetch, programming mode (enter within 10 ms of power-up), NVM resolution read / write per channel, 32-bit sensor ID |
| Source | Renesas HS300x Datasheet (April 9, 2021); programming-mode success status 0x81 from the Renesas FSP `rm_hs300x` driver |
| Status | mock-tested |

There is no CRC and no fixed ID register outside programming mode, so the
driver has no `verify_id`. Resolution changes are written to non-volatile
memory (14 ms each) and persist across power cycles; tell the driver the
programmed resolution with `with_config` so it waits the right time.
