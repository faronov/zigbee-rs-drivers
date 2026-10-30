# zigbee-aht

Aosong (ASAIR) AHT10 / AHT20 / AHT21 / AHT25 / AHT30 and AM2315C:
temperature and relative humidity over I2C.

| | |
|---|---|
| Address | 0x38 (AHT10: 0x39 with ADR high) |
| Conversion | 75 ms (AHT10), 80 ms (others); busy bit → `NotReady` |
| Sleep current | 0.25 µA typ. (AHT10/21/25/AM2315C), ≤ 0.2 µA (AHT20/30), automatic |
| Start-up | 20 ms (AHT10), 40 ms (AHT20), 5 ms (AHT30), 100 ms (AHT21/25/AM2315C) |
| Output | 0.01 °C, 0.01 %RH (20-bit raw) |
| Features | calibration check + init command, AHT21/25/AM2315C register restore steps, CRC-8 (not AHT10), soft reset (AHT10/20/21/25) |
| Source | AHT10 V1.1, AHT20 V1.1 (EN), AHT21/AHT25/AM2315C V1.0 (May 2021), AHT30 (Sept 2023) |
| Status | mock-tested |

`Variant::Aht21` covers AHT21 and AHT25. AM2315C is the encapsulated
AHT21-type probe (the older AM2315 / AM2320 Modbus protocol is in
`zigbee-am2320`).

Datasheet caveats:

- AHT10's init command is `0xE1`; its parameter bytes `0x08 0x00` come from
  common practice, not the datasheet.
- AHT30's datasheet documents no init command; the driver uses the AHT20
  `0xBE 0x08 0x00` sequence.
- AHT21/AHT25/AM2315C datasheets only name registers 0x1B/0x1C/0x1E for the
  power-on restore and defer to vendor sample code, which the
  `restore_register_*` steps follow.
- AM2315C's datasheet says both 80 ms and 250 ms for the conversion; the
  driver returns 80 ms and reports `NotReady` while the busy bit is set.
- Measure at most once per 2 s (AHT10/21/25/AM2315C) or 1 s (AHT20/30) to
  avoid self-heating.
