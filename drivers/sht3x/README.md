# zigbee-sht3x

Sensirion SHT30 / SHT31 / SHT35 (and the GXHT30 clone): temperature and
relative humidity over I2C.

| | |
|---|---|
| Address | 0x44 (ADDR low), 0x45 (ADDR high) |
| Conversion | high 15.5 ms, medium 6.5 ms, low 4.5 ms (single shot, no clock stretching) |
| Idle current | 0.2 µA typ. between single shots |
| Output | 0.01 °C, 0.01 %RH |
| Source | Sensirion Datasheet SHT3x-DIS v7 |
| Status | mock-tested (previous zigbee-rs driver: hw on nRF52840, EFR32MG1) |
