# zigbee-max6675

Maxim MAX6675 K-type thermocouple-to-digital converter (0 to +1023.75 °C,
read-only SPI), blocking and async.

| | |
|---|---|
| Parts | MAX6675 |
| Bus | SPI mode 0, ≤ 4.3 MHz, read-only (MISO, SCK, CS) |
| Conversion | continuous; 220 ms max after CS high (170 ms typ) |
| Sleep current | none, 0.7 mA typ / 1.5 mA max always on; switch VCC for battery devices |
| Output | 0.25 °C (12-bit), in 0.01 °C |
| Errors | open thermocouple (D2) → `InvalidData`; D15/D1 not 0 (no device) → `InvalidData` |
| Wiring | T− grounded (needed for open-thermocouple detection), 0.1 µF on VCC |
| Source | Maxim MAX6675 datasheet 19-2235 Rev 1 |
| Status | mock-tested |
