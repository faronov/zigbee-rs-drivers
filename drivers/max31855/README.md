# zigbee-max31855

Maxim MAX31855 cold-junction compensated thermocouple-to-digital converter
(read-only SPI), blocking and async.

| | |
|---|---|
| Parts | MAX31855K / J / N / T / E / R / S (same frame, one thermocouple type each) |
| Bus | SPI mode 0, ≤ 5 MHz, read-only (MISO, SCK, CS) |
| Conversion | continuous; 100 ms max after CS high (70 ms typ), 200 ms after power-up |
| Sleep current | none, 900 µA typ / 1.5 mA max always on; switch VCC for battery devices |
| Output | thermocouple 0.25 °C (14-bit), cold junction 0.0625 °C (12-bit), in 0.01 °C |
| Errors | open / short-to-GND / short-to-VCC → `InvalidData` (`read_raw().faults()` tells which); reserved bits set (no device) → `InvalidData` |
| Wiring | 10 nF across T+/T−; do not ground T− (short-to-GND detection) |
| Source | Maxim MAX31855 datasheet 19-5793 Rev 2 |
| Status | mock-tested |
