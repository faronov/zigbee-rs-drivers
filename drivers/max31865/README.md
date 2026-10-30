# zigbee-max31865

Maxim MAX31865 RTD-to-digital converter for PT100 / PT1000, blocking and
async, with integer IEC 60751 Callendar-Van Dusen conversion.

| | |
|---|---|
| Parts | MAX31865 (PT100 with 430 Ω reference, PT1000 with 4.3 kΩ; any R0 / Rref in mΩ) |
| Bus | SPI mode 1 or 3, ≤ 5 MHz, CS via `SpiDevice` |
| Conversion | 1-shot 55 ms (60 Hz) / 66 ms (50 Hz); bias-off start (auto mode) 72.6 ms with the 11.5 ms default settling; fault cycle 600 µs |
| Sleep current | no shutdown: 1.5 mA typ / 3 mA max with bias off; driver keeps the ≈ 3.8 mA RTD bias off between readings |
| Output | 0.01 °C (and resistance in mΩ); CVD error < 0.001 °C (0..850 °C), < 0.003 °C (−200..0 °C) plus ADC quantisation (≈ 0.03 °C for PT100/430 Ω) |
| Wiring | 2-, 3- or 4-wire RTD; reference resistor REFIN+ to REFIN−; filter cap across RTDIN± sets the bias settling time (10.5·RC + 1 ms) |
| Errors | RTD fault bit → `InvalidData` (details: `read_fault_status()`); out-of-range resistance → `InvalidData` |
| Source | Maxim MAX31865 datasheet 19-6478 Rev 0; IEC 60751 coefficients A = 3.9083e-3, B = −5.775e-7, C = −4.183e-12 |
| Status | mock-tested |
