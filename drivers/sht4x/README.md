# zigbee-sht4x

Sensirion SHT40 / SHT41 / SHT43 / SHT45 temperature and relative humidity
sensors over I2C. All precision modes, the six heater pulses (20/110/200 mW,
0.1/1 s, each followed by a high-precision measurement), serial number and
soft reset.

| | |
|---|---|
| Address | 0x44 (-A), 0x45 (-B), 0x46 (-C) — fixed by order code |
| Conversion | high 8.3 ms, medium 4.5 ms, low 1.6 ms (max) |
| Heater | 1.1 s / 0.11 s max + 8.3 ms measurement; ≤ 10 % duty cycle, ≤ 65 °C ambient, up to ~75 mA |
| Idle current | 80 nA (no sleep command needed) |
| Output | 0.01 °C, 0.01 %RH (RH cropped to 0–100 %) |
| Source | Sensirion Datasheet SHT4x v7.3 (June 2026) |
| Status | mock-tested |

Wiring: VDD 1.08–3.6 V, pull-ups on SDA/SCL. `serial_number()` takes a
delay (10 ms; no execution time is specified); the measurement path never
delays. The heater draws a large current pulse: size the supply / buffer
capacitor accordingly.
