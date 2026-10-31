# zigbee-htu31d

TE Connectivity HTU31D: temperature and relative humidity over I2C with
CRC, per-channel oversampling, heater, diagnostics and serial number.

| | |
|---|---|
| Address | 0x40 (IC_ADD = GND), 0x41 (IC_ADD = VDD) |
| Conversion (typ.) | RH 1.11 / 2.14 / 4.21 / 8.34 ms, T 1.57 / 3.06 / 6.03 / 11.98 ms (OSR 0–3); driver waits the sum + 50 % |
| Power-up / reset | 10 ms / 15 ms |
| Sleep current | 50 nA typ. (200 nA max), automatic after each conversion |
| Output | 0.01 °C, 0.01 %RH |
| Features | conversion command with OSR (default OSR 2 / 2), CRC-8 (0x31 / 0x00), heater on / off, diagnostic register, 24-bit serial number, soft reset |
| Source | TE Connectivity HTU31D RH/T SENSOR IC datasheet (05/2020) |
| Status | mock-tested |

The sensor gives no ready indication: Read T&RH returns the last stored
result, so reading before the conversion ends silently returns the
previous sample. Conversion times are typical only (25 °C, 5 V), hence
the 50 % margin. The heater dissipates noticeable power (R_heat ≈ 2 kΩ);
switch it off again before measuring humidity.
