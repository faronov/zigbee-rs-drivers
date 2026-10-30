# zigbee-ds18b20

Maxim DS18B20, MAX31820, DS1822 and DS18S20 / DS1820 1-Wire digital
thermometers. Generic over the `zigbee-onewire` bus traits (bit-banged GPIO
or a hardware 1-Wire master), blocking and async.

| | |
|---|---|
| Parts | DS18B20 / MAX31820 (family 28h), DS1822 (22h), DS18S20 / DS1820 (10h) |
| Bus | 1-Wire; Match ROM by 64-bit ROM code, or Skip ROM for a single device |
| Conversion | 9/10/11/12 bit: 94 / 188 / 375 / 750 ms max; DS18S20 750 ms; EEPROM copy 10 ms |
| Sleep current | 750 nA typ, 1 µA max standby (DS18B20) |
| Output | 0.01 °C; DS18S20 extended resolution via COUNT_REMAIN |
| Wiring | DQ with 4.7 kΩ pull-up; VDD 3.0–5.5 V, or parasite power (VDD to GND) with a strong pull-up the bus master provides |
| Errors | power-on +85 °C value → `InvalidData`, scratchpad CRC → `Crc`, wrong family → `UnexpectedId`, no presence → `Timeout` |
| Source | Maxim DS18B20 datasheet rev 042208; DS18S20 datasheet (extended resolution) |
| Status | mock-tested |

Several sensors: enumerate with `zigbee_onewire::Search`, start them all with
`start_all()` (Skip ROM + Convert T), then `read_device(&rom)` each one.
A genuine 85.00 °C reading is indistinguishable from the power-on value and is
also reported as `InvalidData`.
