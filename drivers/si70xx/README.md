# zigbee-si70xx

Temperature and relative humidity over I2C for the Si7021 command-set
family: Silicon Labs Si7006 / Si7013 / Si7020 / Si7021, TE HTU21D(F) and
Sensirion SHT20 / SHT21 / SHT25 (plus the many Si7021/HTU21D clones).

| | |
|---|---|
| Address | 0x40 (Si7013: 0x40 / 0x41 via AD0) |
| Conversion (max, RH 12 bit + T 14 bit) | Si70xx 22.8 ms (RH includes T, read back with 0xE0); HTU21D 16 + 50 ms; SHT2x 29 + 85 ms |
| Standby current | 60 nA (Si7021), 0.02 µA (HTU21D), 0.15 µA (SHT21); automatic after each conversion |
| Power-up | 80 ms max (Si7021 over temperature), 15 ms (HTU21D / SHT2x) |
| Output | 0.01 °C, 0.01 %RH (RH clamped to 0..100 %) |
| Features | resolution (user register, read-modify-write), heater on/off, Si70xx heater current, electronic ID, Si70xx device-code check and firmware revision |
| Source | Si7021-A20 Rev 1.2; HTU21D(F) HPC199_3; SHT21 datasheet + "Electronic Identification Code" app note v1.1 |
| Status | mock-tested |

Wiring: SDA/SCL need pull-ups (10 kΩ typ., many breakouts include them).
The driver uses no-hold-master mode only, so clock stretching is never
required; the sensor NACKs reads while converting (reported as
`Error::Bus`).

HTU21D and SHT2x cannot return the temperature of the previous humidity
conversion, so `read_measurement` performs two phases: the first call
reads RH, starts the temperature conversion and returns `Error::NotReady`;
wait `pending_wait_us()` and call again. `Si70xx::measure` /
`asynch::Si70xx::measure_all` do both steps.

Electronic-ID CRC bytes are not verified (coverage is not specified
consistently across the datasheets).
