# zigbee-hm330x

HM3301 / HM3302 (HM-3300 series, e.g. Seeed Grove "Laser PM2.5 Sensor
HM3301") laser particulate matter sensors over I2C: CF=1 and atmospheric
PM1.0/PM2.5/PM10, particle counts and checksum verification.

| | |
|---|---|
| Bus | I2C 100–400 kHz, 3.3 V levels |
| Address | `0x40` (datasheet: `0x80`/`0x81` 8-bit) |
| ID | sensor number in bytes 2–3 (no fixed ID) |
| Timing | refresh every 1 s; stable 30 s after power-on / wake |
| Current | < 75 mA average, < 120 mA peak; < 150 µA sleeping (SET low); 5 V ± 3 % |
| Output | `Pm1`, `Pm25`, `Pm10` (atmospheric) in 0.1 µg/m³; range 1–500 µg/m³ effective |
| Source | HM-3300/3600 Dust Sensor Data Sheet V2.1, Appendix 2 |
| Status | mock-tested |

## Usage

The module powers up in UART active-upload mode. The first
`start_measurement` writes the select command `0x88` (I2C mode; UART off)
and returns 0; every `read_measurement` reads the 29-byte frame and checks
the checksum (sum of bytes 0..27 = byte 28). If the module is power-cycled,
call `reset_selection()` so the select command is sent again.

## Notes

- Sleep is only possible via the SET pin (low = sleep, fan off); there is no
  I2C command, so `LowPower` is not implemented.
- Particle counts are per **litre** (PMS sensors use 0.1 L) and are exposed
  on `Measurement` only.
- Do not use UART and I2C at the same time. The UART protocol (`42 4D …`,
  PMS-style) is not implemented here.

## Wiring

| HM330x pin | MCU |
|---|---|
| 1 VCC | 5 V |
| 2 GND | GND |
| 3 SET | optional GPIO (low = sleep; pulled up inside) |
| 6 RESET | optional GPIO (low = reset; pulled up inside) |
| 7 SDA | SDA (pull-up to 3.3 V) |
| 8 SCL | SCL (pull-up to 3.3 V) |
