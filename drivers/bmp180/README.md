# zigbee-bmp180

Bosch BMP180 and its predecessor BMP085: barometric pressure and
temperature. Both share registers, chip ID (0x55) and algorithm.

| | |
|---|---|
| Bus | I2C, fixed address 0x77, up to 3.4 MHz |
| Mode | one-shot; two conversions per sample (temperature, then pressure) |
| Conversion | temperature 4.5 ms; pressure 4.5 / 7.5 / 13.5 / 25.5 ms for oss 0-3 |
| Standby current | 0.1 µA typ. (automatic after each conversion, no sleep command) |
| Output | 0.01 °C, Pa |
| Math | 32-bit integer datasheet algorithm |
| Source | BMP180 datasheet BST-BMP180-DS000-09 rev 2.5; BMP085 BST-BMP085-DS000-06 |
| Status | mock-tested |

## Two-phase cycle

`start_measurement()` starts the temperature conversion and returns 4.5 ms.
The first `read_measurement()` captures the temperature, starts the
pressure conversion and returns `Error::NotReady`; wait
`pending_wait_us()` and call `read_measurement()` again for the result.
The generic `measure()` helpers therefore stop at `NotReady`; use
`measure_full(delay)` (blocking and async) in tools and tests.

`read_measurement()` also returns `NotReady` without changing state while
`ctrl_meas.sco` shows that a conversion is still running.

Wiring: VDDIO 1.62-3.6 V, external I2C pull-ups. The BMP085's XCLR and EOC
pins are not used (EOC is replaced by polling `sco`).
