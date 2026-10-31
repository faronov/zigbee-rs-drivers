# zigbee-dps310

Infineon DPS310 and DPS368 barometric pressure and temperature sensors
(same register map and compensation; both read `ID = 0x10`, so the driver
does not tell them apart).

| | |
|---|---|
| Bus | I2C (0x77 SDO floating/high, 0x76 SDO to GND) or 4-wire SPI (mode 3, ≤ 10 MHz) |
| Mode | command mode: temperature conversion, then pressure conversion |
| Conversion | per phase `2 ms + 1.6 ms × N` + 10 %: 4.0 ms (×1), 30.4 ms (×16, default pressure), 227.5 ms (×128) |
| Sleep current | 0.5 µA standby |
| Output | 0.01 °C, Pa |
| Math | 64-bit fixed point (Q24) of the datasheet polynomial, ≤ 1 Pa / 0.01 °C from float |
| Source | Infineon DPS368 datasheet v1.1 (2019-07-03); DPS310 datasheet v1.2 shares registers and formulas |
| Status | mock-tested |

Two-phase API (same as `zigbee-bmp180`): `start_measurement()` starts the
temperature conversion; the first `read_measurement()` captures it, starts
the pressure conversion and returns `NotReady` — wait `pending_wait_us()`
and read again. `measure_full(delay)` runs the whole cycle for bring-up.

Notes:

* `init()` returns `NotReady` until `COEF_RDY` and `SENSOR_RDY` are set
  (up to 40 ms after power-up or soft reset).
* The temperature sensor (`TMP_EXT`) is chosen from `COEF_SRCE`, matching
  the calibration.
* DPS310 errata: some parts read ~60 °C because of a fuse-bit problem;
  `init()` writes Infineon's published fix sequence
  (`0x0E←0xA5, 0x0F←0x96, 0x62←0x02, 0x0E←0x00, 0x0F←0x00`), harmless on
  good parts.
* Pressure oversampling above ×8 turns on the result shift (`CFG_REG`).
* Wiring: CSB high (or floating via pull-up) selects I2C; SDO sets the
  address. VDD 1.7-3.6 V, VDDIO 1.2-3.6 V.
