# zigbee-spl06

Goertek SPL06-001 and Goermicro SPL07-003 barometric pressure and
temperature sensors (DPS310-style register map; the variant is detected
from `ID`: 0x10 SPL06-001, 0x11 SPL07-003).

| | |
|---|---|
| Bus | I2C (0x77 SDO high/floating, 0x76 SDO to GND) or 4-wire SPI (mode 3, ≤ 10 MHz) |
| Mode | command mode: temperature conversion, then pressure conversion |
| Conversion | per phase `2 ms + 1.6 ms × N` + 10 %: 4.0 ms (×1), 30.4 ms (×16, default pressure), 227.5 ms (×128) |
| Sleep current | < 1 µA standby (SPL07-003 0.5 µA typ.) |
| Output | 0.01 °C, Pa |
| Math | 64-bit fixed point (Q24) of the datasheet polynomial, ≤ 1 Pa / 0.01 °C from float |
| Source | Goertek SPL06-001 datasheet version 7.0 (2018-01-26); Goermicro SPL07-003 datasheet Ver1.0 (2023-02-24) |
| Status | mock-tested |

Two-phase API (same as `zigbee-dps310`): `start_measurement()` starts the
temperature conversion; the first `read_measurement()` captures it, starts
the pressure conversion and returns `NotReady` — wait `pending_wait_us()`
and read again. `measure_full(delay)` runs the whole cycle for bring-up.

Variant differences handled by the driver:

* SPL06-001: 18 coefficient bytes (0x10–0x21), `TMP_CFG.TMP_EXT = 1`
  (external MEMS temperature sensor, as the datasheet requires).
* SPL07-003: 21 coefficient bytes (0x10–0x24) adding the 12-bit `c31` and
  `c40` terms; `TMP_EXT` lives in `MEAS_CFG` bit 3 and stays 0 (internal
  ASIC sensor, as recommended).

Notes:

* `init()` returns `NotReady` until `COEF_RDY` and `SENSOR_RDY` are set
  (up to 40 ms after power-up or soft reset).
* Oversampling above ×8 turns on the result shift (`CFG_REG`).
* The SPL06-001 reads the same `ID` (0x10) as the Infineon DPS310; use the
  matching crate for the part actually fitted.
* Wiring: CSB high (internal pull-up) selects I2C; SDO sets the address.
