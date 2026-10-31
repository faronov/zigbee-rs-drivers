# zigbee-ms5611

TE Connectivity (Measurement Specialties) MS5611-01BA and MS5607-02BA
barometric pressure and temperature sensors. The parts have no ID
register; the application picks the `Variant` (it selects the
compensation constants).

| | |
|---|---|
| Bus | I2C (0x77 CSB to GND, 0x76 CSB to VDD; PS high) or SPI (mode 0/3, ≤ 20 MHz; PS low) |
| Mode | command driven: D2 (temperature) conversion, then D1 (pressure) conversion |
| Conversion | per phase, datasheet max: 0.60 / 1.17 / 2.28 / 4.54 / 9.04 ms for OSR 256 … 4096 (default 4096 for both) |
| Sleep current | < 0.15 µA (idles automatically between conversions) |
| Output | 0.01 °C, Pa |
| Math | datasheet integer algorithm (64-bit) including second-order compensation; matches the datasheet examples exactly |
| Source | TE MS5611-01BA03 datasheet 06/2017; MEAS MS5607-02BA03 datasheet DA5607-02BA03_018 (2012-09-26) |
| Status | mock-tested |

Two-phase API (same as `zigbee-bmp180`): `start_measurement()` starts D2;
the first `read_measurement()` reads it, starts D1 and returns `NotReady`
— wait `pending_wait_us()` and read again. `measure_full(delay)` runs the
whole cycle for bring-up.

Notes:

* Send `soft_reset()` once after power-up and wait `RESET_WAIT_US` (3 ms):
  the datasheet requires the reset to load the PROM. `init()` (called by
  `start_measurement`) reads the 8 PROM words and checks the CRC-4
  (`Error::Crc` on mismatch); this is also the presence check.
* There is no status register. Reading the ADC too early returns 0 and
  spoils the conversion: the driver reports `Error::InvalidData` and goes
  back to idle; start a new cycle.
* Keep the bus quiet during conversions for the best noise (datasheet).
* Wiring: CSB must not float in I2C mode (it sets the address LSB). VDD
  1.8-3.6 V, 100 nF decoupling close to the part.
