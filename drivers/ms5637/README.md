# zigbee-ms5637

TE Connectivity MS5637-02BA, MS5837-30BA and MS5837-02BA pressure and
temperature sensors (I2C only). The parts have no ID register and share
the address; the application picks the `Variant` (compensation
constants).

| | |
|---|---|
| Bus | I2C, fixed address 0x76, up to 400 kHz |
| Mode | command driven: D2 (temperature) conversion, then D1 (pressure) conversion |
| Conversion | per phase: 0.60 / 1.17 / 2.28 / 4.54 / 9.04 / 18.08 ms for OSR 256 … 8192 (30BA max table, used for all; default 8192 for both) |
| Sleep current | ≤ 0.1 µA standby at 25 °C (idles automatically between conversions) |
| Output | 0.01 °C, Pa (MS5837-30BA resolution 10 Pa) |
| Math | datasheet integer algorithm (64-bit, floor shifts) with second-order compensation; reproduces the datasheet examples |
| Source | TE MS5637-02BA03 datasheet 06/2017; MS5837-02BA01 REV A8 12/2019; MS5837-30BA REV C2 12/2019 |
| Status | mock-tested |

Two-phase API (same as `zigbee-ms5611`): `start_measurement()` starts D2;
the first `read_measurement()` reads it, starts D1 and returns `NotReady`
— wait `pending_wait_us()` and read again. `measure_full(delay)` runs the
whole cycle for bring-up.

Notes:

* Send `soft_reset()` once after power-up and wait `RESET_WAIT_US`
  (3 ms). `init()` (called by `start_measurement`) reads the 7 PROM words
  and checks the CRC-4 in word 0 (`Error::Crc` on mismatch); this is also
  the presence check.
* No status register: reading the ADC too early returns 0 and spoils the
  conversion; the driver reports `Error::InvalidData` and goes back to
  idle.
* The fixed address means only one of these parts per bus (MS5611 at
  0x76 with CSB high also collides).
* Wiring: VDD 1.5-3.6 V, 100 nF decoupling. The MS5837 gel-filled parts
  are meant for water/liquid contact — keep the O-ring seal per the
  datasheet.
