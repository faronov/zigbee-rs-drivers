# zigbee-as7341

ams-OSRAM AS7341 11-channel spectral sensor (F1–F8 visible, Clear, NIR),
over I2C.

| | |
|---|---|
| Address | 0x39 (fixed) |
| ID | `ID` bits 7:2 = 0b001001 |
| Mode | PON → ATIME/ASTEP/AGAIN → SMUX write + `SMUXEN` (bounded poll) → `SP_EN` → wait → `AVALID` → burst read → PON = 0 |
| Gain / IT | 0.5×…512×; `(ATIME+1)·(ASTEP+1)·2.78 µs` (default 29/599 = 50 ms, 64×) |
| Lux | none from the datasheet; optional `clear · k / (gain · ms)` with a user calibration factor `k` |
| Wait | integration + 10 % + 1 ms |
| Standby current | 0.7 µA typ. (sleep, PON = 0) |
| Output | raw F1–F8 (per bank), Clear, NIR, saturation; milli-lux only when calibrated |
| Interrupt | Clear-channel (ADC4) window + `APERS` (`set_interrupt()`); `take_interrupt()` clears `AINT` |
| Helper | `suggest_gain()` (auto-range), `Measurement::merge()`, `Measurement::peak()`, `with_alternate()` |
| Source | ams DS000504 v3-00 datasheet; SMUX tables from Adafruit_AS7341 |
| Status | mock-tested |

Notes:

* There are only six ADCs, so one sample covers one bank: F1–F4 + Clear +
  NIR, or F5–F8 + Clear + NIR. `with_alternate(true)` switches bank after
  each read, and `merge()` combines two samples into all eight channels.
* `start_measurement` polls `ENABLE.SMUXEN` up to 16 times (no delay)
  and returns `Error::Timeout` if the SMUX command does not finish.
* The SMUX RAM is written one byte at a time, as in the reference code.
* Flicker detection, AGC, the FIFO, SYNS/SYND modes and the LED driver
  are not implemented.
* `Readings` reports nothing unless `with_lux_factor()` is set.
