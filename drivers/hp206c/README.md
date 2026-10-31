# zigbee-hp206c

HopeRF HP206C and HP203B barometric pressure (300-1200 mbar), altitude
and temperature sensors. Both parts compensate on-chip and output 20-bit
results that are already in 0.01 °C and Pa.

| | |
|---|---|
| Bus | I2C only, ≤ 400 kHz; HP206C 0x76, HP203B 0x76 (CSB high) / 0x77 (CSB low) |
| Mode | `ADC_CVT` command per measurement; the part sleeps by itself afterwards |
| Conversion | OSR 128: 4.1 ms … 4096: 131.1 ms for T + P (default OSR 1024: 32.8 ms) |
| Sleep current | 0.1 µA; 1.3 mA peak while converting |
| Output | 0.01 °C, Pa |
| Source | HopeRF HP206C datasheet V2.0, HP203B datasheet V2.0 |
| Status | mock-tested |

Notes:

* No ID register, so there is no identity check. `soft_reset()` returns a
  1 ms wait (power-up is about 400 µs) and `is_ready()` reads `DEV_RDY`.
* `read_measurement()` reads `INT_SRC` first and returns `NotReady` unless
  `PA_RDY` and `T_RDY` are set, then reads both values with `READ_PT`
  (which clears the flags). That costs one extra 1-byte read but makes an
  early or duplicate read safe.
* Compensation (`PARA.CMPS_EN`) is on after reset and is not touched. The
  two datasheets disagree on the `PARA` address (0x0E vs 0x0F).
* Altitude output, thresholds and interrupt pin are not used.
* Wiring: VDD 1.8-3.6 V; 10 kΩ pull-ups on SDA/SCL. On HP203B, tie CSB to
  select the address.
