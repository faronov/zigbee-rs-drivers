# zigbee-omron2smpb

Omron 2SMPB-02E barometric pressure (30-110 kPa) and temperature sensor.
The part outputs raw 24-bit values plus OTP coefficients; this crate does
the compensation in integer fixed point.

| | |
|---|---|
| Bus | I2C (0x70 SDO low / 0x56 SDO high, ≤ 400 kHz) or 4-wire SPI (≤ 10 MHz) |
| Mode | forced: one T + P conversion per start, then sleep |
| Conversion | 5.5 ms (×2/×1) … 33.7 ms (×32/×4) typ.; default ×8/×1 10.6 ms; returned wait = typ. + 25 % |
| Sleep / supply current | 1.1 µA sleep (typ.); 4.1-21.4 µA at 1 Hz by preset |
| Output | 0.01 °C, Pa |
| Source | Omron 2SMPB-02E datasheet (English 2017-2018, Chinese 2021) |
| Status | mock-tested |

Notes:

* `init()` must be called once (after the 10 ms power-up time or after
  `soft_reset()`) and its 1 ms wait respected: the 2021 datasheet asks for
  1 ms of bus silence after the coefficients are read. Until then
  `start_measurement()` returns `NotReady`.
* The datasheet compensation is floating point. The crate evaluates it in
  `i64` fixed point with compile-time conversion factors; host tests
  compare it with the float formula over random OTP sets (worst 0.51 Pa,
  0.01 °C) and run all-extreme register values for overflow. Readings with
  `|T|` ≥ 128 °C are rejected as `InvalidData`.
* Omron gives only typical times. They fit `2.9 + 0.85·Np + 0.9·Nt` ms for
  all five presets; the driver adds 25 % and also checks
  `DEVICE_STAT.measure` (early reads return `NotReady`).
* The IIR filter (`Config::filter`) is written by `init`/`set_config`.
* Wiring: VDD 1.71-3.6 V, VDDIO 1.2-3.6 V. CSB high selects I2C; CSB low
  selects SPI until the next reset. Tie RST to GND if unused. 3-wire SPI
  is not supported.
