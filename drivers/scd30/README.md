# zigbee-scd30

Sensirion SCD30 NDIR CO2 sensor module with temperature and humidity, over
I2C. Continuous measurement with ambient pressure compensation, stop,
measurement interval, data-ready, automatic self-calibration, forced
recalibration, temperature offset, altitude, firmware version, soft reset.

| | |
|---|---|
| Address | 0x61 (fixed; Modbus/PWM modes not supported) |
| Measurement | continuous, interval 2–1800 s (default 2 s, non-volatile) |
| Current | 19 mA average at 1 sample / 2 s, 75 mA max; no sleep command (power-gate it) |
| Boot | < 2 s after power-up / soft reset |
| Output | CO2 ppm, 0.01 °C, 0.01 %RH |
| Source | Sensirion Interface Description SCD30 v1.0 (May 2020), Datasheet SCD30 v1.0 (May 2020) |
| Status | mock-tested |

The sensor sends IEEE 754 floats; they are converted to integer units with
integer-only bit manipulation (`float_bits_to_scaled`), no FPU or soft-float
code. Getters take a delay for the mandatory > 3 ms gap before the read
header. The `Sensor` trait starts continuous measurement on its first call
(next read is `NotReady`), afterwards each cycle is "request (3 ms) → read".

Wiring: VDD 3.3–5.5 V, SEL pin low/floating for I2C. The host I2C master
**must support clock stretching** (up to 30 ms, 150 ms once a day), run at
≤ 100 kHz (50 kHz recommended) and not use repeated starts.
