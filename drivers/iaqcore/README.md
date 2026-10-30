# zigbee-iaqcore

ams iAQ-Core C and iAQ-Core P indoor air quality modules: CO2-equivalent
(ppm), TVOC-equivalent (ppb) and sensor resistance, read as one 9-byte
I2C read.

| | |
|---|---|
| Bus | I2C 0x5A (fixed), 100 kHz, clock stretching required; read-only (never address the module for writing) |
| ID | none |
| Sampling | C: continuous, 1 s (66 mW); P: pulsed, 11 s (9 mW) |
| Warm-up | status RUNIN for ≈ 5 min after power-on |
| Output | CO2 eq. 450–2000 ppm, TVOC eq. 125–600 ppb, resistance Ω |
| Supply | 3.3 V |
| Sleep | none — power-gate the module (requires a new 5 min run-in) |
| Source | ams "iAQ-Core Indoor Air Quality Module" datasheet v1-00 (2015-Apr-30) |
| Status | mock-tested |

## Usage

`start_measurement` returns `0` (free-running). `read_measurement` returns
`NotReady` for BUSY and RUNIN, `InvalidData` for ERROR. Poll no faster than
`Variant::interval_us()`. `read_frame` returns the raw result including the
status, e.g. to show warm-up progress.

The CO2-equivalent value is a VOC-derived prediction, so it is reported as
`Quantity::Eco2`, not `Co2`.

## Wiring

| iAQ-Core | MCU |
|---|---|
| VDD | 3.3 V |
| SDA, SCL | I2C with pull-ups (the host must support clock stretching) |
| GND | GND |
