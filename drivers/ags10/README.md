# zigbee-ags10

Aosong (ASAIR) AGS10 MEMS metal-oxide TVOC sensor (ethanol-calibrated,
0–99999 ppb) with CRC-checked frames, zero-point calibration, resistance
and version readout, and programmable I2C address.

| | |
|---|---|
| Bus | I2C 0x1A (programmable), **SCL ≤ 15 kHz** |
| ID | none (firmware version via `request_version`/`read_version`) |
| Timing | data/resistance 1.5 s processing; other commands 30 ms; ≥ 30 ms between any commands; sampling period ≥ 2 s |
| Pre-heat | 120 s after power-on (longer stabilisation after storage, see DS table 4) |
| Current | 28 ± 5 mA (75 mW), no sleep mode — power-gate the sensor if needed |
| Supply | 3.0 ± 0.1 V (> 3 V damages the sensor) |
| CRC | CRC-8 poly 0x31 init 0xFF over Data1..4 |
| Source | Aosong "Datasheet AGS10" (June 2022) |
| Status | mock-tested |

## Usage

`start_measurement` selects register 0x00 and returns 1.5 s;
`read_measurement` returns `NotReady` while the RDY status bit is set
(pre-heating or polling faster than every 1.5 s). The datasheet notes that
the sensor needs a continuous supply for accurate readings, so power-gating
trades accuracy for current. `calibrate_zero` (after ≥ 15 min in fresh air)
is volatile; `set_address` is stored permanently.

## Wiring

| AGS10 | MCU |
|---|---|
| VCC | 3.0 V |
| SDA, SCL | I2C, 2–10 kΩ pull-ups, bus clocked ≤ 15 kHz |
| GND | GND |
