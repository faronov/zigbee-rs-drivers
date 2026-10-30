# zigbee-stcc4

Sensirion STCC4 miniature thermal-conductivity CO2 sensor over I2C.
Continuous (1 s) and single-shot (500 ms) measurement, RH/T and pressure
compensation input, sleep / exit sleep (`LowPower`), conditioning, soft
reset (general call), factory reset, self test, testing mode, forced
recalibration, product ID + serial (`probe`).

| | |
|---|---|
| Address | 0x64 (ADDR = GND), 0x65 (ADDR = VDD) |
| Conversion | single shot 500 ms; continuous 1 s interval |
| Current | continuous 950 µA, single shot every 10 s 100 µA, idle 55 µA, **sleep 1 µA**, peak 4.2 mA |
| Wake-up | 5 ms (`exit_sleep_mode`, not acknowledged); power-up 10 ms |
| Output | CO2 ppm (380–32 000); T/RH of the attached SHT4x in 0.01 °C / 0.01 %RH |
| Source | Sensirion Datasheet STCC4 v1 (June 2025) |
| Status | mock-tested |

Wiring: VDD 2.7–5.5 V, VDDIO sets the I2C level (1.8 V–VDD; tie low for
VDD). No clock stretching; up to 1 MHz (Fast-mode Plus). The STCC4 needs
T/RH compensation: wire an SHT4x (address 0x44) to SDA_C/SCL_C (internal
10 kΩ pull-ups) — the STCC4 then reports its T/RH — or leave SDA_C/SCL_C
floating, call `set_rht_compensation` with host values and build the
driver `.with_sht4x(false)`.

`soft_reset` uses the I2C general-call reset, which also resets other
devices on the bus that implement it. Run `perform_conditioning` (22 s)
after more than 3 h without measuring. Recommended single-shot interval
for ASC: 5–600 s.
