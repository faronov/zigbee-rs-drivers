# zigbee-sgp30

Sensirion SGP30 multi-pixel MOX gas sensor over I2C. Driver features:

- TVOC (ppb) and CO2eq (ppm) from the on-chip algorithm (`init_air_quality`, `measure_iaq`)
- raw H2 and ethanol signals
- IAQ baseline get and set, plus the inceptive TVOC baseline
- absolute-humidity compensation, with an integer T/RH → g/m³ helper
- feature set (`probe`)
- measure test
- soft reset (general call)

| | |
|---|---|
| Address | 0x58 |
| Conversion | `measure_iaq` 12 ms, `measure_raw` 25 ms, `measure_test` 220 ms, other commands ≤ 10 ms |
| Current | measuring 48.8 mA; **sleep 2 µA, only after power-up or soft reset** |
| Output | CO2eq 400–60 000 ppm, TVOC 0–60 000 ppb |
| Source | Sensirion Datasheet SGP30, version 1.0 (May 2020, D1) |
| Status | mock-tested |

## Wiring

VDD and VDDH need 1.62–1.98 V (typically 1.8 V). Use a level shifter for a 3.3 V MCU. The bus runs up to 400 kHz.

## Measurement cadence

**The caller must send `measure_iaq` once per second** after `iaq_init`, otherwise the dynamic baseline compensation is degraded. The first `start_measurement` sends `iaq_init`, and the read that follows returns `NotReady`. For the first 15 s the sensor reports 400 ppm and 0 ppb.

## Baselines

Store the `get_baseline` values in non-volatile memory at regular intervals. After a power-up or soft reset, send `iaq_init` and then restore them with `set_baseline`; the wire order is TVOC first, then CO2eq. The inceptive TVOC baseline (`get_tvoc_inceptive_baseline` / `set_tvoc_baseline`) needs feature set 34 or later, and is for the first start-up only.

## Power

There is no sleep command, so the crate has no `LowPower` implementation. `soft_reset` uses the I2C general-call reset, which also resets other devices on the bus. Switching the supply is the alternative.

## Humidity helper

`absolute_humidity()` turns T and RH into the 8.8 g/m³ format. It interpolates a 5 °C saturation table and reads up to about 1.5 % high between table points.
