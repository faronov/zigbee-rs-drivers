# zigbee-sen5x

Driver for the Sensirion SEN50, SEN54 and SEN55 environmental sensor nodes over I2C.

| Variant | PM1.0 / 2.5 / 4.0 / 10 | RH / T | VOC index | NOx index |
|---|---|---|---|---|
| SEN50 | ✓ | | | |
| SEN54 | ✓ | ✓ | ✓ | |
| SEN55 | ✓ | ✓ | ✓ | ✓ |

The driver supports:

- start and stop, RHT/gas-only mode (SEN54/55), data-ready flag
- number concentrations and typical particle size (`Read Measured PM Values`)
- temperature compensation (offset, slope, time constant), warm start and RH/T acceleration
- VOC and NOx tuning, VOC state backup and restore
- fan cleaning and its interval
- product name (`probe` picks the variant), serial number, firmware version
- device status and reset

`Variant` hides outputs the part does not have. The sensor also reports unknown values (0xFFFF for PM, 0x7FFF for RH, T and the indices); these become `None`.

| | |
|---|---|
| Address | 0x69 |
| Conversion | free-running at 1 s; `Read Measured Values` takes 20 ms; start 50 ms, stop 200 ms, reset 100 ms |
| Output | PM in 0.1 µg/m³; number concentration in 0.01 #/cm³ (sensor resolution 0.1 #/cm³) as `Quantity::ParticlesBelow` (500/1000/2500/4000/10 000 nm bins); typical particle size in nm; RH 0.01 %RH; T 0.01 °C (sensor resolution 0.005 °C, truncated); VOC and NOx index 1–500 |
| Current | measuring 63–70 mA (100 mA peak); RHT/gas-only 6.5–6.8 mA; idle 0.7 mA (SEN55 2.6–3.8 mA); **no sleep mode** |
| Source | Sensirion Datasheet SEN5x, version 2 (March 2022, D1). `Read Measured PM Values` (0x0413) is not in the datasheet; its layout follows Sensirion's `embedded-i2c-sen5x` driver. |
| Status | mock-tested |

## Wiring

- **Supply:** VDD is 4.5–5.5 V. Wait 50 ms after power-on before the first command.
- **Interface select:** tie SEL to GND at power-up to select I2C.
- **Pull-ups:** use external pull-ups, e.g. 10 kΩ. VIH min is 2.31 V, so 3.3 V logic works.
- **Bus:** up to 100 kHz, no clock stretching. Keep cables under 10 cm.

## Measurement cycle

`start_measurement` starts the configured `Mode` on its first call and returns 1 s; the read that follows is `NotReady`. Each later call requests the latest values and returns 20 ms.

Number concentrations and typical particle size come from a separate command. With `with_number_concentration(true)` in `Mode::Full`, every second cycle on the SEN54/SEN55 reads them, together with PM mass but without RH/T/VOC/NOx. The SEN50 reads them every cycle, because it has PM only. The option is off by default.

## Power

There is no sleep mode, so the crate has no `LowPower` implementation. To save power, stop measuring (idle), or switch the 5 V supply off.

## Volatile settings

All settings are volatile: a reset or power cycle restores the defaults. Set the VOC/NOx tuning parameters in Idle-Mode only. Warm start and RH/T acceleration take effect at the next start.
