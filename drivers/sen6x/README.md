# zigbee-sen6x

Driver for the Sensirion SEN6x environmental sensor nodes over I2C.

| Variant | Address | PM mass | Number conc. | RH / T | VOC / NOx index | CO2 | HCHO |
|---|---|---|---|---|---|---|---|
| SEN60 | 0x6C | ✓ | ✓ (in each sample) | | | | |
| SEN62 | 0x6B | ✓ | ✓ | ✓ | | | |
| SEN63C | 0x6B | ✓ | ✓ | ✓ | | ✓ | |
| SEN65 | 0x6B | ✓ | ✓ | ✓ | ✓ | | |
| SEN66 | 0x6B | ✓ | ✓ | ✓ | ✓ | ✓ | |
| SEN68 | 0x6B | ✓ | ✓ | ✓ | ✓ | | ✓ |
| SEN69C | 0x6B | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |

Every variant has its own `Read Measured Values` command and word layout; `Variant` selects them. The SEN60 has a separate address and command set (SCD4x-style command IDs); the driver maps it automatically.

The driver supports:

- start and stop, data-ready flag, number concentrations
- device status (SEN62–SEN69C can also read and clear it)
- product name (`probe` picks the SEN62–SEN69C variant), serial number, firmware version
- reset and fan cleaning
- SHT heater and its result
- temperature offset slots and temperature acceleration
- VOC algorithm state backup and restore
- for CO2 variants: forced recalibration, automatic self-calibration, ambient pressure, altitude and CO2 factory reset

Commands a variant lacks return `Unsupported` without touching the bus. VOC/NOx tuning and raw values are not implemented.

| | |
|---|---|
| Address | 0x6B; SEN60 0x6C |
| Conversion | free-running at 1 s; first sample about 1.1 s after start. Commands take 20 ms (SEN60 1 ms). Start takes 50 ms, stop 1.4 s (SEN60 1 s), reset 1.2 s. |
| Output | PM in 0.1 µg/m³; number concentration in 0.01 #/cm³ (sensor resolution 0.1 #/cm³) as `Quantity::ParticlesBelow` (500/1000/2500/4000/10 000 nm bins); RH 0.01 %RH; T 0.01 °C (sensor resolution 0.005 °C, truncated); VOC and NOx index 1–500; CO2 in ppm; HCHO in ppb (rounded from 0.1 ppb) |
| Unknown values | 0xFFFF (unsigned) or 0x7FFF (signed) become `None`. This covers NOx for the first 11 s, SEN66 CO2 for the first 6 s, SEN63C/SEN69C CO2 for the first 24 s, and HCHO for the first 60 s. |
| Current | measuring 75–90 mA typ (SEN66 up to 110 mA, 350 mA peak); idle 1.7 mA (SEN62/SEN63C) or 3.3–4.6 mA; **no sleep mode** |
| Source | Sensirion Datasheet SEN6x, version 1.0 (D1), for SEN62–SEN69C; version 0.9 (November 2024, D1, engineering samples) for the SEN60 commands |
| Status | mock-tested |

## Wiring

- **Connector:** ACES 51451-0060N-001 (cable side JST GHR-06V-S). Pin 1 is VDD, pin 2 GND, pin 3 SDA, pin 4 SCL. Pins 5 and 6 duplicate GND and VDD.
- **Supply:** VDD is 3.3 V (3.15–3.6 V; absolute max 3.6 V). SDA and SCL are 5 V tolerant.
- **Pull-ups:** use external pull-ups, e.g. 10 kΩ.
- **Bus:** up to 100 kHz. There is no clock stretching; the sensor NACKs while busy. Keep cables under 10 cm where possible (50 cm max).

## Measurement cycle

`start_measurement` starts continuous measurement on its first call and returns 1.1 s; the read that follows is `NotReady`. Each later call requests the latest values and returns 20 ms (SEN60: 1 ms). The SEN60 buffer empties on read: if no new sample has arrived, the sensor NACKs and the read is `NotReady`.

SEN60 samples include the number concentrations. The other variants read them with a separate command. With `with_number_concentration(true)`, every second cycle reads them instead of the measured values; that sample has only `number_concentration` set. The option is off by default. The SEN6x does not report a typical particle size.

The SEN63C and SEN69C condition their CO2 sensor for 24 s after a start. Do not restart within that time.

## Power

There is no sleep mode, so the crate has no `LowPower` implementation. To save power, stop measuring (idle), or switch the 3.3 V supply off.

## Volatile settings

Temperature offsets and acceleration, ASC, pressure and altitude revert on reset. The forced recalibration result persists.
