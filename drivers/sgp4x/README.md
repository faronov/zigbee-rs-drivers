# zigbee-sgp4x

Sensirion SGP40 (VOC) and SGP41 (VOC + NOx) metal-oxide gas sensors over
I2C, plus a fixed-point port of Sensirion's Gas Index Algorithm
(`gas_index` module) that turns the raw signals into the VOC index
(1–500, 100 = average) and NOx index (1–500, 1 = average).

| | |
|---|---|
| Parts | SGP40, SGP41 |
| Address | 0x59 (fixed) |
| Conversion | SGP40 `measure_raw_signal` 30 ms; SGP41 `measure_raw_signals` / `execute_conditioning` 50 ms; self test 320 ms |
| Current | idle (heater off) 34 µA; measuring at 3.3 V: SGP40 2.6 mA, SGP41 3.0 mA (4.2 mA conditioning) |
| Output | raw SRAW_VOC / SRAW_NOX ticks, VOC index, NOx index |
| Sources | Datasheet SGP40 v1.2 (Feb 2022), Datasheet SGP41 v1.0 (Dec 2021), gas-index-algorithm 3.2.2 (fixpoint) |
| Status | mock-tested; gas index bit-exact against the C fixpoint reference |

Usage notes:

* **Cadence:** the gas index algorithm requires one sample per second
  with the hotplate kept on; call `start_measurement` / `read_measurement`
  at 1 Hz. The indices read 0 (not reported) during the first 45 s and the
  algorithm needs hours to learn its baseline. Persist
  `voc_algorithm().states()` and restore with `set_states` after short
  (< 10 min) interruptions.
* **SGP41 conditioning:** after power-up or heater off, the driver sends
  `execute_conditioning` for the first 10 one-second cycles (configurable
  with `with_conditioning_samples`, never more than 10 s).
* **Humidity compensation:** pass ambient T/RH (e.g. from an SHT4x) with
  `set_compensation`; otherwise the default 25 °C / 50 %RH values keep the
  on-chip compensation off.
* `LowPower::sleep` turns the heater off (34 µA) — this interrupts the
  1 Hz series; reset the algorithms after long pauses.
* `soft_reset` is an I2C general-call reset that also resets other devices
  on the bus.
* VDD and VDDH 1.7–3.6 V from the same supply; up to 400 kHz.

## License note

`src/gas_index.rs` is a port of Sensirion's BSD-3-Clause
`sensirion_gas_index_algorithm` (Copyright (c) 2022, Sensirion AG); its
copyright notice and license text are kept in the file and must be
reproduced with binary distributions.
