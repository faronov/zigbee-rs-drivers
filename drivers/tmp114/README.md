# zigbee-tmp114

TI TMP114 (±0.2 °C max at 20–50 °C; TMP114N ±1 °C): ultra-thin 1.2–1.8 V digital temperature sensor over
I2C (4-ball DSBGA, no ALERT pin).

| | |
|---|---|
| Address | fixed per variant: TMP114A–D 0x48–0x4B, TMP114ND/NC/NB/NA 0x4C–0x4F |
| Conversion | one-shot from shutdown, 6.4 ms typ. / 7.5 ms max; ×8 with averaging (51.2 / 60 ms) |
| ADC time | 6.4 / 3.5 / 2.0 / 1.2 ms (shorter = noisier) |
| Resolution | 0.0078125 °C result, 0.03125 °C limits |
| Continuous periods | 6.4 ms, 32 / 16 / 8 / 4 / 2 / 1 / 0.5 Hz |
| Shutdown current | 0.16 µA typ. (68 µA during a conversion) |
| Alert | T_LOW/T_HIGH limits with separate hysteresis (0.5 °C steps); status/flags in `Alert_Status` only |
| ID | Device_ID 0x0B = 0x?114 (top nibble = revision) |
| Output | 0.01 °C |
| Source | TI SNIS214E (July 2023) |
| Status | mock-tested |

## Usage notes

* Call `configure()` once after power-up (the part starts in continuous
  4 Hz mode). It writes the configuration with Mode = 1 (shutdown).
* `start_measurement()` sets OS (one-shot); the part returns to shutdown by
  itself. `LowPower::sleep` = shutdown, `wake` = continuous.
* The returned waits for the 3.5 / 2.0 / 1.2 ms ADC settings are the typical
  value + 17 % (the data sheet only gives a maximum for 6.4 ms).
* There is no ALERT pin, so the limits cannot wake the host; poll
  `alert_status()` (reading clears the latched flags and Data_Ready).
* CRC on writes (CRC_En) is not used.
