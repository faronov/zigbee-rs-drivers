# zigbee-tmp102

TI TMP102 and TMP112 (TMP112A/B/N/Dx): 12/13-bit digital temperature
sensors over I2C / SMBus.

| | |
|---|---|
| Address | 0x48 (ADD0 = GND), 0x49 (V+), 0x4A (SDA), 0x4B (SCL); TMP112Dx variants have fixed addresses |
| Conversion | one-shot from shutdown; 15 ms max (TMP102 rev. I), 11.25 ms max (TMP112), 35 ms on pre-2024 silicon (driver default) |
| Resolution | 0.0625 °C, 12-bit (−55…+128 °C) or extended 13-bit (to +150 °C) |
| Continuous rates | 0.25 / 1 / 4 / 8 Hz |
| Shutdown current | 0.15 µA typ., 0.35 µA max |
| Alert | T_LOW/T_HIGH thresholds, comparator or interrupt mode, fault queue 1/2/4/6, polarity |
| Output | 0.01 °C |
| Source | TI SBOS397I (TMP102, June 2024), SBOS473L (TMP112, July 2024) |
| Status | mock-tested (previous zigbee-rs TMP102 driver: hw) |

## Usage notes

* Call `configure()` once after power-up: the part powers up in continuous
  4 Hz mode (~5 µA). `configure()` writes the configuration with `SD = 1`.
* `start_measurement()` writes `OS = 1, SD = 1`: one conversion, then the
  part goes back to shutdown. `LowPower::sleep` = shutdown, `wake` =
  continuous mode at the configured rate.
* To wake a sleepy end device on temperature, use **comparator** mode: the
  ALERT pin keeps its state while the part is in shutdown. In interrupt
  mode entering shutdown clears ALERT. Thresholds are only compared after
  a conversion, so one-shot sampling must continue (or use continuous mode
  at 0.25 Hz, ~3 µA).
* The part has no ID register; `probe()` checks the read-only resolution
  bits R1/R0 = 11 in the configuration register.
* ALERT is open-drain: pull-up required. ADD0 must not float.
