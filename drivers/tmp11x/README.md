# zigbee-tmp11x

TI TMP116 / TMP117 / TMP119: ±0.1 °C class 16-bit digital temperature
sensors over I2C / SMBus, with EEPROM-backed configuration and ALERT.

| | |
|---|---|
| Address | 0x48 (ADD0 = GND), 0x49 (V+), 0x4A (SDA), 0x4B (SCL) |
| Conversion | one-shot from shutdown: 15.5 ms typ. / 17.5 ms max per conversion (TMP116 17 ms), × 1 / 8 / 32 / 64 averages |
| Resolution | 0.0078125 °C, 16-bit |
| Continuous cycles | 15.5 ms, 125 / 250 / 500 ms, 1 / 4 / 8 / 16 s (never shorter than the averaging time) |
| Shutdown current | TMP116 0.25 µA, TMP117/TMP119 0.15 µA typ. |
| Alert | high/low limits (7.8 m°C steps), alert (latched) or therm mode, polarity, or Data_Ready on ALERT |
| ID | Device ID 0x0F: TMP116 0x1116, TMP117 0x0117, TMP119 0x2117 |
| Output | 0.01 °C |
| Source | TI SBOS740A (TMP116, May 2019), SNOSD82D (TMP117, Sept 2022), SNIS236 (TMP119, Jan 2024) |
| Status | mock-tested |

## Usage notes

* `probe()` returns and remembers the `Variant` (TMP119 shares DID 0x117
  with the TMP117 and is told apart by revision nibble 2).
* Call `configure()` once after power-up: the factory EEPROM default is
  continuous 1 s / 8 averages. `start_measurement()` writes MOD = 11 and
  returns 17.5 ms × averages (140 ms for the default 8). Use
  `Averaging::None` for the cheapest sample. `LowPower::sleep` = shutdown,
  `wake` = continuous.
* `read_measurement()` returns `NotReady` while the register still reads
  the −256 °C power-up value.
* `set_offset()` writes the volatile offset register (TMP117/TMP119).
  EEPROM programming (unlock register 0x04, ~7 ms per word, limited write
  endurance) is not implemented; configure at run time instead.
* In alert mode the flags and ALERT latch until the configuration register
  is read (`read_flags()`); therm mode is level-based with hysteresis
  between the high and low limits. ALERT is open-drain.
