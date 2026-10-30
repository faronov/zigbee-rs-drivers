# zigbee-tmp108

TI TMP108: ±0.75 °C 12-bit digital temperature sensor (WCSP) over I2C /
SMBus, with window-comparator ALERT.

| | |
|---|---|
| Address | 0x48 (A0 = GND), 0x49 (V+), 0x4A (SDA), 0x4B (SCL) |
| Conversion | one-shot from shutdown, 27 ms typ., 33 ms max |
| Resolution | 0.0625 °C, 12-bit |
| Continuous rates | 0.25 / 1 / 4 / 16 Hz (1 / 2 / 5 / 18 µA typ.) |
| Shutdown current | 0.3 µA typ., 1 µA max |
| Alert | T_LOW/T_HIGH window, hysteresis 0/1/2/4 °C, comparator or interrupt, polarity; FH/FL flags |
| Output | 0.01 °C |
| Source | TI SBOS663A (September 2019) |
| Status | mock-tested |

## Usage notes

* Call `configure()` once after power-up (the part starts in continuous
  1 Hz mode). It writes the configuration with M1:M0 = 00 (shutdown).
* `start_measurement()` writes M1:M0 = 01: one conversion, then shutdown.
  `LowPower::sleep` = shutdown, `wake` = continuous at the configured rate.
* Comparator mode keeps ALERT asserted while the last conversion was
  outside the window, which survives shutdown and can wake a sleepy end
  device. The window is only re-evaluated after a conversion.
* No ID register: `probe()` only checks that the reserved configuration
  bits read 0. In interrupt mode reading the configuration register
  (`read_flags`, `conversion_done`, `probe`) clears FH/FL and ALERT.
* ALERT is open-drain: pull-up required.
