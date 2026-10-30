# zigbee-lm75

LM75-family digital temperature sensors over I2C / SMBus with OS/ALERT
thermostat output: NXP LM75A, LM75B, PCT2075, TI TMP75, onsemi NCT75.

| Variant | Result | Limits | Single conversion (wait used) | Shutdown current |
|---|---|---|---|---|
| LM75A | 11-bit, 0.125 °C | 9-bit, 0.5 °C | emulated, 100 ms typ. (150 ms) | 3.5 µA typ. |
| LM75B | 11-bit, 0.125 °C | 9-bit, 0.5 °C | emulated, 10 ms typ. (15 ms) | 1.0 µA typ. |
| PCT2075 | 11-bit, 0.125 °C | 9-bit, 0.5 °C | emulated, 28 ms typ. (42 ms) | < 1.0 µA |
| TMP75 | 9–12-bit | 12-bit | OS bit, 27.5–220 ms typ. / 37.5–300 ms max | 0.1 µA typ., 3 µA max |
| NCT75 | 12-bit, 0.0625 °C | 12-bit | one-shot register, 48.5 ms typ. (73 ms) | 3 µA typ., 12 µA max |

| | |
|---|---|
| Address | 0x48–0x4F (A2..A0); PCT2075 has 27 addresses with tri-state pins |
| Alert | T_OS / T_HYST (T_HIGH / T_LOW), comparator or interrupt, polarity, fault queue 1/2/4/6 (TMP75 1/2/3/4) |
| Extras | TMP75 resolution (R1:R0); PCT2075 Tidle sample period (`set_sample_period`) |
| ID | none |
| Output | 0.01 °C |
| Source | NXP LM75A Rev. 04 (2007-07-10), NXP LM75B Rev. 6.1 (2015-02-06), NXP PCT2075 Rev. 10.1 (2021-09-15), TI SBOS288M (Dec 2020), onsemi NCT75/D Rev. 7 (June 2018) |
| Status | mock-tested |

## Usage notes

* Select the part with the `Variant` passed to `new`; there is no ID
  register to detect it.
* Call `configure()` once after power-up (all parts start converting). It
  writes the configuration in shutdown (NCT75: one-shot mode, D5 = 1).
* TMP75: `start_measurement()` writes OS = 1 + SD = 1. NCT75: writes the
  one-shot register 0x04 (data ignored). LM75A/LM75B/PCT2075 have no
  one-shot: `start_measurement()` clears SHUTDOWN (a conversion starts at
  once) and `read_measurement()` sets it again after reading.
* Data sheets for LM75A/LM75B/PCT2075/NCT75 give only typical conversion
  times; the driver waits typical × 1.5. Override with
  `with_conversion_wait_us`.
* The default `Config` selects 12-bit on TMP75 (300 ms max); use
  `Resolution::Bits9` for a 37.5 ms conversion.
* In interrupt mode reading any register or entering shutdown clears
  OS/ALERT; use comparator mode to wake a sleepy end device. In comparator
  mode the output keeps the last conversion's state through shutdown.
* OS/ALERT is open-drain: pull-up required.
