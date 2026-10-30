# zigbee-tmp1075

TI TMP1075 / TMP1075N: LM75-compatible 12-bit digital temperature sensor
over I2C / SMBus, with ALERT (comparator or interrupt).

| | |
|---|---|
| Address | TMP1075: 0x40–0x5F (A2/A1/A0 = GND/V+/SDA/SCL), 0x48 with all pins at GND; TMP1075N: 0x48–0x4B |
| Conversion | one-shot from shutdown: TMP1075 5.5 ms typ. / 7 ms max, TMP1075N 10 / 15 ms |
| Resolution | 0.0625 °C, 12-bit |
| Continuous periods | 27.5 / 55 / 110 / 220 ms (TMP1075N fixed 250 ms) |
| Shutdown current | 0.37 µA typ. (TMP1075N 0.15 µA) |
| Alert | T_LOW/T_HIGH (comparator hysteresis = HLIM − LLIM), fault queue 1–4 (TMP1075N 1/2/4/6), polarity, SMBus alert |
| ID | Device ID 0x0F = 0x7500 (TMP1075 only) |
| Output | 0.01 °C |
| Source | TI SBOS854F (June 2024) |
| Status | mock-tested |

## Usage notes

* Pick the orderable with `with_variant(Variant::Tmp1075N)` for the SOT-563
  part: it sets the one-shot wait and makes `probe()` check the read-only
  R1:R0 = 11 bits instead of the (missing) Device ID register.
* The configuration register is written as a single byte (bits 15:8): the
  TMP1075N requires it, the TMP1075 accepts it.
* Call `configure()` once after power-up (the part starts in continuous
  mode). `start_measurement()` writes OS = 1 with SD = 1; the part converts
  once and shuts down. `LowPower::sleep` = shutdown, `wake` = continuous.
* On the TMP1075 every configuration write (which includes TM) releases
  ALERT and clears the fault counter. With one-shot sampling ALERT is
  therefore re-evaluated after every conversion; use `FaultQueue::One`, as
  higher counts never accumulate across one-shots.
* ALERT is open-drain: pull-up required.
