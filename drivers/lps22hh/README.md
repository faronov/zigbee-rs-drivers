# zigbee-lps22hh

ST LPS22HH absolute pressure (260-1260 hPa) and temperature sensor,
one-shot mode. `WHO_AM_I` must read 0xB3. Output is compensated on-chip.

| | |
|---|---|
| Bus | I2C (0x5C SA0 low, 0x5D SA0 high) or 4-wire SPI (mode 0/3, ≤ 10 MHz) |
| Mode | one-shot from power-down (`ODR = 000`); the part returns to power-down by itself |
| Conversion | not specified by ST; the driver waits one period of the mode's maximum ODR (low-current 200 Hz → 5 ms, low-noise 75 Hz → 13.34 ms) and checks `STATUS` |
| Supply current | 4 µA (low-current) / 12 µA (low-noise) at 1 Hz; 0.9 µA power-down |
| Output | 0.01 °C (100 LSB/°C), Pa (4096 LSB/hPa, rounded) |
| Source | LPS22HH datasheet DocID030890 Rev 2 (February 2019) |
| Status | mock-tested |

Notes:

* `Mode` maps to `CTRL_REG2.LOW_NOISE_EN` (default low-current, as after
  reset). The datasheet allows changing it only in power-down, which is
  where the part sits between one-shot conversions.
* `read_measurement()` burst-reads `STATUS..TEMP_OUT_H` and returns
  `NotReady` until `P_DA` and `T_DA` are both set.
* `soft_reset()` sets `SWRESET`; no time is specified, the driver
  suggests 5 ms.
* Wiring: CS high selects I2C/I3C; SDO/SA0 must not float. VDD 1.7-3.6 V.
