# zigbee-lps2xdf

ST LPS22DF and LPS28DFW absolute pressure and temperature sensors,
one-shot mode. Both read `WHO_AM_I = 0xB4`, so the caller selects the
`Variant`. Output is compensated on-chip.

| | |
|---|---|
| Parts | LPS22DF (260-1260 hPa, I2C/I3C/SPI); LPS28DFW (water-resistant, 260-1260 or 260-4060 hPa, I2C/I3C) |
| Bus | I2C (0x5C SA0 low, 0x5D SA0 high) or 4-wire SPI (LPS22DF only, mode 0/3) |
| Mode | one-shot from power-down (`ODR = 0000`); the part returns to power-down by itself |
| Conversion | not specified by ST; the driver waits one period of the maximum ODR for the selected averaging: AVG 4/8/16/32/64/128/512 → 2/2.5/3.33/5/10/13.33/40 ms, then checks `STATUS` |
| Supply current | one-shot at 1 Hz: 1.7 µA (AVG 4) to 32.2 µA (AVG 512), default AVG 16 2.7 µA; 0.9 µA power-down |
| Output | 0.01 °C (100 LSB/°C), Pa (4096 LSB/hPa, or 2048 LSB/hPa in LPS28DFW mode 2; rounded) |
| Source | LPS22DF datasheet DS13316 Rev 3; LPS28DFW datasheet DS13317 Rev 1 |
| Status | mock-tested |

Notes:

* `Config { avg, full_scale }` is written to `CTRL_REG1`/`CTRL_REG2` by
  `init()`. `FullScale::Hpa4060` on an LPS22DF returns `Unsupported`.
* The LPS28DFW datasheet text quotes a 2096 LSB/hPa mode-2 sensitivity in
  one place; its table, register description and worked example use 2048,
  which the driver follows.
* `read_measurement()` burst-reads `STATUS..TEMP_OUT_H` and returns
  `NotReady` until `P_DA` and `T_DA` are both set.
* `soft_reset()` sets `SWRESET`; no time is specified, the driver
  suggests 5 ms.
* Wiring: CS high selects I2C/I3C (LPS22DF); SDO/SA0 must not float.
  VDD 1.7-3.6 V.
