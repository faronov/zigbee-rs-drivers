# zigbee-bmp3xx

Bosch BMP388 and BMP390 barometric pressure and temperature sensors. The
part is identified from `CHIP_ID` (0x50 BMP388, 0x60 BMP390).

| | |
|---|---|
| Bus | I2C (0x76 SDO low, 0x77 SDO high) or 4-wire SPI (mode 0/3, ≤ 10 MHz; one dummy byte after the read address) |
| Mode | forced (one conversion, then sleep) |
| Conversion | datasheet `T_conv` × 19/16: 5.9 ms (P×1 T×1), 13.1 ms (default P×4 T×1), 87.5 ms (P×32 T×2) |
| Sleep current | 2 µA (BMP388), 1.4 µA (BMP390) |
| Output | 0.01 °C, Pa |
| Math | 64-bit fixed-point port of the datasheet float formulas (≤ 0.5 Pa / 0.005 °C from the float reference) |
| Source | BMP390 datasheet BST-BMP390-DS002-07 rev 1.7; BMP388 datasheet BST-BMP388-DS001-07 rev 1.7 |
| Status | mock-tested |

Notes:

* `read_measurement()` returns `NotReady` until both `drdy_press` and
  `drdy_temp` are set; the status is read separately from the data burst
  so a conversion finishing mid-read cannot clear the flags unseen.
* The IIR filter state is kept between forced conversions; it is reset
  when `press_en`/`temp_en` change, which this driver never does.
* Wiring: CSB high at power-up selects I2C; SDO must not float (it sets
  the I2C address). VDD 1.65-3.6 V, VDDIO 1.2-3.6 V.
