# zigbee-bmp5xx

Bosch BMP580, BMP581 and BMP585 barometric pressure and temperature
sensors. The part is identified from `CHIP_ID` (0x50 BMP580/BMP581, 0x51
BMP585). Output is compensated on-chip; no calibration is read.

| | |
|---|---|
| Bus | I2C (0x46 SDO low, 0x47 SDO high) or 4-wire SPI (mode 0/3, ≤ 12 MHz, no dummy byte) |
| Mode | forced (one conversion, then deep standby) |
| Conversion | `tstartup_deep` 4 ms + 1.05 × (`tconv_p` + `tconv_t`): 6.1 ms (P×1 T×1), 8.1 ms (default P×4 T×1), 90.6 ms (P×128 T×8) |
| Sleep current | 0.55 µA deep standby (1.0 µA standby) |
| Output | 0.01 °C, Pa |
| Source | BMP581 datasheet BST-BMP581-DS004-13 rev 1.13; BMP585 datasheet BST-BMP585-DS003-02 rev 1.2 |
| Status | mock-tested |

Notes:

* `init()` starts with a discarded `CHIP_ID` read, which switches the
  interface to SPI after power-up or reset (datasheet 5.1). It then checks
  `STATUS` (`nvm_rdy = 1`, `nvm_err = 0`, else `InvalidData`) and enables
  the data-ready status flag (`INT_SOURCE`); the INT pin stays disabled.
* `read_measurement()` returns `NotReady` until `INT_STATUS.drdy_data_reg`
  is set. The flag clears on read.
* The IIR filter stays bypassed so the part can enter deep standby between
  conversions (its registers can only be written in STANDBY anyway).
* The soft-reset command is not ACKed over I2C; `soft_reset()` ignores that
  bus result.
* Wiring: tie CSB to VDDIO for I2C; SDO must not float (it sets the
  address). VDD 1.71-3.6 V, VDDIO 1.08-3.6 V. Wait 2 ms after power-up
  before the first access.
