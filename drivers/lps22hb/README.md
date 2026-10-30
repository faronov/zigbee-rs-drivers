# zigbee-lps22hb

ST LPS22HB absolute pressure (260-1260 hPa) and temperature sensor,
one-shot mode. `WHO_AM_I` must read 0xB1. Output is compensated on-chip.

| | |
|---|---|
| Bus | I2C (0x5C SA0 low, 0x5D SA0 high, ≤ 400 kHz) or 4-wire SPI (mode 0/3, ≤ 10 MHz) |
| Mode | one-shot from power-down (`ODR = 000`); the part returns to power-down by itself |
| Conversion | not specified by ST; the driver waits one 75 Hz period (13.34 ms) and checks `STATUS` |
| Supply current | 12 µA (low-noise) / 3 µA (low-current) at 1 Hz; 1 µA power-down |
| Output | 0.01 °C (100 LSB/°C), Pa (4096 LSB/hPa, rounded) |
| Source | LPS22HB datasheet DocID027083 Rev 6 (June 2017); AN4833 DocID029003 Rev 2 |
| Status | mock-tested |

Notes:

* `Mode::LowCurrent` sets `RES_CONF.LC_EN`; the datasheet allows this only
  in power-down, which is where the part sits between one-shot conversions.
* `read_measurement()` burst-reads `STATUS..TEMP_OUT_H` and returns
  `NotReady` until `P_DA` and `T_DA` are both set. `BDU` is left off: with
  I2C it would require single-byte reads (`IF_ADD_INC = 0`).
* `soft_reset()` sets `SWRESET`; wait 3 ms (also after power-up).
* Wiring: CS high selects I2C; SDO/SA0 must not float. VDD 1.7-3.6 V.
