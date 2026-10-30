# zigbee-lps25hb

ST LPS25HB absolute pressure (260-1260 hPa) and temperature sensor,
one-shot mode. `WHO_AM_I` must read 0xBD.

| | |
|---|---|
| Bus | I2C (0x5C SA0 low, 0x5D SA0 high, ≤ 400 kHz) or 4-wire SPI (mode 0/3, ≤ 10 MHz) |
| Mode | one-shot (`PD = 1`, `ODR = 000`); `sleep()` clears `PD` |
| Conversion | not specified by ST; the driver waits one 25 Hz period (40 ms) and checks `STATUS` |
| Supply current | 4 µA at 1 Hz with `Config::LOW_POWER` (`RES_CONF = 04h`), 25 µA with the default high-resolution `0Fh`; 0.5 µA power-down |
| Output | 0.01 °C (42.5 °C + raw/480), Pa (4096 LSB/hPa, rounded) |
| Source | LPS25HB datasheet DocID027112 Rev 4 (August 2016); AN4672 DocID027655 Rev 3 |
| Status | mock-tested |

Notes:

* The +42.5 °C temperature offset is not printed in the LPS25HB
  datasheet (only the 480 LSB/°C sensitivity); it comes from ST's LPS25H
  documentation and ST's LPS25HB standard C driver.
* Multi-byte reads set the auto-increment bit: sub-address bit 7 over
  I2C, bit 6 (`MS`) over SPI.
* `read_measurement()` burst-reads `STATUS..TEMP_OUT_H` and returns
  `NotReady` until `P_DA` and `T_DA` are both set.
* `reboot()` sets `BOOT` (trimming reload, 2.2 ms); the driver suggests
  3 ms and re-initialises on the next start.
* Wiring: CS high selects I2C; SDO/SA0 must not float. VDD 1.7-3.6 V.
