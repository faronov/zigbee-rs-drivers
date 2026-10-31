# zigbee-hts221

STMicroelectronics HTS221: capacitive temperature and relative humidity
sensor over I2C or 3-wire SPI, with per-device factory calibration.

| | |
|---|---|
| Address | 0x5F (fixed); SPI when CS is low |
| Conversion | one-shot; no conversion time in the datasheet, driver waits 80 ms (one 12.5 Hz period) and checks T_DA / H_DA |
| Power-up | not specified; `wake` returns 0 |
| Current | 2 µA at 1 Hz (default averaging), 0.5 µA power-down (typ., 2.5 V) |
| Output | 0.01 °C, 0.01 %RH (clipped to 0–100 %RH) |
| Features | WHO_AM_I check, calibration read and linear interpolation in integer math, averaging (AV_CONF), heater, BOOT, power-down |
| Source | ST DocID026333 HTS221 datasheet, Rev 4 (August 2016) |
| Status | mock-tested |

The bus is behind a small `Interface` trait (`read(reg, buf)`,
`write(reg, value)`); `I2cInterface` and `SpiInterface` implement it for
`embedded-hal` and `embedded-hal-async`. SPI is 3-wire (shared SDI/SDO),
so the `SpiDevice` must handle half-duplex reads on that line.

`init` checks the ID, reads the 16 calibration bytes once and sets PD and
BDU. Calibration can also be injected with `with_calibration` to save the
read after a deep sleep. Higher averaging settings lengthen the conversion
beyond 80 ms; `read_measurement` then returns `NotReady` and the caller
should retry. Do not measure while the heater is on.
