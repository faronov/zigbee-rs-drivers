# zigbee-bme280

Bosch BME280 (pressure, temperature, humidity) and BMP280 (pressure,
temperature), detected from `chip_id` (0x60 / 0x58).

| | |
|---|---|
| Bus | I2C (0x76 SDO low, 0x77 SDO high) or 4-wire SPI (mode 0/3, ≤ 10 MHz) |
| Mode | forced (one conversion per `start_measurement`, then sleep) |
| Conversion | `1.25 + 2.3·osrs_t + 2.3·osrs_p + 0.575 + 2.3·osrs_h + 0.575` ms max; 9.3 ms (BME280) / 6.4 ms (BMP280) at ×1 |
| Sleep current | 0.1 µA typ. (0.3 µA max) |
| Output | 0.01 °C, Pa, 0.01 %RH (BME280 only) |
| Math | 32-bit temperature/humidity, 64-bit pressure (one 64-bit division per sample) |
| Source | BME280 datasheet BST-BME280-DS002 rev 1.23; BMP280 datasheet BST-BMP280-DS001 rev 1.26 |
| Status | mock-tested |

Wiring: CSB high selects I2C (tie to VDDIO); a falling edge on CSB
switches the part to SPI until the next power cycle. SDO selects the I2C
address and is MISO in SPI mode. `init()` reads the calibration and is
called automatically by the first `start_measurement()`. After
`soft_reset()` wait `RESET_WAIT_US` (2 ms) before the next access.
