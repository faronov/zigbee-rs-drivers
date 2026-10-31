# zigbee-bme680

Bosch BME680 and BME688: temperature, pressure, humidity and raw gas
resistance (Ω) of the heated metal-oxide sensor. The variant is read from
`variant_id` (BME688 uses a different gas ADC and formula). No BSEC: IAQ /
eCO2 / bVOC are proprietary and not computed.

| | |
|---|---|
| Bus | I2C (0x76 SDO low, 0x77 SDO high) or 4-wire SPI (mode 0/3, ≤ 10 MHz, page switching handled) |
| Mode | forced, one heater step (`res_heat_0` / `gas_wait_0`) |
| Conversion | 1.963 ms per oversample + 5.293 ms overhead + heater time (default T×2 P×16 H×1, 300 °C / 100 ms → 142.6 ms) |
| Sleep current | 0.15 µA typ. (heater: ~9-13 mA while heating) |
| Output | 0.01 °C, Pa, 0.01 %RH, gas resistance in Ω (`gas_valid` / `heat_stab` flags) |
| Math | integer datasheet formulas; BME680 gas needs one 64-bit division |
| Source | BME680 datasheet BST-BME680-DS001 rev 1.9; BME688 datasheet BST-BME688-DS000 rev 1.3 |
| Status | mock-tested |

Notes:

* `res_heat_0` depends on the ambient temperature; the driver uses the last
  measured temperature (25 °C before the first sample) and rewrites it
  before each heated measurement.
* `Readings` publishes the gas resistance only when both `gas_valid` and
  `heat_stab` are set. Use `Config { heater: None, .. }` for T/P/RH only.
* The integer pressure formula is up to ~8 Pa away from the float one.
* Wiring: CSB high selects I2C; a falling edge on CSB switches to SPI until
  power-off. SDO selects the I2C address.
