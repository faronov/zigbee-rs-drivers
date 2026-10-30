# zigbee-soil

Capacitive soil-moisture probes with an analog output: "Capacitive Soil
Moisture Sensor v1.2 / v2.0" (TLC555 / NE555 based), DFRobot SEN0193 and
similar boards.

| | |
|---|---|
| Interface | ADC channel (`zigbee_sensor::adc::Millivolts` / `AsyncMillivolts`), optional supply-enable GPIO |
| Conversion | one ADC sample; with a power pin, `start_measurement` returns the settling time (default 100 ms) |
| Output | `Quantity::SoilMoisture` in 0.01 % from a two-point (dry/wet) linear calibration, plus raw mV |
| Current | ≈ 5 mA while powered; 0 when switched off by the power pin |
| Source | no vendor datasheet for these boards; calibration per probe |
| Status | mock-tested |

Wiring notes:

* v1.2 boards with a TLC555 run from 3.3 V; NE555 boards (often sold as
  v2.0) need 5 V and do not work reliably at 3.3 V.
* Output is about 2.5–3 V dry and 1.2–1.5 V in water at 3.3 V supply;
  keep it within the ADC input range (use the ADC attenuation/divider).
* To save power, feed VCC from a GPIO (≤ 10 mA) or a P-MOSFET/load switch
  and pass it with `with_power_pin(pin, settle_us)`.
* Calibrate with the probe in air (dry) and in a glass of water (wet), or
  in completely dry and saturated samples of the target soil.
