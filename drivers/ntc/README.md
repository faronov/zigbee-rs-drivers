# zigbee-ntc

NTC thermistor in a resistor divider read through any
`zigbee_sensor::adc::Millivolts` channel, blocking and async, with
integer-only Beta, Steinhart-Hart and lookup-table models.

| | |
|---|---|
| Parts | any NTC thermistor (e.g. 10 kΩ B3950, `Model::B3950_10K`); Beta (R0, T0, β), Steinhart-Hart (A, B, C × 1e15) or user `(Ω, 0.01 °C)` table |
| Bus | ADC channel via `Millivolts` / `AsyncMillivolts` (platform HAL) |
| Conversion | immediate (`start_measurement` returns 0); optional averaging of n samples |
| Sleep current | none in the driver; the divider draws Vs / (Rs + R) (≈ 165 µA at 3.3 V, 10 k/10 k, 25 °C), so power it from a GPIO on battery devices |
| Output | 0.01 °C and resistance in mΩ; model math error ≤ 0.01 °C (fixed-point ln, error < 2/65536), accuracy limited by ADC resolution, Rs tolerance and the thermistor's own tolerance |
| Wiring | `NtcToGround`: Rs from supply to node, NTC to ground; `NtcToSupply`: NTC from supply to node, Rs to ground. Ratiometric: power the divider from the ADC reference and set `supply_mv` to the full-scale reading; otherwise pass a measured rail to `measure_with_supply` |
| Errors | node at 0 mV or ≥ supply (shorted/open thermistor) or temperature outside the model/table → `InvalidData`; ADC error → `Bus` |
| Source | Beta and Steinhart-Hart equations; test vectors from the 10 kΩ B3950 Beta curve and the classic A = 1.009249522e-3, B = 2.378405444e-4, C = 2.019202697e-7 set |
| Status | mock-tested |
