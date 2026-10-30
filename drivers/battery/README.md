# zigbee-battery

Battery voltage via an ADC channel and resistor divider, mapped to the
Zigbee Power Configuration cluster (0x0001) with piecewise-linear
discharge curves.

| | |
|---|---|
| Interface | `zigbee_sensor::adc::Millivolts` / `AsyncMillivolts` (platform ADC, calibrated mV) |
| Conversion | one ADC sample; optional settle time (`with_settle_us`) for switched / high-impedance dividers |
| Power-down current | divider only: V / (R_top + R_bottom) (e.g. 3 V / 1.33 MΩ ≈ 2.3 µA; switch it off with a GPIO/FET for lower) |
| Output | Voltage mV (battery side), BatteryPercent 0.5 % (0..=200) |
| Calibration | `Divider::new(R_top Ω, R_bottom Ω)` (`u64` math, rounded); `Curve` presets or custom tables; `with_compensation_mv` |
| Source | typical manufacturer discharge curves (Li-MnO₂ coin, alkaline, NiMH, Li-ion, LiFePO₄); approximate |
| Status | mock-tested |

Notes:
- Presets: `CR2032`, `CR2450` (same Li-MnO₂ profile), `ALKALINE_2X`
  (2 × AA / AAA), `LI_ION_1S` (Li-ion / LiPo rest voltage), `LIFEPO4_1S`
  (flat plateau → coarse mid-range), `NIMH_2X`. Custom tables:
  `Curve::new(&[Point { mv, half_percent }, ...])` (falling voltage,
  non-increasing charge; validated, `const`).
- Load compensation: a cell sags by `I_load × R_internal` (CR2032:
  10-40 Ω, higher when cold/empty — a 10 mA radio burst costs
  100-400 mV). Sample while the radio is idle, or add the expected drop
  with `with_compensation_mv(mv)`; it only shifts the curve lookup, the
  reported voltage is the measured one. ADC input impedance and sampling
  capacitor charge also load a high-value divider — add a capacitor at the
  pin or a settle time.
- ZCL: `zcl::battery_voltage(m.voltage as u32)` → BatteryVoltage (0x0020,
  100 mV) and `zcl::battery_percentage(m.percent)` →
  BatteryPercentageRemaining (0x0021, 0.5 %); see the crate-level doc
  test.
- Integer math only (one `u64` division per sample in the divider, one
  `u32` division in the interpolation).
