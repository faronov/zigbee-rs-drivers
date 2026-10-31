# zigbee-bl0937

Belling BL0937 and HLW8012 pulse-output energy metering ICs (Sonoff POW,
Tuya / BlitzWolf smart plugs).

| | |
|---|---|
| Interface | CF (power pulses), CF1 (current RMS with SEL low, voltage RMS with SEL high), SEL output pin |
| Conversion | platform-defined counting window (default 1 s); SEL switching ≤ 10 µs |
| Power-down current | none (≈ 3 mA operating, no sleep mode) |
| Output | Voltage mV, Current µA, Power mW, Energy mWh, PowerFactor 0.001 (computed) |
| Calibration | `Calibration::from_circuit(Chip, shunt µΩ, R_top Ω, R_bottom Ω)`; `BL0937_REFERENCE` (1 mΩ, 6×330 kΩ / 1 kΩ), `HLW8012_REFERENCE` (1 mΩ, 6×470 kΩ / 1 kΩ) |
| Source | Belling BL0937 datasheet V1.02; HLW8012 datasheet REV 1.3 |
| Status | mock-tested |

Notes:
- The driver does not count pulses: the platform measures CF / CF1 with a
  timer/counter or edge interrupts and passes `Pulses { count,
  duration_us }` (whole periods between first and last edge, or one
  captured period with `Pulses::period`). No edge → `Pulses::NONE`.
- `start_measurement` drives SEL and returns the window; `read_measurement`
  converts the supplied windows and, in `Cf1Mode::Alternate`, switches SEL
  to the other channel right after a CF1 window. Voltage / current keep
  their latest value.
- Energy: feed every CF pulse once via `add_energy_pulses` (hardware
  counter); `energy_pulses` / `set_energy_pulses` persist the total.
  BL0937 reference ≈ 1.707 J per pulse.
- Conversion is `count · factor · 1000 / duration_us` (one u64 division
  per quantity, also for energy) — acceptable on Cortex-M0 at reporting
  rates.
- Nominal constants (Vref, oscillator) vary ±several % between parts:
  calibrate against a reference load by scaling a factor by
  `true / reported`.
- The BL0937 V1.02 formula legend swaps the V(V)/V(I) labels; the driver
  uses V(V) = voltage channel, V(I) = shunt voltage.
- No `LowPower`: the parts have no sleep mode.
