# zigbee-cse7766

Chipsea CSE7766 single-phase energy metering IC with a transmit-only UART
(Sonoff POW R2, Sonoff S31 and similar smart plugs).

| | |
|---|---|
| Interface | UART TX only (RI reserved), 4800 bit/s 8E1 (even parity), 24-byte frame streamed continuously (54.9 ms each) |
| Conversion | continuous; `start_measurement` drains stale bytes and returns 110 ms (rest of a frame + one frame) |
| Power-down current | none (no sleep mode) |
| Output | Voltage mV, Current µA, Power mW, Energy mWh, PowerFactor 0.001 (computed) |
| Calibration | factory-calibrated coefficients for 1 mΩ shunt and 1 MΩ / 1 kΩ divider (`Calibration::FACTORY`); other circuits `Calibration::from_circuit(shunt µΩ, R_top Ω, R_bottom Ω)` |
| Source | Chipsea CSE7766 datasheet Rev1.2 |
| Status | mock-tested |

Notes:
- The UART must implement `embedded_io::ReadReady`; `read_measurement`
  never blocks, parses up to 4 frames of buffered bytes and converts the
  newest valid one (`NotReady` without a frame, `Crc` if only corrupt
  frames arrived). Sync is on header 1 (0x55 / 0xAA / 0xFx) followed by
  0x5A; checksum = low byte of Σ bytes 2..=22.
- Value = coefficient / cycle (µs) × board factor in u64 integer math
  (24-bit × 32-bit < 2^56). Fine-tune `voltage_ppm` / `current_ppm` /
  `power_ppm` by `true / reported` against a reference meter.
- Header 0xAA (not calibrated), 0xFx with bit 3 / 2 (voltage / current
  cycle out of range: abnormal circuit or chip damage) or bit 0
  (coefficient storage abnormal) → `InvalidData`. Bit 1 (power cycle out
  of range) → 0 W.
- A cycle without its Adj bit 4/5/6 is incomplete (> 1 s, partial
  count); the driver keeps the last complete value, capped by the value
  of the growing incomplete cycle, so readings decay to 0 when the load
  is switched off instead of flickering.
- Energy: one CF pulse = Pcoef · kv · ki µJ (`pulse_nwh`, ≈ 1.44 mWh at
  Pcoef ≈ 5.2 · 10^6). `PulseCounter` extends the 16-bit CF counter with
  the Adj.7 overflow toggle and applies the datasheet 3.6.2 rules (CFl =
  0 frames ignored; CF is sampled ~50 ms before Adj.7). The first frame
  = pulses since power-up; persist `pulse_counter().total()` and restore
  it with `set_total`. Read at least every ~65 000 pulses (≈ 90 s at
  3.6 kW) or a wrap is missed.
- Datasheet erratum: section 3.5.2 / 3.6.2 divide the voltage (and the
  1000 W pulse rate) by V2R; physically a larger top resistor lowers the
  pin voltage, so the driver multiplies by the divider ratio (kv = V2R),
  as the HLW8032 datasheet does for the same frame format.
- No frequency output; no `LowPower` (no sleep mode). The HLW8032 /
  CSE7759 share the frame format and have their own crate
  (`zigbee-hlw8032`).
