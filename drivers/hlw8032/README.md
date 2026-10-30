# zigbee-hlw8032

HLW8032 (and the pin/protocol-compatible CSE7759) single-phase energy
metering IC with a transmit-only UART (smart plugs, DIN-rail meters).

| | |
|---|---|
| Interface | UART TX only, 4800 bit/s 8E1 (even parity), 24-byte frame every ~50 ms |
| Conversion | continuous; `start_measurement` drains stale bytes and returns 110 ms (rest of a frame + one frame) |
| Power-down current | none (no sleep mode) |
| Output | Voltage mV, Current µA, Power mW, Energy mWh, PowerFactor 0.001 (computed) |
| Calibration | `Calibration::from_circuit(shunt µΩ, R_top Ω, R_bottom Ω)` → Kv = R_top / (R_bottom · 1000), Ki = 1 / (R_shunt · 1000); `REFERENCE` = 1 mΩ, 4 × 470 kΩ / 1 kΩ (Kv 1.88, Ki 1) |
| Source | HLW Technology HLW8032 datasheet REV 1.5 |
| Status | mock-tested |

Notes:
- The UART must implement `embedded_io::ReadReady`; `read_measurement`
  never blocks, parses up to 4 frames of buffered bytes and converts the
  newest valid one (`NotReady` without a frame, `Crc` if only corrupt
  frames arrived). Sync is on a State byte (0x55 / 0xAA / 0xFx) followed
  by 0x5A; checksum = low byte of Σ bytes 2..=22.
- Value = parameter register / cycle register × coefficient, all u64
  integer math (24-bit × 32-bit < 2^56). The factory parameter registers
  are per chip; fine-tune `voltage_ppm` / `current_ppm` / `power_ppm`
  by `true / reported` against a reference meter.
- State 0xAA or bit 0 (parameter registers unusable) → `InvalidData`.
  Overflow bits 3/2/1 mean the quantity is ≈ 0 and report 0.
- A register without its Data Update bit holds an incomplete cycle; the
  driver keeps the last complete value, capped by the value of the
  growing incomplete cycle, so readings decay to 0 when the load is
  switched off instead of flickering.
- Energy: one PF pulse = PP · Kv · Ki µJ (`pulse_nwh`, ≈ 1.44 mWh for
  PP ≈ 5.2 · 10^6). `PulseCounter` extends the 16-bit PF REG with the
  Data Update bit 7 overflow toggle (the first frame = pulses since
  power-up); persist `pulse_counter().total()` and restore it with
  `set_total`. Read at least every ~65 000 pulses (≈ 90 s at 3.6 kW on
  the reference board) or a wrap is missed.
- No frequency output (the chip does not report the line period).
- No `LowPower`: the part has no sleep mode.
