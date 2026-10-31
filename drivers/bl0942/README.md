# zigbee-bl0942

Belling BL0942 single-phase energy metering IC (smart plugs, e.g. Tuya /
Sonoff modules), UART or SPI.

| | |
|---|---|
| Interface | UART 8N1 4800/9600 (pin) or 19200/38400 (MODE), chip address 0-3 by A2/A1 (SSOP10: 0); SPI mode 1 ≤ 900 kHz (SEL = 1) |
| Conversion | RMS/power refresh 400 ms (800 ms option); UART full packet 23 bytes ≈ 48 ms at 4800 bit/s |
| Power-down current | none (≈ 3 mA operating, no sleep mode) |
| Output | Voltage mV, Current µA, Power mW, Energy mWh, Frequency mHz, PowerFactor 0.001 (computed) |
| Calibration | `Calibration::from_circuit(shunt µΩ, R_top Ω, R_bottom Ω, Vref µV)`; `REFERENCE` = 1 mΩ, 5×390 kΩ / 510 Ω, 1.218 V |
| Source | Belling BL0942 datasheet V1.06 |
| Status | mock-tested |

Notes:
- UART needs `embedded_io::ReadReady`: `start_measurement` drains stale
  bytes, sends `58|addr AA` and returns the transfer time;
  `read_measurement` returns `NotReady` until all 23 bytes arrived. Bytes
  before the 0x55 header are skipped; checksum `!(cmd + Σ bytes)`.
- SPI has no full packet: `read_measurement` reads I_RMS, V_RMS,
  I_FAST_RMS, WATT, CF_CNT, FREQ and STATUS (7 × 6-byte frames, each
  checksum-verified). `start_measurement` returns 0.
- Register access over both buses: `request_register` + `read_requested`,
  `write_register`, `unlock` (USR_WRPROT = 0x55), `set_mode`,
  `soft_reset`.
- Energy: one CF_CNT pulse = mW/LSB · 1638.4 · 256 s (≈ 186.9 mWh on the
  reference circuit, coarse). `EnergyCounter` extends the 24-bit counter
  (first read = pulses since power-up); restore a saved total with
  `energy_counter().set_total()`. Keep MODE.CF_CNT_CLR off.
- Calibration assumes current gain 16 (default GAIN_CR); fine-tune by
  scaling a factor by `true / reported` against a reference meter. VP
  input impedance is ≈ 370 kΩ, which slightly lowers a 510 Ω bottom
  resistor.
- Power factor = P / (V·I) (i64 division), clamped to ±1.000; noisy at
  low load.
- Multi-chip UART: keep ≥ 20 ms between frames when switching chips; the
  UART resets on a > 20 ms gap between bytes.
- No `LowPower`: the part has no sleep mode.
