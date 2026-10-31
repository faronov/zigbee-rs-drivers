# zigbee-pzem004t

Peacefair PZEM-004T v3.0 / PZEM-016 (AC) and PZEM-003 / PZEM-017 (DC)
energy monitor modules over Modbus RTU.

| | |
|---|---|
| Interface | Modbus RTU 9600 bit/s: PZEM-004T TTL UART 8N1 (5 V module supply), PZEM-016 RS-485 8N1, PZEM-003/017 RS-485 8N2; addresses 0x01-0xF7, 0xF8 = general (single slave) |
| Conversion | request/response: transfer time + 50 ms assumed processing (AC read ≈ 84 ms) |
| Power-down current | none (mains powered module) |
| Output | AC: Voltage, Current, Power, Energy, Frequency, PowerFactor, power alarm. DC: Voltage, Current, Power, Energy, high/low voltage alarms |
| Calibration | factory calibrated; PZEM-017 shunt range (50/100/200/300 A) by `request_shunt_range` |
| Source | Peacefair PZEM-004T V3.0 user manual; PZEM-003/017 manual |
| Status | mock-tested |

Notes:
- The UART must implement `embedded_io::ReadReady`. Requests drain
  stale RX bytes first; `read_measurement` / `read_reply` never block and
  return `NotReady` until the reply is complete (also forever if the
  module is silent — the caller times out). Replies from other addresses
  are skipped; with the general address 0xF8 any reply address is
  accepted.
- CRC: `zigbee_sensor::crc::crc16_modbus`, low byte first (standard
  Modbus; the manuals' "CRC high byte + low byte" wording is not what
  the modules send). Exception replies → `Unsupported` (0x01 illegal
  function) or `InvalidData`.
- Scaling (integer): AC V 0.1 V, I 0.001 A (32-bit, low word first),
  P 0.1 W (32-bit), E 1 Wh (32-bit), F 0.1 Hz, PF 0.01; DC V 0.01 V,
  I 0.01 A (16-bit), P 0.1 W, E 1 Wh. Values beyond `i32` saturate.
- Parameters: `request_parameter(reg)` / `request_write(reg, value)` /
  `request_reset_energy()` then `read_reply()` (writes are confirmed by
  the echoed frame). Registers in `param`: AC power alarm (1 W), DC
  high / low voltage alarm (0.01 V), Modbus address (0x01-0xF7, else
  `InvalidArgument`; the driver follows a confirmed address change), PZEM-017 shunt range. Function 0x41
  (factory calibration) is intentionally not exposed.
- Energy is kept by the module (non-volatile) and only cleared by the
  reset command.
- No `LowPower`: the modules have no sleep mode.
