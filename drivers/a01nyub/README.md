# zigbee-a01nyub

DFRobot waterproof ultrasonic distance sensors with UART output:
A01NYUB (SEN0313) and A02YYUW (SEN0311).

| | |
|---|---|
| Interface | UART 9600 8N1, sensor TX only (`embedded_io::Read + ReadReady` / `embedded_io_async::Read`) |
| Frame | `FF H L SUM`, `SUM = (FF + H + L) & 0xFF`, distance = `H << 8 \| L` mm |
| Conversion | free running: processed value every 100–300 ms (RX high/floating) or real-time value every ~100 ms (RX low) |
| Output | `Quantity::Distance` in mm, plus an in-range flag |
| Range | A01NYUB 280–7500 mm, A02YYUW 30–4500 mm |
| Current | A01NYUB ≤ 15 mA, A02YYUW < 8 mA |
| Source | DFRobot wiki SEN0313 / SEN0311 |
| Status | mock-tested |

Notes:

* The blocking driver never waits: `read_measurement` drains the bytes already
  in the UART and returns the newest valid frame, `NotReady` if there is none
  or `Crc` if only corrupt frames arrived. Poll it at least every ~100 ms or
  give the UART a receive buffer of a few frames.
* The async driver awaits the next valid frame; wrap it in a timeout.
* The parser resynchronises on `0xFF`, which may also appear as a data byte.
* The RX pin selects the mode; use `set_output_mode` with a GPIO wired to it,
  or leave RX unconnected for processed output.
* No "no target" value is documented; out-of-range readings are flagged via
  `Measurement::in_range`.
