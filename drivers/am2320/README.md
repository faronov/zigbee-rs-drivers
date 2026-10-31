# zigbee-am2320

Aosong AM2320: temperature and relative humidity over I2C with the
Modbus-like register protocol (function 0x03, CRC-16 Modbus).

| | |
|---|---|
| Address | 0x5C (0xB8 write byte), fixed |
| Bus | ≤ 100 kHz; ≥ 30 µs between read address and first data clock |
| Transaction | wake (NACK ignored) → 1 ms → read command → 2 ms → 8-byte answer; three phases via `start_measurement` / `read_measurement` (`Error::Pending`) |
| Interval | ≥ 2 s; each answer holds the values measured after the previous transaction |
| Sleep | automatic after each transaction (or 3 s timeout) |
| Output | 0.01 °C, 0.01 %RH (0.1 resolution, sign-magnitude temperature) |
| Features | CRC-16 check, device info (model / version / ID) |
| Source | Aosong AM2320 product manual §8.2 |
| Status | mock-tested |

The datasheet gives no expected model / ID values, so there is no
`verify_id`; `device_info` returns the raw registers.

The AM2315C (AHT21-type protocol) is supported by `zigbee-aht`
(`Variant::Am2315c`). The AM2320 single-wire mode is the DHT22 protocol
(`zigbee-dht`).

Multi-phase: the first `read_measurement` after waking sends the read
command and returns `Error::Pending(READ_WAIT_US)`; the next call reads the
answer. The core `measure` helpers follow this automatically.
