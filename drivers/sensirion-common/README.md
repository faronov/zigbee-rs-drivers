# zigbee-sensirion-common

Shared I2C framing for the Sensirion drivers in this repository: 16-bit
big-endian words with CRC-8 (polynomial 0x31, init 0xFF), command encoding
with argument words, "send command and fetch result" sequences, NACK
handling (Sensirion parts NACK their read header while busy or when no data
is available) and the common `-45 + 175·S/65535` / `100·S/65535` transfer
functions. Blocking (`blocking::Interface`) and async (`asynch::Interface`).

Not a sensor driver by itself; used by `zigbee-sht4x`, `zigbee-scd4x`,
`zigbee-sen5x`, ...

| | |
|---|---|
| Source | Sensirion datasheets (word/CRC format, e.g. SCD4x v1.7 §3.3/§3.12) |
| Status | mock-tested |
