# zigbee-mcp9600

Microchip MCP9600 / MCP9601 thermocouple EMF-to-temperature converter,
blocking and async.

| | |
|---|---|
| Parts | MCP9600, MCP9601 (adds short-circuit detection); thermocouple types K, J, T, N, S, E, B, R |
| Bus / address | I2C 0x60–0x67 (ADDR pin divider), standard mode ≤ 100 kHz, clock stretching up to 60 µs |
| Conversion | burst (default, 1 sample) or continuous; `(max(ADC, cold junction) + 12 ms) × 1.25` per sample: 18-bit ADC 415 ms, 12-bit/coarse CJ 93.75 ms; burst of 2^n samples (n = 0..7) multiplies |
| Sleep current | shutdown 2 µA typ; burst mode returns to shutdown after each conversion |
| Output | hot junction (cold-junction compensated) and cold junction in 0.01 °C (register LSB 0.0625 °C); raw 24-bit ADC via `read_raw_adc()` |
| Wiring | thermocouple to VIN+/VIN−, VIN− biased per datasheet; ALERT pins unused |
| Errors | input range / open circuit or short (MCP9601) → `InvalidData`; update flag not set → `NotReady`; unknown device ID → `UnexpectedId` |
| Errata | MCP9600 with device-ID revision ≤ 0x12 (date code before 1845) may repeat the first byte of multi-byte reads above ~85 kHz; check `DeviceId::has_i2c_errata()` and run the bus ≤ 85 kHz (DS80000741) |
| Source | Microchip MCP960X/L0X/RL0X datasheet DS20005426; errata DS80000741 |
| Status | mock-tested |

Every register access ends with a STOP condition (pointer write, then a
separate read), as required by the datasheet.
