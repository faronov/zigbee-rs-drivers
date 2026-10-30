# zigbee-tcn75a

Microchip TCN75A and MCP9800 / MCP9801 / MCP9802 / MCP9803 digital
temperature sensors over I2C / SMBus, with ALERT output. One driver
(`Tcn75a`, alias `Mcp980x`) covers all five.

| | TCN75A | MCP9800 / MCP9802 | MCP9801 / MCP9803 |
|---|---|---|---|
| Address | 0x48–0x4F (A2..A0 pins) | 0x48–0x4F fixed by ordering code A0–A7 (SOT-23-5) | 0x48–0x4F (A2..A0 pins) |
| Conversion | one-shot from shutdown; 30 / 60 / 120 / 240 ms typ. (9–12 bit) | same; max 75 / 150 / 300 / 600 ms | same |
| Resolution | 9–12 bit (0.5 … 0.0625 °C) | 9–12 bit | 9–12 bit |
| Shutdown current | 0.1 µA typ., 2 µA max | 0.1 µA typ., 1 µA max | 0.1 µA typ., 1 µA max |
| Alert | ALERT pin, T_SET / T_HYST (9-bit, 0.5 °C), comparator or interrupt, polarity, fault queue 1/2/4/6 | same | same |
| Bus timeout | none | MCP9802 35 ms typ.; MCP9800 none | MCP9803 35 ms typ.; MCP9801 none |
| ID | none | none | none |
| Output | 0.01 °C | 0.01 °C | 0.01 °C |
| Source | Microchip DS21935D (2010) | Microchip DS21909D (2010) | DS21909D |
| Status | mock-tested | mock-tested | mock-tested |

## Usage notes

* The driver waits the MCP980x maximum conversion time (75 ms at 9 bit,
  doubling per bit) for every part; the TCN75A data sheet only gives
  typical values for the same core.
* One-shot is honoured only while in shutdown. `configure()` enters
  shutdown. If the driver has not yet written shutdown (fresh driver, or
  after `wake()`), `start_measurement()` writes it before the one-shot.
  `LowPower::sleep` = shutdown, `wake` = continuous (200 µA typ.).
* Comparator-mode ALERT stays asserted through shutdown, so it works as a
  wake-up line with one-shots. The data sheet says it deasserts only in
  continuous conversion with T_A below T_HYST, so after handling the
  alert call `wake()`, wait one conversion, then `sleep()`. Interrupt-mode ALERT is cleared by shutdown and by
  any register read.
* Limits are 9-bit (0.5 °C) in bits 15:7 regardless of the resolution.
