# zigbee-cht831x

Sensylink CHT8310 / CHT8315: temperature and relative humidity over
I2C / SMBus with one-shot standby, PEC-checked reads and an ALERT output.

| | |
|---|---|
| Address | 0x40 (AD0 GND), 0x44 (VCC), 0x48 (SDA), 0x4C (SCL) |
| Conversion | 11 ms typ. (T + RH, no maximum published); driver waits 20 ms and checks the busy bit |
| Standby current | 55 nA typ. (CHT8310), 35 nA typ. (CHT8315) |
| Output | 0.01 °C, 0.01 %RH |
| Features | one-shot or continuous (120 s … 125 ms), 13 / 14-bit temperature (EM), heater, PEC (CRC-8 0x07) on every read, manufacturer and device ID |
| Source | Sensylink CHT8310 datasheet Rev 1.1 (2023-09), CHT8315 datasheet Rev 1.6 |
| Status | mock-tested |

`init` sets SD = 1 so each `start_measurement` writes the one-shot
register; after `set_continuous(Some(rate))` the sensor converts on its
own and `start_measurement` returns 0 without bus traffic. The driver does
not implement `LowPower`: one-shot mode already returns to standby after
each conversion.

The device ID is not in the datasheets. `verify_id` accepts 0x8215
(CHT8310, as read by pvvx firmware) and 0x8315 (CHT8315, Zephyr driver)
and rejects anything else, including a CHT8305 (0x8305) that shares the
manufacturer ID 0x5959.

Reads take the third PEC byte and a mismatch is `Error::Crc`. Writes are
sent without PEC, as the Zephyr and pvvx drivers do; the datasheets do not
say whether a write PEC is checked. The datasheet's PEC example says both
0xE6 and 0xB3; 0xB3 is correct. There is no reset: the General Call reset
resets every device on the bus and the 0xFC soft-reset register is not
documented. Config writes are read-modify-write, which keeps the reserved
bit 11 that the Zephyr driver reports must stay set.
