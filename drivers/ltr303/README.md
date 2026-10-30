# zigbee-ltr303

Lite-On LTR-303ALS-01 / LTR-329ALS-01 two-channel (visible+IR / IR)
ambient light sensor, over I2C. The LTR-329 is the LTR-303 without the INT
pin.

| | |
|---|---|
| Address | 0x29 (fixed) |
| ID | `PART_ID[7:4]` = 0xA, `MANUFAC_ID` = 0x05 |
| Mode | active → wait → check `ALS_STATUS` new/valid → read CH1, CH0 → standby; `with_continuous(true)` stays active |
| Gain / IT | ×1/2/4/8/48/96; 50–400 ms (repeat rate set to the shortest ≥ IT) |
| Lux | Appendix A piece-wise formula on CH1/(CH0+CH1), integer ×10⁴ coefficients, optional window factor |
| Wait | 10 ms wake-up + repeat period + 10 % |
| Standby current | 5 µA max |
| Output | milli-lux, CH0, CH1, saturation flag |
| Interrupt | LTR-303: 16-bit window, persistence 1–16, polarity (`set_interrupt()`, call in standby) |
| Helper | `suggest_gain()` |
| Source | Lite-On LTR-303ALS-01 and LTR-329ALS-01 datasheets (BNS-OD-FC002/A4) |
| Status | mock-tested |

Notes:

* The datasheet refers to a separate "Appendix A" for the lux formula; the
  coefficients used are the ones published there and used by ESPHome and
  Adafruit.
* The LTR-329 datasheet shows `PART_ID` default 0xA0 in the register table
  but 0x92 in the bit diagram; the LTR-303 says 0xA0. `probe()` accepts
  part number 0xA only.
* Allow 100 ms after power-up before the first `start_measurement`.
