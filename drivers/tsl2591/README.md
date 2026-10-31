# zigbee-tsl2591

ams TSL2591 high-dynamic-range (600 M:1) light-to-digital converter: full
spectrum (CH0) and infrared (CH1) photodiodes over I2C.

| | |
|---|---|
| Address | 0x29 (fixed) |
| ID | `ID` register 0x12 = 0x50 (`probe()`) |
| Mode | `PON | AEN` → wait → check `AVALID`, read 4 data bytes in one transfer → disable; `with_continuous(true)` keeps the ALS running |
| Gain / IT | ×1 / ×25 / ×428 / ×9876; 100–600 ms (full scale 37888 at 100 ms, else 65535) |
| Lux | ams reference formula `(CH0 − CH1)(1 − CH1/CH0)·408 / (ATIME_ms · gain)`, integer milli-lux |
| Wait | 2 ms + 108 ms (max step) per 100 ms of ATIME |
| Sleep current | 2.3 µA typ. (PON = 0) |
| Output | milli-lux, raw CH0 / CH1, saturation flag |
| Interrupt | persisted and no-persist CH0 windows, persistence filter, sleep-after-interrupt, force / clear special functions |
| Helper | `suggest_gain()` (pure auto-gain step) |
| Source | ams TSL2591 datasheet ams163.5 (April 2013); lux formula from ams reference code (Linux `tsl2591` IIO driver, Adafruit library) |
| Status | mock-tested |

The datasheet does not publish a lux equation; the device factor (408)
and formula are the ones ams ships in its reference drivers.
