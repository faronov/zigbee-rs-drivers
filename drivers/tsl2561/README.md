# zigbee-tsl2561

TAOS / ams TSL2560 (SMBus) and TSL2561 (I2C) light-to-digital converters:
two photodiode channels (visible + IR, IR only) and the datasheet's
empirical lux formula.

| | |
|---|---|
| Address | 0x29 (ADDR SEL GND), 0x39 (float), 0x49 (VDD) |
| ID | `ID` register 0x0A, PARTNO 0000/0001 (CS) or 0100/0101 (T/FN/CL) |
| Mode | power up → wait one integration → read CH0/CH1 → power down (no one-shot mode, no ready flag); `with_continuous(true)` stays powered |
| Gain / IT | ×1, ×16; 13.7 / 101 / 402 ms (full scale 5047 / 37177 / 65535) |
| Lux | integer `CalculateLux()` from the datasheet, T/FN/CL or CS coefficient table, scaled to milli-lux |
| Wait | IT + 10 % + 2 ms |
| Power-down current | 3.2 µA typ. |
| Output | milli-lux, raw CH0 / CH1, saturation flag |
| Interrupt | CH0 low/high thresholds (raw), level / SMBAlert / test mode, persistence; `clear_interrupt()` |
| Helper | `suggest_gain()` (pure auto-gain step) |
| Source | TAOS059N (March 2009) |
| Status | mock-tested |

The package is not taken from the ID register automatically (TAOS059N only
documents PARTNO 0000/0001; shipping T-package parts report 0x5x); pick the
coefficient table with `with_package()`. Manual integration (INTEG = 11) is
not supported.
