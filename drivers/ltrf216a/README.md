# zigbee-ltrf216a

Lite-On LTR-F216A ambient light sensor (photopic + clear channels, up to
20-bit), over I2C.

| | |
|---|---|
| Address | 0x53 (fixed) |
| ID | `PART_ID` = 0xB1 |
| Mode | enable → wait → check `MAIN_STATUS` data flag → burst-read status..ALS → standby; `with_continuous(true)` keeps measuring |
| Gain / IT | ×1/3/6/9/18; 25–400 ms (16–20 bit) |
| Lux | `0.45 · ALS · WFAC / (GAIN · T_int/100 ms)` |
| Wait | 10 ms wake-up + integration time + 10 % |
| Output | milli-lux, ALS and clear counts, saturation flag |
| Interrupt | 20-bit ALS window, persistence 1–16 (`set_interrupt()`) |
| Helper | `suggest_gain()` |
| Source | Linux `iio/light/ltrf216a.c` (Lite-On co-authored), Zephyr `sensor/liteon/ltrf216a` |
| Status | mock-tested |

Notes:

* The Lite-On datasheet is not publicly available. The register map, part
  ID, `ALS_MEAS_RES` values and lux factor come from the Linux and Zephyr
  drivers. Gain codes, rate behaviour and the `INT_CFG`/`INT_PST` bit
  layout are taken from the Broadcom APDS-9306 datasheet, which has the
  identical register map and `PART_ID` 0xB1 (see `zigbee-apds9306`).
* The part does not acknowledge the software reset, so `reset()` ignores
  bus errors (as Linux does).
* The LTR-308 uses the same map with a 0.6 lux factor and no clear
  channel; it is not handled by this crate.
