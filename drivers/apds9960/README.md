# zigbee-apds9960

Broadcom APDS-9960 RGBC / ambient light + proximity sensor, over I2C.

| | |
|---|---|
| Address | 0x39 (fixed) |
| ID | `ID` = 0xAB (also accepts 0x9C, 0xA8 and 0x9E, as ESPHome does) |
| Mode | `PON` + `AEN`/`PEN` → wait → check `AVALID`/`PVALID` → one burst read of status, C/R/G/B and `PDATA` → power down; `with_continuous(true)` keeps measuring |
| Gain / IT | ALS ×1/4/16/64, 1–256 cycles of 2.78 ms; proximity ×1/2/4/8, 12.5–100 mA LED, 4–32 µs × 1–64 pulses |
| Lux | DN40 estimate (`G''·GA·DF/(T_ms·AGAIN)`, IR removed), with TCS3472 coefficients by default; configurable |
| CCT | DN40 `CT_coef·B'/R' + CT_offset` |
| Wait | 7 ms exit-sleep + ALS integration + proximity time (`t_INIT + t_CNVT + n·t_ACC`, Table 2) + 10 % |
| Standby current | 1 µA typ. (sleep, PON = 0) |
| Output | milli-lux, CCT, raw C/R/G/B, 8-bit proximity (`Quantity::Proximity`), `CPSAT`/`PGSAT` saturation flags |
| Interrupt | clear-channel and proximity windows with persistence (`set_interrupt()`); `take_interrupt()` clears them with `AICLEAR` |
| Helper | `suggest_gain()`, `max_count()`, `dn40()`, `set_proximity_offset()` |
| Source | Broadcom AV02-4191EN datasheet |
| Status | mock-tested |

Notes:

* **Gesture engine skipped.** `GEN` is never set, and the gesture FIFO,
  `GCONF*` and gesture offsets are not exposed.
* Broadcom publishes no lux or CCT equation for this part. The default
  `DN40_DEFAULT` coefficients are the ams TCS3472 open-air values. The
  RGBC engine is the same TAOS design, so treat the result as an estimate.
  Calibrate it with `with_coefficients()` / `with_glass_attenuation()`.
* The wait timer and sleep-after-interrupt (`SAI`) are not used.
