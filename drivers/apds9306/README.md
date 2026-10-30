# zigbee-apds9306

Broadcom APDS-9306 / APDS-9306-065 ambient light sensor (photopic + clear
channels, 13–20 bit), over I2C.

| | |
|---|---|
| Address | 0x52 (fixed) |
| ID | `PART_ID` = 0xB1 (APDS-9306) / 0xB3 (APDS-9306-065); `probe()` selects the variant, `with_variant()` overrides |
| Mode | enable → wait → check `MAIN_STATUS` data flag → burst-read status..ALS → standby; `with_continuous(true)` keeps measuring |
| Gain / IT | ×1/3/6/9/18; 3.125 ms (13 bit), 25–400 ms (16–20 bit) |
| Lux | `ALS · K / (GAIN · T_int/3.125 ms)`, K = 16.326432 (‑9306) / 14.09712 (‑9306‑065) |
| Wait | 10 ms wake-up + integration time + 10 % |
| Standby current | 2 µA typ. (ALS disabled) |
| Output | milli-lux, ALS and clear counts, saturation flag |
| Interrupt | 20-bit ALS window or variance mode (8·2ⁿ counts), persistence 1–16 (`set_interrupt()`) |
| Helper | `suggest_gain()` |
| Source | Broadcom APDS-9306/APDS-9306-065 datasheet; lux factor as in Linux `iio/light/apds9306.c` |
| Status | mock-tested |

Notes:

* The datasheet has no lux equation, only the response point (2000 counts
  at 3×, 100 ms for 49.8 µW/cm² resp. 43 µW/cm² at 530 nm). The K factors
  are the Linux driver's derivation (340.134 lx resp. 293.69 lx at that
  point), scaled linearly with gain and integration time.
* The repeat rate is set to its fastest code; the part then converts
  back-to-back at the selected resolution.
* The interrupt source is fixed to the ALS channel.
