# zigbee-tcs34725

ams (TAOS) TCS3472x RGBC colour light sensor with IR filter, over I2C.
Illuminance and CCT follow ams DN40, using integer arithmetic.

| | |
|---|---|
| Parts | TCS34721 / TCS34725 (ID 0x44), TCS34723 / TCS34727 (ID 0x4D) |
| Address | 0x29 (fixed) |
| Mode | `PON+AEN` → wait → check `AVALID` → burst-read status + C/R/G/B → sleep; `with_continuous(true)` keeps measuring |
| Gain / IT | ×1/4/16/60; 1–256 cycles of 2.4 ms (2.4–614 ms) |
| Lux | DN40: IR removal `(R+G+B−C)/2`, `G'' = 0.136R' + G' − 0.444B'`, `lux = G''·GA·DF/(T_ms·AGAIN)` with DF = 310 |
| CCT | DN40: `3810·B'/R' + 1391` K (`Measurement::cct`) |
| Wait | 2.4 ms warm-up + integration time + 10 % |
| Standby current | 2.5 µA typ. (sleep, PON = 0) |
| Output | milli-lux, CCT, raw C/R/G/B, saturation flag (DN40 ripple rule) |
| Interrupt | 16-bit clear-channel window, `APERS` persistence (`set_interrupt()`, `take_interrupt()` sends the clear special function) |
| Helper | `suggest_gain()`, `saturation()`, `dn40()` |
| Source | TAOS135 TCS3472 datasheet; ams DN40 coefficients for the TCS3472 (as also used by ESPHome / Adafruit) |
| Status | mock-tested |

Notes:

* DN40 is not in the local datasheet set. The TCS3472 open-air
  coefficients (R 0.136, G 1.0, B −0.444, DF 310, CT 3810/1391) were
  checked against ESPHome `tcs34725.cpp`. Unlike ESPHome, this driver
  applies the DN40 IR subtraction, as Adafruit's `calculateColorTemperature_dn40`
  does.
* Behind a cover glass, set `with_glass_attenuation()` or supply new
  coefficients via `with_coefficients()`.
* The wait timer (`WEN`/`WTIME`) is not used.
