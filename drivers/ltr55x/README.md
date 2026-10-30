# zigbee-ltr55x

Lite-On LTR-553ALS-01 (external IR LED) / LTR-559ALS-01 (integrated LED)
ambient light + proximity sensor, over I2C.

| | |
|---|---|
| Address | 0x23 (fixed) |
| ID | `PART_ID[7:4]` = 0x9 (rev 0x2), `MANUFAC_ID` = 0x05 |
| Mode | ALS and/or PS (`with_channels()`): active → wait → check new-data flags → read → standby; `with_continuous(true)` stays active |
| ALS gain / IT | ×1/2/4/8/48/96; 50–400 ms |
| PS | 11-bit count, LED 5–100 mA, 1–15 pulses at 60 kHz / 100 % duty, 50 ms rate, 10-bit offset (`set_ps_offset()`) |
| Lux | LTR-303/55x Appendix A piece-wise formula (integer ×10⁴ coefficients), optional window factor |
| Wait | 10 ms wake-up + max(ALS repeat period, 50 ms PS period) + 10 % |
| Output | milli-lux + CH0/CH1, `Quantity::Proximity` raw count, ALS/PS saturation flags |
| Interrupt | ALS (CH0) and PS windows, separate persistence, polarity (`set_interrupt()`, call in standby) |
| Helper | `suggest_gain()` |
| Source | Linux `ltr501` IIO driver, ESPHome `ltr_als_ps`, Pimoroni `ltr559-python` |
| Status | mock-tested |

Notes:

* The Lite-On LTR-553/559 datasheets could not be downloaded; the
  register map and bit fields were taken from the three open-source drivers
  above, which agree with each other and with the LTR-303 ALS core.
* The PS gain bits (PS_CONTR[3:2]) only exist on LTR-558/659 and are left
  at 0.
* Allow 100 ms after power-up before the first `start_measurement`.
