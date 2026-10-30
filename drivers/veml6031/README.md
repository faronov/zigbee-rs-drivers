# zigbee-veml6031

Vishay VEML6031X00 (ALS + IR, AEC-Q100) and VEML6035 (ALS + WHITE, low
transmittance lens designs) ambient light sensors over I2C. The two parts
have different register maps; the crate has one driver for each.

| | `Veml6031` | `Veml6035` |
|---|---|---|
| Address | 0x29 or 0x10 (ordering option) | 0x29 |
| ID | `ID_L` 0x01 (`probe()`) | none; `probe()` checks reserved config bits |
| Mode | active force: power on + trigger → poll `AF_DATA_READY` → read → shut down | power on → wait → read → shut down; or `start_continuous()` with PSM wait 0.4–3.2 s |
| Gain / IT | ×0.5, ×0.66, ×1, ×2; full or ¼ photodiode; 3.125–400 ms | GAIN, DG, SENS (1/8); 25–800 ms |
| Resolution | 2.568 mlx/count (×2, 400 ms) to 5.26 lx/count; 172 klx max | 0.4 mlx/count to 0.41 lx/count; ~6.7 klx max |
| Wait | 4 ms start-up + IT + 10 % | 4 ms start-up + IT + 10 % |
| Shutdown current | 0.5 µA typ. | 0.5 µA typ. |
| Output | milli-lux, raw ALS, raw IR | milli-lux, raw ALS, raw WHITE |
| Interrupt | high/low window (milli-lux → counts), persistence, flags | same |
| Source | doc. 80007 rev. 1.4 (Dec 2025) | doc. 84889 rev. 1.3 (Nov 2024) |
| Status | mock-tested | mock-tested |

The VEML6031X00 resolution follows datasheet rev. 1.4 ("corrected
resolution"): 0.0026 lx/count at ×2 / 400 ms. Earlier revisions (and the
Zephyr driver) use 0.0034 lx/count. `ALS_ON_0` and `ALS_ON_1` are always
written together, with `ALS_CAL` = 1 on power-on as the datasheet requires.
