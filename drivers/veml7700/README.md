# zigbee-veml7700

Vishay VEML7700 and VEML6030 ambient light sensors (0–140 klx, 16-bit,
human-eye response, plus a WHITE channel) over I2C.

| | |
|---|---|
| Address | 0x10 (VEML7700; VEML6030 ADDR low), 0x48 (VEML6030 ADDR high) |
| ID | register 0x07 low byte 0x81 (`probe()`); high byte 0xC4/0xD4 by address option |
| Mode | one-shot by default: power on → integrate → read ALS/WHITE → shut down; `start_continuous()` with optional power-saving mode 1–4 |
| Conversion | IT 25–800 ms; driver waits 4 ms start-up + IT + 10 % |
| Shutdown current | 0.5 µA typ. |
| Gain / IT | ×1/8, ×1/4, ×1, ×2 and 25–800 ms; resolution 4.2 mlx/count (×2, 800 ms) to 2.15 lx/count; `auto_range()` follows the app-note ladder |
| Output | milli-lux, raw ALS and WHITE counts |
| Correction | app-note polynomial `6.0135e-13x⁴ − 9.3924e-9x³ + 8.1488e-5x² + 1.0023x` at gain ×1/4 and ×1/8, integer Horner form (uses `i128`); `Config::no_correction` disables it |
| Interrupt | high/low window (milli-lux → counts), persistence 1–8, flags in register 0x06 (VEML6030 also drives INT) |
| Source | Vishay doc. 84286 rev. 1.8 (VEML7700), 84366 (VEML6030), app notes 84323 / 84367 |
| Status | mock-tested |

There is no data-ready flag: read after the returned wait. The part keeps
the last result while shut down. Gains ×1 and ×2 are non-linear above
about 100 lx; `auto_range()` steps away from them.
