# zigbee-display

The pieces all zigbee-rs display drivers share. Firmware that only uses
sensors never depends on this crate, so it never pulls in embedded-graphics.

| Item | Purpose |
|---|---|
| `blocking::Display` / `asynch::Display` | `init`, `start_refresh() -> WaitUs`, `is_busy`, `sleep` |
| `blocking::{refresh, wait_idle, sleep}` | blocking BUSY-polling helpers (tools, bring-up) |
| `SpiInterface` | 4-wire SPI: `SpiDevice` + D/C pin (low = command) |
| `I2cInterface` | SSD1306/SH1106 I2C (control byte `0x00` cmd / `0x40` data, 32-byte chunks) |
| `MonoFrame`, `TriFrame`, `PageFrame`, `Rgb565Frame` | caller-owned frame buffers implementing `embedded_graphics_core::draw_target::DrawTarget`, with software rotation and dirty-rectangle tracking |
| `NoPin` | placeholder for an unconnected RST / BUSY / DISP pin |
| `DisplayError<B>` | `zigbee_sensor::Error<BusError<B>>` |

## Non-blocking refresh

E-paper controllers drive BUSY high (SSD16xx, IL38xx) or low (UC8151) for
the whole 0.3–20 s waveform. No driver waits for that. Instead:

```ignore
epd.write_frame(&frame)?;              // SPI transfer, a few ms
let wait_us = epd.start_full_refresh()?; // returns typical duration
// ... schedule a timer for wait_us and return to the Zigbee stack ...
if !epd.is_busy()? { epd.sleep()?; }   // BUSY is authoritative
```

The async drivers also have `wait_idle()`, which awaits the BUSY edge when
the pin implements `embedded_hal_async::digital::Wait`. Every operation
that sends commands first checks BUSY and returns `Error::NotReady` while
the panel is still refreshing.

LCDs, OLEDs and TFTs update as soon as RAM is written:
`start_refresh()` returns `0` and `is_busy()` is `false`.

## RAM

Buffers are supplied by the caller (`[u8; N]` or `&'static mut [u8]`), so
the cost is explicit:

| Panel | Frame | Bytes |
|---|---|---|
| 128x32 OLED | `PageFrame` | 512 |
| 128x64 OLED | `PageFrame` | 1 024 |
| 128x128 Sharp | `MonoFrame` | 2 048 |
| 122x250 e-paper (2.13") | `MonoFrame` / `TriFrame` | 4 000 / 8 000 |
| 128x296 e-paper (2.9") | `MonoFrame` / `TriFrame` | 4 736 / 9 472 |
| 200x200 e-paper (1.54") | `MonoFrame` / `TriFrame` | 5 000 / 10 000 |
| 400x240 Sharp | `MonoFrame` | 12 000 |
| 240x240 TFT | none required (windowed writes) | 0, or a tile |

## Status

mock-tested, not run on hardware.
