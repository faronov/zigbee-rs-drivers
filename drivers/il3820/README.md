# zigbee-il3820

Good Display / Waveshare **IL3820** (2.9" 128x296, Waveshare 2.9" V1),
**IL3895** (2.13" 122x250, Waveshare 2.13" V1) and **IL3829** (1.54"
200x200 V1) black/white e-paper controllers. This is the older "type A"
generation; newer modules with the same sizes use SSD1680/SSD1675 (see
`zigbee-ssd1680`).

| | |
|---|---|
| Bus | 4-wire SPI mode 0 ≤ 4 MHz write, D/C low = command; BUSY high = busy |
| Refresh | register LUT only (no OTP waveform): full ~2 s, partial ~0.4 s |
| Partial | window widened to 8-pixel columns; `finish_refresh` keeps the second internal buffer in sync |
| Sleep | deep sleep `0x10 0x01`; image stays; wake with `init` (RST) |
| Source | IL3820 / IL3895 datasheets; full/partial LUTs and booster/VCOM values from epd-waveshare `type_a` (ISC, see NOTICE) |
| Status | **mock-tested, not run on hardware** |

## Not blocking the Zigbee stack

```ignore
let mut epd = Il3820::new(SpiInterface::new(spi_dev, dc), busy, Panel::IL3820_2IN9)
    .with_reset(rst);
epd.init(&mut delay)?;                 // 20 ms reset + short BUSY polls + full LUT
let wait_us = epd.update(&fb)?;        // write RAM + full refresh
// schedule a timer for wait_us, return to the stack, later:
if epd.is_busy()? { /* try again later */ }
epd.finish_refresh(&fb)?;              // REQUIRED after every refresh, see below

if let Some(wait_us) = epd.start_partial_refresh_dirty(&mut fb)? {
    // ...after wait_us and !is_busy(), with the frame untouched:
    epd.finish_refresh(&fb)?;
}
if epd.partial_count() >= 8 { epd.update(&fb)?; } // clean ghosting
epd.sleep()?;
```

The controller has two frame buffers behind a single write command and
swaps them after each refresh (this is why Waveshare's V1 examples
write every frame twice). `finish_refresh` writes the last refreshed
window again so the next partial refresh compares against the right
image. `needs_finish()` tells whether that is still outstanding.

Every call samples BUSY and returns `Error::NotReady` instead of
waiting. `asynch::Il3820` has the same API over `AsyncInterface`, plus
`wait_idle()`, which awaits BUSY low.

Other methods: `write_frame`, `start_full_refresh`,
`start_partial_refresh(&fb, area)`, `with_luts(Luts { full, partial })`,
`with_update_control(0xC7)`, `command(cmd, data)`.

## Panels and RAM

| Preset | Size | Frame | Bytes |
|---|---|---|---|
| `IL3820_2IN9` | 128x296 | `MonoFrame` | 4 736 |
| `IL3895_2IN13` | 122x250 | `MonoFrame` | 4 000 |
| `IL3829_1IN54` | 200x200 | `MonoFrame` | 5 000 |

## Wiring

| Panel pin | MCU |
|---|---|
| DIN | SPI MOSI |
| CLK | SPI SCK |
| CS | SpiDevice CS |
| DC | GPIO (low = command) |
| RST | GPIO (needed to wake from deep sleep) |
| BUSY | GPIO input, high while busy |

## Caveats

- The bundled LUTs are the Waveshare 2.9"/1.54" V1 waveforms. Waveshare
  ships a different waveform for the 2.13" V1 (IL3895); if the 2.13"
  image is weak or ghosts, pass that panel's LUT via `with_luts`.
- Update control `0xC4` (Waveshare) leaves the analog block on after the
  update; the datasheet sequence `0xC7` also turns it off (lower idle
  current). Neither has been verified on hardware.
