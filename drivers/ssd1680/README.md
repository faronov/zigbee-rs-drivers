# zigbee-ssd1680

Solomon **SSD1680** / **SSD1681** / **SSD1675** e-paper controllers, used
on 2.13" 122x250, 2.9" 128x296 and 1.54" 200x200 black/white panels and
their black/white/red versions (Waveshare V2/V3/V4, WeAct, Good Display
GDEY/GDEH).

| | |
|---|---|
| Bus | 4-wire SPI mode 0 ≤ 20 MHz write (10 MHz recommended), D/C low = command; BUSY high = busy |
| Refresh | full: OTP `0x22 0xF7` (B/W ~2.5 s, red ~15 s); partial: OTP mode 2 `0x22 0xFF` (~0.4 s) or register LUT `0xC7` |
| Partial | B/W only; window widened to 8-pixel columns; `finish_refresh` keeps the previous-image RAM in sync |
| Sleep | deep sleep mode 1 (`0x10 0x01`), ~1 µA, image stays; wake with `init` |
| Source | SSD1680 v0.14, SSD1681 v0.13, SSD1675A v0.11 datasheets; SSD1675 2.13" LUT/voltages from epd-waveshare (ISC, see NOTICE) |
| Status | **mock-tested, not run on hardware** |

## Not blocking the Zigbee stack

```ignore
let mut epd = Ssd1680::new(SpiInterface::new(spi_dev, dc), busy, Panel::SSD1680_2IN13)
    .with_reset(rst);
epd.init(&mut delay)?;                 // 20 ms reset + short BUSY polls
let wait_us = epd.update(&fb)?;        // write RAM + start full refresh
// schedule a timer for wait_us, return to the stack, then:
if epd.is_busy()? { /* try again later */ }
epd.sleep()?;

// partial update of what changed
if let Some(wait_us) = epd.start_partial_refresh_dirty(&mut fb)? {
    // ...after wait_us and !is_busy(), with the frame untouched:
    epd.finish_refresh(&fb)?;
}
if epd.partial_count() >= 8 { epd.update(&fb)?; } // clean ghosting
```

Every call samples BUSY and returns `Error::NotReady` instead of
waiting. `asynch::Ssd1680` has the same API over `AsyncInterface`, plus
`wait_idle()`, which awaits the BUSY falling edge (`Wait` pin).

Other methods: `write_frame`, `write_tri_frame` / `update_tri`
(tricolour), `start_full_refresh`, `start_partial_refresh(&fb, area)`,
`command(cmd, data)` (raw escape hatch), `with_waveform(Waveform::Custom
{ full, partial })`.

## Panels and RAM

| Preset | Size | Frame | Bytes |
|---|---|---|---|
| `SSD1680_2IN13` / `SSD1675_2IN13` | 122x250 | `MonoFrame` | 4 000 |
| `SSD1680_2IN13_RED` | 122x250 | `TriFrame` | 8 000 |
| `SSD1680_2IN9` | 128x296 | `MonoFrame` | 4 736 |
| `SSD1680_2IN9_RED` | 128x296 | `TriFrame` | 9 472 |
| `SSD1681_1IN54` | 200x200 | `MonoFrame` | 5 000 |
| `SSD1681_1IN54_RED` | 200x200 | `TriFrame` | 10 000 |

Use `MonoFrame::new(...).with_rotation(Rotation::Deg90)` for landscape
drawing; the RAM layout stays native.

## Wiring

| Panel pin | MCU |
|---|---|
| DIN | SPI MOSI |
| CLK | SPI SCK |
| CS | SpiDevice CS |
| DC | GPIO (low = command) |
| RST | GPIO (optional, needed to wake from deep sleep without a power cycle) |
| BUSY | GPIO input (interrupt-capable for `wait_idle`) |

## Caveats

- OTP display mode 2 (partial) exists on most SSD1680/SSD1681 panels but
  not all. If partial updates look wrong, supply a LUT with
  `Waveform::Custom`.
- Do not draw into the frame between `start_partial_refresh` and
  `finish_refresh`.
