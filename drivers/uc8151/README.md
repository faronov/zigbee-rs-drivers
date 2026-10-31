# zigbee-uc8151

UltraChip **UC8151** (UC8151C/D) and Good Display **IL0373** e-paper
controllers, used on 2.9" 128x296 and 2.13" 104x212 black/white panels
(Pimoroni Badger 2040, Waveshare 2.9"/2.13" D) and their black/white/red
versions (Waveshare 2.9"/2.13" B/C, Adafruit tricolour FeatherWings).

| | |
|---|---|
| Bus | 4-wire SPI mode 0, ≤ 10 MHz write (4 MHz is safe on long wires), D/C low = command; **BUSY_N low = busy** |
| Refresh | full: OTP waveform via DRF `0x12` (B/W ~3 s, red ~15 s) |
| Partial | B/W only: PTIN/PTL/PTOUT window (8-pixel columns). Uses the OTP waveform unless you supply register LUTs (`with_partial_luts`); none are bundled |
| Sleep | two-phase: CDI + POF (`Error::Pending`), then DSLP `0x07 0xA5`, ~1 µA, image stays; wake with `init` (RST required) |
| Source | UC8151C v0.3 and IL0373 v0.x datasheets |
| Status | **mock-tested, not run on hardware** |

## Not blocking the Zigbee stack

```ignore
let mut epd = Uc8151::new(SpiInterface::new(spi_dev, dc), busy, Panel::UC8151_2IN9)
    .with_reset(rst);
epd.init(&mut delay)?;                 // 20 ms reset + power-on BUSY polls
let wait_us = epd.update(&fb)?;        // DTM1 + DTM2 + DRF
// schedule a timer for wait_us, return to the stack, later:
if epd.is_busy()? { /* try again later */ }
match epd.sleep() {                    // phase 1: power off
    Err(Error::Pending(us)) => { /* come back after `us`, call sleep() again */ }
    r => r?,
}

if let Some(wait_us) = epd.start_partial_refresh_dirty(&mut fb)? {
    // ...after wait_us and !is_busy(), with the frame untouched:
    epd.finish_refresh(&fb)?;          // DTM1 window + PTOUT
}
```

Every call samples BUSY_N and returns `Error::NotReady` instead of
waiting. `zigbee_display::blocking::sleep` / `asynch::Display::sleep_with`
follow both sleep phases for you. `asynch::Uc8151` has the same API
over `AsyncInterface`, plus `wait_idle()`, which awaits BUSY_N rising.

Other methods: `write_frame`, `write_tri_frame` / `update_tri`,
`start_full_refresh`, `start_partial_refresh(&fb, area)`,
`command(cmd, data)` (raw escape hatch).

## Panels and RAM

| Preset | Size | Frame | Bytes |
|---|---|---|---|
| `UC8151_2IN9` | 128x296 | `MonoFrame` | 4 736 |
| `UC8151_2IN13` | 104x212 | `MonoFrame` | 2 756 |
| `IL0373_2IN9_RED` | 128x296 | `TriFrame` | 9 472 |
| `IL0373_2IN13_RED` | 104x212 | `TriFrame` | 5 512 |

`Panel::new(w, h, tricolor)` accepts other sizes (w multiple of 8, ≤ 160;
h ≤ 296). `TriFrame` stores red as 1 = red; the driver inverts it to
the controller's 0 = red on the fly (64-byte stack buffer).

## Wiring

| Panel pin | MCU |
|---|---|
| DIN | SPI MOSI |
| CLK | SPI SCK |
| CS | SpiDevice CS |
| DC | GPIO (low = command) |
| RST | GPIO (needed to wake from deep sleep) |
| BUSY | GPIO input, low while busy (interrupt-capable for `wait_idle`) |

## Caveats

- Without custom LUTs a partial refresh flashes the window and takes
  about as long as a full refresh; it only limits what changes.
- PTL "scan" byte is `0x01` (scan inside the window only). Some panels
  copy new→old RAM automatically after refresh; `finish_refresh` writes
  DTM1 explicitly, which is harmless if they do.
- Do not draw into the frame between `start_partial_refresh` and
  `finish_refresh`.
