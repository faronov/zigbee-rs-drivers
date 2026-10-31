# zigbee-st7789

Sitronix **ST7789** (240x240, 240x320, 135x240, 170x320 IPS) and
**ST7735** (128x160, 128x128, 80x160) RGB565 TFT controllers over
4-wire SPI.

> **Not for battery devices.** A TFT and its backlight draw tens of mA
> whenever they are on. Use this on mains-powered Zigbee routers and
> coordinators (status screens, gateways), not on sleepy end devices.
> For battery devices use e-paper (`zigbee-ssd1680`, `zigbee-uc8151`,
> `zigbee-il3820`) or a Sharp Memory LCD (`zigbee-sharp-memlcd`).

| | |
|---|---|
| Bus | 4-wire SPI, D/C low = command. ST7789 write cycle ≥ 16 ns (62.5 MHz max), ST7735 ≥ 66 ns (15 MHz). Many headerless ST7789 modules have no CS and need **SPI mode 3** |
| Format | RGB565, 2 bytes/pixel, big-endian (COLMOD `0x55` ST7789 / `0x05` ST7735) |
| Refresh | none: writes are visible immediately; `start_refresh` returns 0, `is_busy` is always false |
| Sleep | `sleep()` = DISPOFF + SLPIN (RAM kept); `wake(delay)` = SLPOUT + 120 ms + DISPON. The backlight is a separate GPIO/PWM |
| Source | ST7789V v1.0 and ST7735S v1.1 datasheets; rotation offsets cross-checked with mipidsi (MIT/Apache) |
| Status | **mock-tested, not run on hardware** |

## No frame buffer needed

The controller holds the image, so the driver works with windows:

```ignore
let mut tft = St7789::new(SpiInterface::new(spi_dev, dc), Panel::ST7789_240X240)
    .with_reset(rst)
    .with_rotation(Rotation::Deg90);
tft.init(&mut delay)?;                         // ~280 ms, blocking
tft.clear_screen(Rgb565::BLACK)?;              // streamed from 64 B on the stack
tft.fill_area(Area::new(0, 0, 240, 24), Rgb565::BLUE)?;

// Draw text into a small tile and push only what changed.
let mut buf = [0u8; rgb565_len(96, 24)];       // 4.6 KB
let mut tile = Rgb565Frame::new(&mut buf[..], 96, 24).unwrap();
Text::new("21.5 °C", Point::new(0, 16), style).draw(&mut tile)?;
tft.write_dirty_at(10, 40, &mut tile)?;        // CASET/RASET/RAMWR of the dirty rect

// Or draw straight to the panel (blocking driver implements DrawTarget):
Rectangle::new(Point::new(10, 80), Size::new(50, 20))
    .into_styled(PrimitiveStyle::with_fill(Rgb565::RED))
    .draw(&mut tft)?;                          // one window, solid fill
```

`DrawTarget` on the driver: `fill_solid`/`clear` and in-bounds
`fill_contiguous` are one window each; `draw_iter` (lines, text drawn
directly) costs one window (~11 bytes of commands) per pixel, so render
text into a tile instead.

`asynch::St7789` has the same API over `AsyncInterface` (no
`DrawTarget`, since embedded-graphics is synchronous: draw into a tile
and use `write_frame_at` / `write_dirty_at`).

Other methods: `set_window` + `write_pixels_continue` (stream your own
pixels), `write_pixels(area, bytes)`, `set_rotation`, `set_invert`,
`set_display_on`, `command(cmd, data)` for gamma/power/frame-rate
tuning.

## Panels and RAM

The driver needs only a 64-byte stack buffer. A tile costs
`w * h * 2` bytes. For reference, a full-screen frame would be:

| Preset | Size | Offset (Deg0) | Inversion / order | Full frame |
|---|---|---|---|---|
| `ST7789_240X240` | 240x240 | 0, 0 | on / RGB | 115 200 |
| `ST7789_240X320` | 240x320 | 0, 0 | on / RGB | 153 600 |
| `ST7789_135X240` | 135x240 | 52, 40 | on / RGB | 64 800 |
| `ST7789_170X320` | 170x320 | 35, 0 | on / RGB | 108 800 |
| `ST7735_128X160` | 128x160 | 0, 0 | off / RGB | 40 960 |
| `ST7735_128X128` | 128x128 | 2, 1 | off / RGB | 32 768 |
| `ST7735_80X160` | 80x160 | 26, 1 | on / BGR | 25 600 |

Offsets for the other rotations are derived from the controller RAM size
(240x320 / 132x162). Modules differ: adjust with
`Panel::with_offset(x, y)`, `with_invert`, `with_bgr`.

## Wiring

| Module pin | MCU |
|---|---|
| SDA / DIN | SPI MOSI |
| SCL / CLK | SPI SCK |
| CS | SpiDevice CS (if absent: SPI mode 3) |
| DC / RS | GPIO (low = command) |
| RES / RST | GPIO (optional; SWRESET is always sent) |
| BLK / BL | GPIO or PWM for the backlight (not managed by the driver) |

## Caveats

- ST7735 uses its power-on defaults for frame rate, power and gamma; no
  panel-specific tables are bundled. Send them with `command()` if
  colours look off.
- The `ST7735_128X128` offsets depend on the module ("green tab" variants
  use 2,1 or 2,3).
