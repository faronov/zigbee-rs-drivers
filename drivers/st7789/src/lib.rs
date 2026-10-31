//! Sitronix **ST7789** (240x240, 240x320, 135x240, 170x320 IPS) and
//! **ST7735** (128x160, 128x128, 80x160) RGB565 TFT controllers over
//! 4-wire SPI.
//!
//! **Not for battery-powered Zigbee end devices**: the panel and its
//! backlight draw tens of mA while on. Use it on mains-powered routers
//! and coordinators, or switch it fully off (backlight GPIO + `sleep`).
//!
//! The controller keeps the whole image in its own RAM, so the MCU does
//! not need a full frame buffer (240x320x2 = 150 KB would not fit). Write
//! into windows instead:
//!
//! * [`St7789::fill_area`] — solid rectangle, streamed from a 64-byte
//!   stack buffer;
//! * [`St7789::write_pixels`] — raw big-endian RGB565 bytes into a window;
//! * [`St7789::write_frame_at`] / [`St7789::write_dirty_at`] — a small
//!   caller-owned [`Rgb565Frame`] tile (e.g. 64x32 = 4 KB) drawn with
//!   embedded-graphics, written as a whole or just its dirty rectangle;
//! * the blocking driver also implements embedded-graphics
//!   [`DrawTarget<Color = Rgb565>`](embedded_graphics_core::draw_target::DrawTarget)
//!   directly (fills and
//!   contiguous fills are windowed; single pixels cost one window each).
//!
//! Writes show up immediately: there is no refresh step, `start_refresh`
//! returns 0 and `is_busy` is always `false`.
//!
//! Orientation is done in hardware (MADCTL) with offsets recomputed per
//! rotation, as in the ST7789V/ST7735S datasheets' memory maps (RAM 240x320
//! and 132x162).
//!
//! References: Sitronix ST7789V v1.0 and ST7735S v1.1 datasheets.

#![no_std]

use embedded_graphics_core::pixelcolor::Rgb565;
use embedded_graphics_core::pixelcolor::raw::{RawData, RawU16};
use embedded_hal::digital::OutputPin;
use zigbee_display::{Area, DisplayError, Error, NoPin, Rgb565Frame, Rotation, WaitUs, pin_error};

pub use zigbee_display;
pub use zigbee_display::SpiInterface;

/// Command bytes (MIPI DCS subset).
pub mod cmd {
    /// Software reset.
    pub const SWRESET: u8 = 0x01;
    /// Sleep in.
    pub const SLPIN: u8 = 0x10;
    /// Sleep out.
    pub const SLPOUT: u8 = 0x11;
    /// Normal display mode on.
    pub const NORON: u8 = 0x13;
    /// Inversion off.
    pub const INVOFF: u8 = 0x20;
    /// Inversion on.
    pub const INVON: u8 = 0x21;
    /// Display off.
    pub const DISPOFF: u8 = 0x28;
    /// Display on.
    pub const DISPON: u8 = 0x29;
    /// Column address set.
    pub const CASET: u8 = 0x2A;
    /// Row address set.
    pub const RASET: u8 = 0x2B;
    /// Memory write.
    pub const RAMWR: u8 = 0x2C;
    /// Memory data access control.
    pub const MADCTL: u8 = 0x36;
    /// Interface pixel format.
    pub const COLMOD: u8 = 0x3A;
}

/// MADCTL bits.
pub mod madctl {
    /// Row address order (mirror Y).
    pub const MY: u8 = 0x80;
    /// Column address order (mirror X).
    pub const MX: u8 = 0x40;
    /// Row/column exchange.
    pub const MV: u8 = 0x20;
    /// BGR colour order.
    pub const BGR: u8 = 0x08;
}

/// Wait after a hardware or software reset.
pub const RESET_WAIT_US: u32 = 150_000;
/// Wait after SLPOUT before the next command (datasheet: 120 ms).
pub const SLEEP_OUT_US: u32 = 120_000;

/// Controller variant.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Controller {
    /// ST7789 / ST7789V (RAM 240x320, COLMOD `0x55`).
    St7789,
    /// ST7735 / ST7735S (RAM 132x162, COLMOD `0x05`).
    St7735,
}

impl Controller {
    /// Controller RAM size (columns, rows).
    pub const fn ram_size(self) -> (u16, u16) {
        match self {
            Controller::St7789 => (240, 320),
            Controller::St7735 => (132, 162),
        }
    }

    const fn colmod(self) -> u8 {
        match self {
            Controller::St7789 => 0x55,
            Controller::St7735 => 0x05,
        }
    }
}

/// Panel description in its native (portrait, `Rotation::Deg0`) orientation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Panel {
    controller: Controller,
    width: u16,
    height: u16,
    x_offset: u16,
    y_offset: u16,
    invert: bool,
    bgr: bool,
}

impl Panel {
    /// ST7789 1.3"/1.54" 240x240 IPS.
    pub const ST7789_240X240: Panel = Panel::raw(Controller::St7789, 240, 240, 0, 0, true, false);
    /// ST7789 2.0"/2.4"/2.8" 240x320 IPS.
    pub const ST7789_240X320: Panel = Panel::raw(Controller::St7789, 240, 320, 0, 0, true, false);
    /// ST7789 1.14" 135x240 IPS (TTGO T-Display, Waveshare 1.14").
    pub const ST7789_135X240: Panel = Panel::raw(Controller::St7789, 135, 240, 52, 40, true, false);
    /// ST7789 1.9" 170x320 IPS.
    pub const ST7789_170X320: Panel = Panel::raw(Controller::St7789, 170, 320, 35, 0, true, false);
    /// ST7735 1.8" 128x160 TN.
    pub const ST7735_128X160: Panel = Panel::raw(Controller::St7735, 128, 160, 0, 0, false, false);
    /// ST7735 1.44" 128x128 TN (offsets vary between modules).
    pub const ST7735_128X128: Panel = Panel::raw(Controller::St7735, 128, 128, 2, 1, false, false);
    /// ST7735S 0.96" 80x160 IPS.
    pub const ST7735_80X160: Panel = Panel::raw(Controller::St7735, 80, 160, 26, 1, true, true);

    #[allow(clippy::too_many_arguments)]
    const fn raw(
        controller: Controller,
        width: u16,
        height: u16,
        x_offset: u16,
        y_offset: u16,
        invert: bool,
        bgr: bool,
    ) -> Self {
        Self {
            controller,
            width,
            height,
            x_offset,
            y_offset,
            invert,
            bgr,
        }
    }

    /// Custom panel without offset (non-inverted, RGB). `None` if it
    /// does not fit the controller RAM.
    pub const fn new(controller: Controller, width: u16, height: u16) -> Option<Self> {
        Self::raw(controller, width, height, 0, 0, false, false).with_offset(0, 0)
    }

    /// Set the RAM offset of the visible area (native orientation).
    /// `None` if the panel would not fit the controller RAM.
    pub const fn with_offset(mut self, x: u16, y: u16) -> Option<Self> {
        let (rw, rh) = self.controller.ram_size();
        if self.width == 0
            || self.height == 0
            || x as u32 + self.width as u32 > rw as u32
            || y as u32 + self.height as u32 > rh as u32
        {
            return None;
        }
        self.x_offset = x;
        self.y_offset = y;
        Some(self)
    }

    /// Colour inversion (most IPS panels need it on).
    pub const fn with_invert(mut self, invert: bool) -> Self {
        self.invert = invert;
        self
    }

    /// BGR instead of RGB subpixel order.
    pub const fn with_bgr(mut self, bgr: bool) -> Self {
        self.bgr = bgr;
        self
    }

    /// Controller variant.
    pub const fn controller(&self) -> Controller {
        self.controller
    }

    /// Native width.
    pub const fn width(&self) -> u16 {
        self.width
    }

    /// Native height.
    pub const fn height(&self) -> u16 {
        self.height
    }

    /// Bytes of a full-screen [`Rgb565Frame`] (for reference: the driver
    /// does not need one).
    pub const fn frame_len(&self) -> usize {
        zigbee_display::rgb565_len(self.width, self.height)
    }

    /// Logical size for an orientation.
    pub const fn size(&self, rotation: Rotation) -> (u16, u16) {
        match rotation {
            Rotation::Deg0 | Rotation::Deg180 => (self.width, self.height),
            Rotation::Deg90 | Rotation::Deg270 => (self.height, self.width),
        }
    }

    /// MADCTL value for an orientation.
    pub const fn madctl(&self, rotation: Rotation) -> u8 {
        let m = match rotation {
            Rotation::Deg0 => 0,
            Rotation::Deg90 => madctl::MX | madctl::MV,
            Rotation::Deg180 => madctl::MX | madctl::MY,
            Rotation::Deg270 => madctl::MY | madctl::MV,
        };
        if self.bgr { m | madctl::BGR } else { m }
    }

    /// Column/row address offsets for an orientation.
    pub const fn offset(&self, rotation: Rotation) -> (u16, u16) {
        let (rw, rh) = self.controller.ram_size();
        let mx = matches!(rotation, Rotation::Deg90 | Rotation::Deg180);
        let my = matches!(rotation, Rotation::Deg180 | Rotation::Deg270);
        let x = if mx {
            rw - self.width - self.x_offset
        } else {
            self.x_offset
        };
        let y = if my {
            rh - self.height - self.y_offset
        } else {
            self.y_offset
        };
        match rotation {
            Rotation::Deg0 | Rotation::Deg180 => (x, y),
            Rotation::Deg90 | Rotation::Deg270 => (y, x),
        }
    }
}

const CHUNK: usize = 64;

fn color_bytes(color: Rgb565) -> [u8; 2] {
    RawU16::from(color).into_inner().to_be_bytes()
}

fn addr(start: u16, end: u16) -> [u8; 4] {
    let [s0, s1] = start.to_be_bytes();
    let [e0, e1] = end.to_be_bytes();
    [s0, s1, e0, e1]
}

macro_rules! driver {
    ($Iface:path, $Delay:path, $($async:tt $await:tt)?) => {
        /// ST7789 / ST7735 driver; RST is optional (SWRESET is always sent).
        pub struct St7789<DI, RST = NoPin> {
            di: DI,
            rst: Option<RST>,
            panel: Panel,
            rotation: Rotation,
        }

        impl<DI: $Iface> St7789<DI> {
            /// Driver without a reset pin, `Rotation::Deg0`.
            pub fn new(di: DI, panel: Panel) -> Self {
                Self {
                    di,
                    rst: None,
                    panel,
                    rotation: Rotation::Deg0,
                }
            }
        }

        impl<DI: $Iface, RST: OutputPin> St7789<DI, RST> {
            /// Add the RST pin, pulsed by [`Self::init`].
            pub fn with_reset<P: OutputPin>(self, rst: P) -> St7789<DI, P> {
                St7789 {
                    di: self.di,
                    rst: Some(rst),
                    panel: self.panel,
                    rotation: self.rotation,
                }
            }

            /// Initial orientation (applied by [`Self::init`]).
            pub fn with_rotation(mut self, rotation: Rotation) -> Self {
                self.rotation = rotation;
                self
            }

            /// Panel description.
            pub const fn panel(&self) -> Panel {
                self.panel
            }

            /// Current orientation.
            pub const fn rotation(&self) -> Rotation {
                self.rotation
            }

            /// Logical size in the current orientation.
            pub const fn size(&self) -> (u16, u16) {
                self.panel.size(self.rotation)
            }

            /// Give the peripherals back.
            pub fn release(self) -> (DI, Option<RST>) {
                (self.di, self.rst)
            }

            $($async)? fn cmd(&mut self, command: u8, data: &[u8]) -> Result<(), DisplayError<DI::Error>> {
                self.di.command(&[command]) $(.$await)?.map_err(Error::Bus)?;
                self.di.data(data) $(.$await)?.map_err(Error::Bus)
            }

            /// Send a raw command with parameters (gamma, power, frame rate,
            /// backlight PWM on controllers that have it).
            pub $($async)? fn command(&mut self, command: u8, data: &[u8]) -> Result<(), DisplayError<DI::Error>> {
                self.cmd(command, data) $(.$await)?
            }

            /// Reset, sleep out, RGB565, orientation, inversion, display on.
            /// Blocks for ~280 ms (reset 150 ms + sleep out 120 ms + 10 ms);
            /// contents of the RAM are undefined until written.
            pub $($async)? fn init<D: $Delay>(&mut self, delay: &mut D) -> Result<(), DisplayError<DI::Error>> {
                if let Some(rst) = self.rst.as_mut() {
                    rst.set_low().map_err(pin_error)?;
                    delay.delay_us(20) $(.$await)?;
                    rst.set_high().map_err(pin_error)?;
                    delay.delay_us(RESET_WAIT_US) $(.$await)?;
                }
                self.cmd(cmd::SWRESET, &[]) $(.$await)??;
                delay.delay_us(RESET_WAIT_US) $(.$await)?;
                self.cmd(cmd::SLPOUT, &[]) $(.$await)??;
                delay.delay_us(SLEEP_OUT_US) $(.$await)?;
                let p = self.panel;
                self.cmd(cmd::COLMOD, &[p.controller.colmod()]) $(.$await)??;
                self.cmd(cmd::MADCTL, &[p.madctl(self.rotation)]) $(.$await)??;
                self.cmd(if p.invert { cmd::INVON } else { cmd::INVOFF }, &[]) $(.$await)??;
                self.cmd(cmd::NORON, &[]) $(.$await)??;
                delay.delay_us(10_000) $(.$await)?;
                self.cmd(cmd::DISPON, &[]) $(.$await)?
            }

            /// Change the orientation (MADCTL) at run time. Existing RAM
            /// content is not rotated.
            pub $($async)? fn set_rotation(&mut self, rotation: Rotation) -> Result<(), DisplayError<DI::Error>> {
                self.rotation = rotation;
                let m = self.panel.madctl(rotation);
                self.cmd(cmd::MADCTL, &[m]) $(.$await)?
            }

            /// Colour inversion on/off.
            pub $($async)? fn set_invert(&mut self, invert: bool) -> Result<(), DisplayError<DI::Error>> {
                self.cmd(if invert { cmd::INVON } else { cmd::INVOFF }, &[]) $(.$await)?
            }

            /// Display output on/off (RAM kept; does not touch the backlight).
            pub $($async)? fn set_display_on(&mut self, on: bool) -> Result<(), DisplayError<DI::Error>> {
                self.cmd(if on { cmd::DISPON } else { cmd::DISPOFF }, &[]) $(.$await)?
            }

            /// Display off + sleep in (RAM kept). Switch the backlight off
            /// separately. Wait ≥ 5 ms before [`Self::wake`].
            pub $($async)? fn sleep(&mut self) -> Result<(), DisplayError<DI::Error>> {
                self.cmd(cmd::DISPOFF, &[]) $(.$await)??;
                self.cmd(cmd::SLPIN, &[]) $(.$await)?
            }

            /// Sleep out (blocks 120 ms) + display on.
            pub $($async)? fn wake<D: $Delay>(&mut self, delay: &mut D) -> Result<(), DisplayError<DI::Error>> {
                self.cmd(cmd::SLPOUT, &[]) $(.$await)??;
                delay.delay_us(SLEEP_OUT_US) $(.$await)?;
                self.cmd(cmd::DISPON, &[]) $(.$await)?
            }

            /// Open a RAM write window (logical coordinates, current
            /// orientation). Follow with exactly `area.width * area.height`
            /// pixels via [`Self::write_pixels_continue`].
            pub $($async)? fn set_window(&mut self, area: Area) -> Result<(), DisplayError<DI::Error>> {
                let (w, h) = self.size();
                if area.is_empty() || area.right() > w || area.bottom() > h {
                    return Err(Error::InvalidArgument);
                }
                let (ox, oy) = self.panel.offset(self.rotation);
                self.cmd(cmd::CASET, &addr(area.x + ox, area.right() - 1 + ox)) $(.$await)??;
                self.cmd(cmd::RASET, &addr(area.y + oy, area.bottom() - 1 + oy)) $(.$await)??;
                self.cmd(cmd::RAMWR, &[]) $(.$await)?
            }

            /// Stream more big-endian RGB565 bytes into the open window.
            pub $($async)? fn write_pixels_continue(&mut self, bytes: &[u8]) -> Result<(), DisplayError<DI::Error>> {
                self.di.data(bytes) $(.$await)?.map_err(Error::Bus)
            }

            /// Write `area.width * area.height * 2` big-endian RGB565 bytes
            /// into a window.
            pub $($async)? fn write_pixels(&mut self, area: Area, bytes: &[u8]) -> Result<(), DisplayError<DI::Error>> {
                if bytes.len() != usize::from(area.width) * usize::from(area.height) * 2 {
                    return Err(Error::InvalidArgument);
                }
                self.set_window(area) $(.$await)??;
                self.write_pixels_continue(bytes) $(.$await)?
            }

            /// Fill a rectangle with one colour (64-byte stack buffer).
            pub $($async)? fn fill_area(&mut self, area: Area, color: Rgb565) -> Result<(), DisplayError<DI::Error>> {
                self.set_window(area) $(.$await)??;
                let [hi, lo] = color_bytes(color);
                let mut chunk = [0u8; CHUNK];
                for px in chunk.chunks_exact_mut(2) {
                    px[0] = hi;
                    px[1] = lo;
                }
                let mut left = usize::from(area.width) * usize::from(area.height) * 2;
                while left > 0 {
                    let n = left.min(CHUNK);
                    self.di.data(&chunk[..n]) $(.$await)?.map_err(Error::Bus)?;
                    left -= n;
                }
                Ok(())
            }

            /// Fill the whole screen.
            pub $($async)? fn clear_screen(&mut self, color: Rgb565) -> Result<(), DisplayError<DI::Error>> {
                let (w, h) = self.size();
                self.fill_area(Area::full(w, h), color) $(.$await)?
            }

            /// Write a whole tile with its top-left corner at (`x`, `y`).
            /// The tile's native layout is used (tile rotation is a
            /// drawing aid only).
            pub $($async)? fn write_frame_at<B: AsRef<[u8]>>(&mut self, x: u16, y: u16, tile: &Rgb565Frame<B>) -> Result<(), DisplayError<DI::Error>> {
                let area = Area::new(x, y, tile.width(), tile.height());
                self.write_pixels(area, tile.as_bytes()) $(.$await)?
            }

            /// Write only the tile's dirty rectangle (tile at (`x`, `y`)),
            /// then clear it. Returns `false` if nothing was dirty.
            pub $($async)? fn write_dirty_at<B: AsRef<[u8]> + AsMut<[u8]>>(&mut self, x: u16, y: u16, tile: &mut Rgb565Frame<B>) -> Result<bool, DisplayError<DI::Error>> {
                let Some(d) = tile.dirty() else {
                    return Ok(false);
                };
                self.set_window(Area::new(x + d.x, y + d.y, d.width, d.height)) $(.$await)??;
                let stride = usize::from(tile.width()) * 2;
                let (x0, x1) = (usize::from(d.x) * 2, usize::from(d.right()) * 2);
                if d.x == 0 && d.width == tile.width() {
                    let rows = &tile.as_bytes()[usize::from(d.y) * stride..usize::from(d.bottom()) * stride];
                    self.di.data(rows) $(.$await)?.map_err(Error::Bus)?;
                } else {
                    for row in d.y..d.bottom() {
                        let start = usize::from(row) * stride;
                        let bytes = &tile.as_bytes()[start + x0..start + x1];
                        self.di.data(bytes) $(.$await)?.map_err(Error::Bus)?;
                    }
                }
                tile.clear_dirty();
                Ok(true)
            }
        }
    };
}

driver!(zigbee_display::Interface, embedded_hal::delay::DelayNs,);

impl<DI, RST> zigbee_display::blocking::Display for St7789<DI, RST>
where
    DI: zigbee_display::Interface,
    RST: OutputPin,
{
    type BusError = DI::Error;

    fn init<D: embedded_hal::delay::DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<(), DisplayError<DI::Error>> {
        St7789::init(self, delay)
    }

    /// Writes are visible immediately; nothing to wait for.
    fn start_refresh(&mut self) -> Result<WaitUs, DisplayError<DI::Error>> {
        Ok(0)
    }

    fn is_busy(&mut self) -> Result<bool, DisplayError<DI::Error>> {
        Ok(false)
    }

    fn sleep(&mut self) -> Result<(), DisplayError<DI::Error>> {
        St7789::sleep(self)
    }
}

mod graphics {
    use super::*;
    use embedded_graphics_core::Pixel;
    use embedded_graphics_core::draw_target::DrawTarget;
    use embedded_graphics_core::geometry::{Dimensions, OriginDimensions, Size};
    use embedded_graphics_core::primitives::{PointsIter, Rectangle};

    impl<DI: zigbee_display::Interface, RST: OutputPin> OriginDimensions for St7789<DI, RST> {
        fn size(&self) -> Size {
            let (w, h) = St7789::size(self);
            Size::new(u32::from(w), u32::from(h))
        }
    }

    fn to_area(r: &Rectangle) -> Option<Area> {
        let br = r.bottom_right()?;
        Some(Area::new(
            u16::try_from(r.top_left.x).ok()?,
            u16::try_from(r.top_left.y).ok()?,
            u16::try_from(br.x - r.top_left.x + 1).ok()?,
            u16::try_from(br.y - r.top_left.y + 1).ok()?,
        ))
    }

    impl<DI: zigbee_display::Interface, RST: OutputPin> DrawTarget for St7789<DI, RST> {
        type Color = Rgb565;
        type Error = DisplayError<DI::Error>;

        fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
        where
            I: IntoIterator<Item = Pixel<Rgb565>>,
        {
            let bounds = self.bounding_box();
            for Pixel(p, c) in pixels {
                if bounds.contains(p) {
                    let area = Area::new(p.x as u16, p.y as u16, 1, 1);
                    self.write_pixels(area, &color_bytes(c))?;
                }
            }
            Ok(())
        }

        fn fill_contiguous<I>(&mut self, area: &Rectangle, colors: I) -> Result<(), Self::Error>
        where
            I: IntoIterator<Item = Rgb565>,
        {
            let clipped = area.intersection(&self.bounding_box());
            if clipped != *area {
                // Partly off-screen: fall back to per-pixel clipping.
                return self.draw_iter(area.points().zip(colors).map(|(p, c)| Pixel(p, c)));
            }
            let Some(a) = to_area(area) else {
                return Ok(());
            };
            self.set_window(a)?;
            let mut chunk = [0u8; CHUNK];
            let mut n = 0;
            for c in colors
                .into_iter()
                .take(usize::from(a.width) * usize::from(a.height))
            {
                chunk[n..n + 2].copy_from_slice(&color_bytes(c));
                n += 2;
                if n == CHUNK {
                    self.write_pixels_continue(&chunk)?;
                    n = 0;
                }
            }
            if n > 0 {
                self.write_pixels_continue(&chunk[..n])?;
            }
            Ok(())
        }

        fn fill_solid(&mut self, area: &Rectangle, color: Rgb565) -> Result<(), Self::Error> {
            match to_area(&area.intersection(&self.bounding_box())) {
                Some(a) => self.fill_area(a, color),
                None => Ok(()),
            }
        }

        fn clear(&mut self, color: Rgb565) -> Result<(), Self::Error> {
            self.clear_screen(color)
        }
    }
}

/// Async driver over [`zigbee_display::AsyncInterface`]. Draw into an
/// [`Rgb565Frame`] tile and push it with `write_frame_at` /
/// `write_dirty_at` (embedded-graphics `DrawTarget` is synchronous).
pub mod asynch {
    use super::*;

    driver!(zigbee_display::AsyncInterface, embedded_hal_async::delay::DelayNs, async await);

    impl<DI, RST> zigbee_display::asynch::Display for St7789<DI, RST>
    where
        DI: zigbee_display::AsyncInterface,
        RST: OutputPin,
    {
        type BusError = DI::Error;

        async fn init<D: embedded_hal_async::delay::DelayNs>(
            &mut self,
            delay: &mut D,
        ) -> Result<(), DisplayError<DI::Error>> {
            St7789::init(self, delay).await
        }

        async fn start_refresh(&mut self) -> Result<WaitUs, DisplayError<DI::Error>> {
            Ok(0)
        }

        async fn is_busy(&mut self) -> Result<bool, DisplayError<DI::Error>> {
            Ok(false)
        }

        async fn sleep(&mut self) -> Result<(), DisplayError<DI::Error>> {
            St7789::sleep(self).await
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_graphics_core::draw_target::DrawTarget;
    use embedded_graphics_core::geometry::{Point, Size};
    use embedded_graphics_core::pixelcolor::RgbColor;
    use embedded_graphics_core::primitives::Rectangle;
    use embedded_hal_mock::eh1::delay::NoopDelay;
    use embedded_hal_mock::eh1::digital::{Mock as PinMock, State, Transaction as PinTx};
    use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiTx};
    use std::vec;
    use std::vec::Vec;

    #[derive(Default)]
    struct Script {
        spi: Vec<SpiTx<u8>>,
        dc: Vec<PinTx>,
    }

    impl Script {
        fn write(&mut self, data: bool, bytes: &[u8]) -> &mut Self {
            if bytes.is_empty() {
                return self;
            }
            self.dc
                .push(PinTx::set(if data { State::High } else { State::Low }));
            self.spi.push(SpiTx::transaction_start());
            self.spi.push(SpiTx::write_vec(bytes.to_vec()));
            self.spi.push(SpiTx::transaction_end());
            self
        }
        fn cmd(&mut self, c: u8, data: &[u8]) -> &mut Self {
            self.write(false, &[c]).write(true, data)
        }
        fn window(&mut self, x0: u16, x1: u16, y0: u16, y1: u16) -> &mut Self {
            self.cmd(0x2A, &addr(x0, x1))
                .cmd(0x2B, &addr(y0, y1))
                .write(false, &[0x2C])
        }
        fn mocks(&self) -> (SpiMock<u8>, PinMock) {
            (SpiMock::new(&self.spi), PinMock::new(&self.dc))
        }
    }

    #[test]
    fn st7789_init_sequence() {
        let mut s = Script::default();
        s.cmd(0x01, &[])
            .cmd(0x11, &[])
            .cmd(0x3A, &[0x55])
            .cmd(0x36, &[0x60])
            .cmd(0x21, &[])
            .cmd(0x13, &[])
            .cmd(0x29, &[]);
        let (mut spi, mut dc) = s.mocks();
        let mut rst = PinMock::new(&[PinTx::set(State::Low), PinTx::set(State::High)]);
        let mut tft = St7789::new(
            SpiInterface::new(spi.clone(), dc.clone()),
            Panel::ST7789_240X320,
        )
        .with_reset(rst.clone())
        .with_rotation(Rotation::Deg90);
        tft.init(&mut NoopDelay).unwrap();
        assert_eq!(tft.size(), (320, 240));
        spi.done();
        dc.done();
        rst.done();
    }

    #[test]
    fn st7735_init_uses_colmod_05_bgr_and_inversion() {
        let mut s = Script::default();
        s.cmd(0x01, &[])
            .cmd(0x11, &[])
            .cmd(0x3A, &[0x05])
            .cmd(0x36, &[0x08])
            .cmd(0x21, &[])
            .cmd(0x13, &[])
            .cmd(0x29, &[]);
        let (mut spi, mut dc) = s.mocks();
        let mut tft = St7789::new(
            SpiInterface::new(spi.clone(), dc.clone()),
            Panel::ST7735_80X160,
        );
        tft.init(&mut NoopDelay).unwrap();
        spi.done();
        dc.done();
    }

    #[test]
    fn offsets_follow_rotation() {
        let p = Panel::ST7789_135X240;
        assert_eq!(p.offset(Rotation::Deg0), (52, 40));
        assert_eq!(p.offset(Rotation::Deg90), (40, 53));
        assert_eq!(p.offset(Rotation::Deg180), (53, 40));
        assert_eq!(p.offset(Rotation::Deg270), (40, 52));
        let q = Panel::ST7789_240X240;
        assert_eq!(q.offset(Rotation::Deg180), (0, 80));
        assert_eq!(q.offset(Rotation::Deg270), (80, 0));
        assert_eq!(q.madctl(Rotation::Deg180), 0xC0);
        assert_eq!(q.madctl(Rotation::Deg270), 0xA0);
        assert_eq!(Panel::ST7735_80X160.offset(Rotation::Deg180), (26, 1));
        assert!(Panel::new(Controller::St7735, 128, 160).is_some());
        assert!(Panel::new(Controller::St7735, 240, 240).is_none());
        assert!(Panel::ST7789_240X240.with_offset(1, 81).is_none());
        assert_eq!(Panel::ST7789_240X320.frame_len(), 153_600);
    }

    #[test]
    fn fill_area_streams_in_chunks_with_offset() {
        // 33 pixels = 66 bytes: one 64-byte chunk + 2 bytes.
        let mut s = Script::default();
        s.window(52 + 1, 52 + 11, 40 + 2, 40 + 4)
            .write(true, &[0xF8, 0x00].repeat(32))
            .write(true, &[0xF8, 0x00]);
        let (mut spi, mut dc) = s.mocks();
        let mut tft = St7789::new(
            SpiInterface::new(spi.clone(), dc.clone()),
            Panel::ST7789_135X240,
        );
        tft.fill_area(Area::new(1, 2, 11, 3), Rgb565::RED).unwrap();
        assert_eq!(
            tft.fill_area(Area::new(130, 0, 6, 1), Rgb565::RED),
            Err(Error::InvalidArgument)
        );
        spi.done();
        dc.done();
    }

    #[test]
    fn tile_full_and_dirty_writes() {
        let mut buf = [0u8; 4 * 3 * 2];
        let mut tile = Rgb565Frame::new(&mut buf[..], 4, 3).unwrap();
        tile.clear_dirty();
        tile.set_pixel(1, 1, Rgb565::BLUE);
        tile.set_pixel(2, 1, Rgb565::GREEN);
        let mut s = Script::default();
        // dirty: x 1..=2, y 1 → screen (11..=12, 21)
        s.window(11, 12, 21, 21)
            .write(true, &[0x00, 0x1F, 0x07, 0xE0]);
        // whole tile at (0, 0)
        let mut all = vec![0u8; 24];
        all[10..14].copy_from_slice(&[0x00, 0x1F, 0x07, 0xE0]);
        s.window(0, 3, 0, 2).write(true, &all);
        let (mut spi, mut dc) = s.mocks();
        let mut tft = St7789::new(
            SpiInterface::new(spi.clone(), dc.clone()),
            Panel::ST7789_240X320,
        );
        assert!(tft.write_dirty_at(10, 20, &mut tile).unwrap());
        assert!(!tft.write_dirty_at(10, 20, &mut tile).unwrap());
        tft.write_frame_at(0, 0, &tile).unwrap();
        assert_eq!(
            tft.write_pixels(Area::new(0, 0, 2, 2), &[0; 6]),
            Err(Error::InvalidArgument)
        );
        spi.done();
        dc.done();
    }

    #[test]
    fn draw_target_clips_and_windows() {
        let mut s = Script::default();
        // fill_solid partly off-screen → clipped window 126..=127, 0..=0
        s.window(126, 127, 0, 0)
            .write(true, &[0xFF, 0xFF, 0xFF, 0xFF]);
        // contiguous fill fully on-screen
        s.window(0, 1, 0, 0).write(true, &[0xF8, 0x00, 0x00, 0x1F]);
        // single pixel
        s.window(5, 5, 6, 6).write(true, &[0x07, 0xE0]);
        let (mut spi, mut dc) = s.mocks();
        let mut tft = St7789::new(
            SpiInterface::new(spi.clone(), dc.clone()),
            Panel::ST7735_128X160,
        )
        .with_rotation(Rotation::Deg0);
        tft.fill_solid(
            &Rectangle::new(Point::new(126, -3), Size::new(10, 4)),
            Rgb565::WHITE,
        )
        .unwrap();
        tft.fill_contiguous(
            &Rectangle::new(Point::new(0, 0), Size::new(2, 1)),
            [Rgb565::RED, Rgb565::BLUE],
        )
        .unwrap();
        tft.draw_iter([
            embedded_graphics_core::Pixel(Point::new(5, 6), Rgb565::GREEN),
            embedded_graphics_core::Pixel(Point::new(-1, 6), Rgb565::GREEN),
        ])
        .unwrap();
        spi.done();
        dc.done();
    }

    #[test]
    fn sleep_wake_and_async_tile() {
        let mut s = Script::default();
        s.cmd(0x28, &[])
            .cmd(0x10, &[])
            .cmd(0x11, &[])
            .cmd(0x29, &[])
            .cmd(0x36, &[0xC0])
            .window(0, 0, 80, 80)
            .write(true, &[0x00, 0x00]);
        let (mut spi, mut dc) = s.mocks();
        let tile = Rgb565Frame::new([0u8; 2], 1, 1).unwrap();
        let mut tft = asynch::St7789::new(
            SpiInterface::new(spi.clone(), dc.clone()),
            Panel::ST7789_240X240,
        );
        block_on(async {
            tft.sleep().await?;
            tft.wake(&mut NoopDelay).await?;
            tft.set_rotation(Rotation::Deg180).await?;
            tft.write_frame_at(0, 0, &tile).await
        })
        .unwrap();
        spi.done();
        dc.done();
    }

    fn block_on<F: core::future::Future>(fut: F) -> F::Output {
        let mut cx = core::task::Context::from_waker(core::task::Waker::noop());
        let mut fut = core::pin::pin!(fut);
        loop {
            if let core::task::Poll::Ready(v) = fut.as_mut().poll(&mut cx) {
                return v;
            }
        }
    }
}
