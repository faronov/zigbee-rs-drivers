//! Caller-owned frame buffers with embedded-graphics [`DrawTarget`]s.
//!
//! The storage `B` is anything that is `AsRef<[u8]> + AsMut<[u8]>`: an owned
//! `[u8; N]` (size it with the `*_len` const fns) or a borrowed `&mut [u8]`
//! (e.g. a `static` buffer). The RAM cost is therefore explicit and paid by
//! the application, never hidden in a driver.
//!
//! | Type | Layout | Used by | Bytes |
//! |---|---|---|---|
//! | [`MonoFrame`] | rows, MSB = leftmost pixel, bit 0 = ink | Sharp memory LCD, SSD1680/1681/1675, UC8151 B/W, IL3820/IL3895 | [`mono_len`] |
//! | [`TriFrame`] | two `MonoFrame` planes (black, red) | SSD1680/1681, UC8151/IL0373 tricolour | [`tri_len`] |
//! | [`PageFrame`] | 8-row pages, LSB = top pixel, bit 1 = lit | SSD1306, SH1106 | [`page_len`] |
//! | [`Rgb565Frame`] | rows, 2 bytes/pixel big-endian | ST7789/ST7735 tiles | [`rgb565_len`] |
//!
//! Frames know their native (panel) size and an optional software
//! [`Rotation`]; drawing uses rotated (logical) coordinates while
//! [`Area`]s, rows and the dirty rectangle are in native coordinates,
//! because that is what the controllers address.
//!
//! Every frame tracks a dirty rectangle: the bounding box of the pixels
//! whose value changed since the last [`MonoFrame::clear_dirty`]. Drivers
//! use it for line-, page- or window-limited updates.

use core::convert::Infallible;

use embedded_graphics_core::Pixel;
use embedded_graphics_core::draw_target::DrawTarget;
use embedded_graphics_core::geometry::{OriginDimensions, Size};
use embedded_graphics_core::pixelcolor::{BinaryColor, PixelColor, Rgb565, raw::RawU16};
use embedded_graphics_core::prelude::RawData;

/// Bytes of a [`MonoFrame`]: `ceil(width / 8) * height`.
pub const fn mono_len(width: u16, height: u16) -> usize {
    (width as usize).div_ceil(8) * height as usize
}

/// Bytes of a [`TriFrame`]: two mono planes.
pub const fn tri_len(width: u16, height: u16) -> usize {
    2 * mono_len(width, height)
}

/// Bytes of a [`PageFrame`]: `width * ceil(height / 8)`.
pub const fn page_len(width: u16, height: u16) -> usize {
    width as usize * (height as usize).div_ceil(8)
}

/// Bytes of an [`Rgb565Frame`]: `width * height * 2`.
pub const fn rgb565_len(width: u16, height: u16) -> usize {
    width as usize * height as usize * 2
}

/// Software rotation applied when drawing (clockwise).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Rotation {
    /// Native orientation.
    #[default]
    Deg0,
    /// Rotated 90° clockwise: logical width = native height.
    Deg90,
    /// Rotated 180°.
    Deg180,
    /// Rotated 270° clockwise: logical width = native height.
    Deg270,
}

/// Rectangle in native panel coordinates (pixels).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Area {
    /// Left column.
    pub x: u16,
    /// Top row.
    pub y: u16,
    /// Width in pixels.
    pub width: u16,
    /// Height in pixels.
    pub height: u16,
}

impl Area {
    /// Area from origin and size.
    pub const fn new(x: u16, y: u16, width: u16, height: u16) -> Self {
        Self {
            x,
            y,
            width,
            height,
        }
    }

    /// The whole `width` x `height` panel.
    pub const fn full(width: u16, height: u16) -> Self {
        Self::new(0, 0, width, height)
    }

    /// No pixels.
    pub const fn is_empty(&self) -> bool {
        self.width == 0 || self.height == 0
    }

    /// First column right of the area (`x + width`).
    pub const fn right(&self) -> u16 {
        self.x.saturating_add(self.width)
    }

    /// First row below the area (`y + height`).
    pub const fn bottom(&self) -> u16 {
        self.y.saturating_add(self.height)
    }

    /// Smallest area containing both.
    pub fn union(self, other: Area) -> Area {
        if self.is_empty() {
            return other;
        }
        if other.is_empty() {
            return self;
        }
        let x = self.x.min(other.x);
        let y = self.y.min(other.y);
        Area::new(
            x,
            y,
            self.right().max(other.right()) - x,
            self.bottom().max(other.bottom()) - y,
        )
    }

    /// Clip to a `width` x `height` panel; `None` if nothing is left.
    pub fn clip(self, width: u16, height: u16) -> Option<Area> {
        let right = self.right().min(width);
        let bottom = self.bottom().min(height);
        if self.x >= right || self.y >= bottom {
            return None;
        }
        Some(Area::new(self.x, self.y, right - self.x, bottom - self.y))
    }

    /// Grow horizontally to multiples of 8 pixels (byte boundaries of a
    /// row-major 1-bpp buffer), clipped to `panel_width`.
    pub fn align_x8(self, panel_width: u16) -> Area {
        let x = self.x & !7;
        let right = self.right().div_ceil(8).saturating_mul(8).min(panel_width);
        Area::new(x, self.y, right.saturating_sub(x), self.height)
    }

    /// Grow vertically to multiples of 8 rows (pages of a [`PageFrame`]),
    /// clipped to `panel_height`.
    pub fn align_y8(self, panel_height: u16) -> Area {
        let y = self.y & !7;
        let bottom = self
            .bottom()
            .div_ceil(8)
            .saturating_mul(8)
            .min(panel_height);
        Area::new(self.x, y, self.width, bottom.saturating_sub(y))
    }
}

/// Size, rotation and dirty tracking shared by all frame types.
#[derive(Debug, Clone, Copy)]
struct Geometry {
    width: u16,
    height: u16,
    rotation: Rotation,
    /// Inclusive native bounds (x0, y0, x1, y1).
    dirty: Option<(u16, u16, u16, u16)>,
}

impl Geometry {
    const fn new(width: u16, height: u16) -> Self {
        Self {
            width,
            height,
            rotation: Rotation::Deg0,
            dirty: None,
        }
    }

    fn logical_size(&self) -> Size {
        match self.rotation {
            Rotation::Deg0 | Rotation::Deg180 => {
                Size::new(u32::from(self.width), u32::from(self.height))
            }
            Rotation::Deg90 | Rotation::Deg270 => {
                Size::new(u32::from(self.height), u32::from(self.width))
            }
        }
    }

    /// Logical (rotated) coordinates to native ones; `None` if outside.
    fn map_native(&self, x: i32, y: i32) -> Option<(u16, u16)> {
        let w = i32::from(self.width);
        let h = i32::from(self.height);
        let (nx, ny) = match self.rotation {
            Rotation::Deg0 => (x, y),
            Rotation::Deg90 => (w - 1 - y, x),
            Rotation::Deg180 => (w - 1 - x, h - 1 - y),
            Rotation::Deg270 => (y, h - 1 - x),
        };
        if nx < 0 || ny < 0 || nx >= w || ny >= h {
            return None;
        }
        Some((nx as u16, ny as u16))
    }

    fn mark(&mut self, x: u16, y: u16) {
        self.dirty = Some(match self.dirty {
            None => (x, y, x, y),
            Some((x0, y0, x1, y1)) => (x0.min(x), y0.min(y), x1.max(x), y1.max(y)),
        });
    }

    fn mark_all(&mut self) {
        if self.width > 0 && self.height > 0 {
            self.dirty = Some((0, 0, self.width - 1, self.height - 1));
        }
    }

    fn dirty_area(&self) -> Option<Area> {
        self.dirty
            .map(|(x0, y0, x1, y1)| Area::new(x0, y0, x1 - x0 + 1, y1 - y0 + 1))
    }
}

/// Methods every frame type has, generated once.
macro_rules! common_frame_methods {
    () => {
        /// Native (panel) width in pixels.
        pub const fn width(&self) -> u16 {
            self.geo.width
        }

        /// Native (panel) height in pixels.
        pub const fn height(&self) -> u16 {
            self.geo.height
        }

        /// The whole native panel as an [`Area`].
        pub const fn full_area(&self) -> Area {
            Area::full(self.geo.width, self.geo.height)
        }

        /// Current software rotation.
        pub const fn rotation(&self) -> Rotation {
            self.geo.rotation
        }

        /// Change the software rotation used by later drawing.
        pub fn set_rotation(&mut self, rotation: Rotation) {
            self.geo.rotation = rotation;
        }

        /// Builder form of [`Self::set_rotation`].
        pub fn with_rotation(mut self, rotation: Rotation) -> Self {
            self.geo.rotation = rotation;
            self
        }

        /// Bounding box (native coordinates) of the pixels changed since
        /// the last [`Self::clear_dirty`]; `None` if nothing changed.
        pub fn dirty(&self) -> Option<Area> {
            self.geo.dirty_area()
        }

        /// Forget the dirty rectangle (after the driver pushed it).
        pub fn clear_dirty(&mut self) {
            self.geo.dirty = None;
        }

        /// Mark the whole frame dirty.
        pub fn mark_all_dirty(&mut self) {
            self.geo.mark_all();
        }

        /// Give the storage back.
        pub fn release(self) -> B {
            self.buf
        }
    };
}

/// 1 bit per pixel, row-major, MSB = leftmost pixel; bit `0` = ink
/// ([`BinaryColor::On`], black), bit `1` = paper ([`BinaryColor::Off`],
/// white). This is the native RAM format of Sharp memory LCDs and of the
/// black/white RAM of SSD16xx, UC8151 and IL38xx e-paper controllers.
pub struct MonoFrame<B> {
    buf: B,
    geo: Geometry,
}

impl<B: AsRef<[u8]>> MonoFrame<B> {
    /// Wrap a buffer of at least [`mono_len`]`(width, height)` bytes;
    /// `None` if it is too small. The content is left as is (call
    /// `clear(BinaryColor::Off)` for a white page).
    pub fn new(buf: B, width: u16, height: u16) -> Option<Self> {
        (buf.as_ref().len() >= mono_len(width, height)).then_some(Self {
            buf,
            geo: Geometry::new(width, height),
        })
    }

    common_frame_methods!();

    /// Bytes per row.
    pub const fn stride(&self) -> usize {
        (self.geo.width as usize).div_ceil(8)
    }

    /// The frame bytes, row 0 first.
    pub fn as_bytes(&self) -> &[u8] {
        &self.buf.as_ref()[..mono_len(self.geo.width, self.geo.height)]
    }

    /// Bytes of native row `y`.
    pub fn row(&self, y: u16) -> &[u8] {
        let s = self.stride();
        &self.as_bytes()[usize::from(y) * s..][..s]
    }

    /// Native rows of `area`, each cut to the bytes that cover
    /// `area.x .. area.right()` (whole bytes; align the area with
    /// [`Area::align_x8`] first when the controller needs it).
    pub fn window_rows(&self, area: Area) -> impl Iterator<Item = &[u8]> + '_ {
        window_rows(self.as_bytes(), self.stride(), area)
    }

    /// Pixel at logical coordinates; `None` if outside.
    pub fn pixel(&self, x: i32, y: i32) -> Option<BinaryColor> {
        let (nx, ny) = self.geo.map_native(x, y)?;
        let (i, mask) = mono_index(self.stride(), nx, ny);
        Some(if self.buf.as_ref()[i] & mask == 0 {
            BinaryColor::On
        } else {
            BinaryColor::Off
        })
    }
}

impl<B: AsRef<[u8]> + AsMut<[u8]>> MonoFrame<B> {
    /// Mutable frame bytes. Changes made here are not dirty-tracked; call
    /// [`Self::mark_all_dirty`] if needed.
    pub fn as_mut_bytes(&mut self) -> &mut [u8] {
        let n = mono_len(self.geo.width, self.geo.height);
        &mut self.buf.as_mut()[..n]
    }

    /// Set the pixel at logical coordinates; ignored if outside.
    pub fn set_pixel(&mut self, x: i32, y: i32, color: BinaryColor) {
        if let Some((nx, ny)) = self.geo.map_native(x, y) {
            let (i, mask) = mono_index(self.stride(), nx, ny);
            let byte = &mut self.buf.as_mut()[i];
            let new = if color.is_on() {
                *byte & !mask
            } else {
                *byte | mask
            };
            if new != *byte {
                *byte = new;
                self.geo.mark(nx, ny);
            }
        }
    }

    /// Fill the whole frame and mark it dirty.
    pub fn fill(&mut self, color: BinaryColor) {
        let value = if color.is_on() { 0x00 } else { 0xFF };
        self.as_mut_bytes().fill(value);
        self.geo.mark_all();
    }
}

fn mono_index(stride: usize, x: u16, y: u16) -> (usize, u8) {
    (
        usize::from(y) * stride + usize::from(x / 8),
        0x80 >> (x % 8),
    )
}

/// Rows `area.y .. area.bottom()` of a row-major 1-bpp plane, each cut to
/// the bytes covering columns `area.x .. area.right()`.
pub fn window_rows(plane: &[u8], stride: usize, area: Area) -> impl Iterator<Item = &[u8]> + '_ {
    let first = usize::from(area.x / 8);
    let last = usize::from(area.right()).div_ceil(8).min(stride);
    (area.y..area.bottom()).map(move |y| {
        let row = usize::from(y) * stride;
        &plane[row + first..row + last.max(first)]
    })
}

impl<B: AsRef<[u8]>> OriginDimensions for MonoFrame<B> {
    fn size(&self) -> Size {
        self.geo.logical_size()
    }
}

impl<B: AsRef<[u8]> + AsMut<[u8]>> DrawTarget for MonoFrame<B> {
    type Color = BinaryColor;
    type Error = Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Infallible>
    where
        I: IntoIterator<Item = Pixel<BinaryColor>>,
    {
        for Pixel(p, c) in pixels {
            self.set_pixel(p.x, p.y, c);
        }
        Ok(())
    }

    fn clear(&mut self, color: BinaryColor) -> Result<(), Infallible> {
        self.fill(color);
        Ok(())
    }
}

/// Colours of a black/white/red (or black/white/yellow) e-paper panel.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum TriColor {
    /// Paper.
    #[default]
    White,
    /// Black ink.
    Black,
    /// Third pigment (red or yellow, depending on the panel).
    Red,
}

impl PixelColor for TriColor {
    type Raw = ();
}

impl From<BinaryColor> for TriColor {
    fn from(c: BinaryColor) -> Self {
        if c.is_on() { Self::Black } else { Self::White }
    }
}

/// Two row-major 1-bpp planes for tricolour e-paper, stored back to back
/// in one buffer of [`tri_len`] bytes.
///
/// * Black plane: bit `0` = black, `1` = white (as [`MonoFrame`]).
/// * Red plane: bit `1` = red/yellow, `0` = no pigment. Drivers whose
///   controller wants the opposite polarity invert it while sending.
pub struct TriFrame<B> {
    buf: B,
    geo: Geometry,
}

impl<B: AsRef<[u8]>> TriFrame<B> {
    /// Wrap a buffer of at least [`tri_len`]`(width, height)` bytes.
    pub fn new(buf: B, width: u16, height: u16) -> Option<Self> {
        (buf.as_ref().len() >= tri_len(width, height)).then_some(Self {
            buf,
            geo: Geometry::new(width, height),
        })
    }

    common_frame_methods!();

    /// Bytes per row of one plane.
    pub const fn stride(&self) -> usize {
        (self.geo.width as usize).div_ceil(8)
    }

    /// Black plane (bit 0 = black).
    pub fn black_plane(&self) -> &[u8] {
        let n = mono_len(self.geo.width, self.geo.height);
        &self.buf.as_ref()[..n]
    }

    /// Red plane (bit 1 = red).
    pub fn red_plane(&self) -> &[u8] {
        let n = mono_len(self.geo.width, self.geo.height);
        &self.buf.as_ref()[n..2 * n]
    }

    /// Pixel at logical coordinates; `None` if outside. Red wins over
    /// black, as on the panel.
    pub fn pixel(&self, x: i32, y: i32) -> Option<TriColor> {
        let (nx, ny) = self.geo.map_native(x, y)?;
        let (i, mask) = mono_index(self.stride(), nx, ny);
        Some(if self.red_plane()[i] & mask != 0 {
            TriColor::Red
        } else if self.black_plane()[i] & mask == 0 {
            TriColor::Black
        } else {
            TriColor::White
        })
    }
}

impl<B: AsRef<[u8]> + AsMut<[u8]>> TriFrame<B> {
    /// Set the pixel at logical coordinates; ignored if outside.
    pub fn set_pixel(&mut self, x: i32, y: i32, color: TriColor) {
        if let Some((nx, ny)) = self.geo.map_native(x, y) {
            let n = mono_len(self.geo.width, self.geo.height);
            let (i, mask) = mono_index(self.stride(), nx, ny);
            let buf = self.buf.as_mut();
            let (black, red) = (buf[i], buf[n + i]);
            let (new_black, new_red) = match color {
                TriColor::White => (black | mask, red & !mask),
                TriColor::Black => (black & !mask, red & !mask),
                TriColor::Red => (black | mask, red | mask),
            };
            if (new_black, new_red) != (black, red) {
                buf[i] = new_black;
                buf[n + i] = new_red;
                self.geo.mark(nx, ny);
            }
        }
    }

    /// Fill the whole frame and mark it dirty.
    pub fn fill(&mut self, color: TriColor) {
        let n = mono_len(self.geo.width, self.geo.height);
        let (black, red) = match color {
            TriColor::White => (0xFF, 0x00),
            TriColor::Black => (0x00, 0x00),
            TriColor::Red => (0xFF, 0xFF),
        };
        let buf = self.buf.as_mut();
        buf[..n].fill(black);
        buf[n..2 * n].fill(red);
        self.geo.mark_all();
    }
}

impl<B: AsRef<[u8]>> OriginDimensions for TriFrame<B> {
    fn size(&self) -> Size {
        self.geo.logical_size()
    }
}

impl<B: AsRef<[u8]> + AsMut<[u8]>> DrawTarget for TriFrame<B> {
    type Color = TriColor;
    type Error = Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Infallible>
    where
        I: IntoIterator<Item = Pixel<TriColor>>,
    {
        for Pixel(p, c) in pixels {
            self.set_pixel(p.x, p.y, c);
        }
        Ok(())
    }

    fn clear(&mut self, color: TriColor) -> Result<(), Infallible> {
        self.fill(color);
        Ok(())
    }
}

/// 1 bit per pixel in 8-row pages (SSD1306/SH1106 GDDRAM): byte
/// `page * width + x` holds rows `8 * page .. 8 * page + 7` of column `x`,
/// LSB = top row; bit `1` = lit ([`BinaryColor::On`]).
pub struct PageFrame<B> {
    buf: B,
    geo: Geometry,
}

impl<B: AsRef<[u8]>> PageFrame<B> {
    /// Wrap a buffer of at least [`page_len`]`(width, height)` bytes.
    pub fn new(buf: B, width: u16, height: u16) -> Option<Self> {
        (buf.as_ref().len() >= page_len(width, height)).then_some(Self {
            buf,
            geo: Geometry::new(width, height),
        })
    }

    common_frame_methods!();

    /// Number of 8-row pages.
    pub const fn pages(&self) -> u16 {
        self.geo.height.div_ceil(8)
    }

    /// The frame bytes, page 0 first.
    pub fn as_bytes(&self) -> &[u8] {
        &self.buf.as_ref()[..page_len(self.geo.width, self.geo.height)]
    }

    /// Bytes of `page`, columns `x0 .. x1`.
    pub fn page(&self, page: u16, x0: u16, x1: u16) -> &[u8] {
        let w = usize::from(self.geo.width);
        let base = usize::from(page) * w;
        &self.as_bytes()[base + usize::from(x0)..base + usize::from(x1)]
    }

    /// Pixel at logical coordinates; `None` if outside.
    pub fn pixel(&self, x: i32, y: i32) -> Option<BinaryColor> {
        let (nx, ny) = self.geo.map_native(x, y)?;
        let (i, mask) = page_index(self.geo.width, nx, ny);
        Some(BinaryColor::from(self.buf.as_ref()[i] & mask != 0))
    }
}

fn page_index(width: u16, x: u16, y: u16) -> (usize, u8) {
    (
        usize::from(y / 8) * usize::from(width) + usize::from(x),
        1 << (y % 8),
    )
}

impl<B: AsRef<[u8]> + AsMut<[u8]>> PageFrame<B> {
    /// Mutable frame bytes (not dirty-tracked).
    pub fn as_mut_bytes(&mut self) -> &mut [u8] {
        let n = page_len(self.geo.width, self.geo.height);
        &mut self.buf.as_mut()[..n]
    }

    /// Set the pixel at logical coordinates; ignored if outside.
    pub fn set_pixel(&mut self, x: i32, y: i32, color: BinaryColor) {
        if let Some((nx, ny)) = self.geo.map_native(x, y) {
            let (i, mask) = page_index(self.geo.width, nx, ny);
            let byte = &mut self.buf.as_mut()[i];
            let new = if color.is_on() {
                *byte | mask
            } else {
                *byte & !mask
            };
            if new != *byte {
                *byte = new;
                self.geo.mark(nx, ny);
            }
        }
    }

    /// Fill the whole frame and mark it dirty.
    pub fn fill(&mut self, color: BinaryColor) {
        let value = if color.is_on() { 0xFF } else { 0x00 };
        self.as_mut_bytes().fill(value);
        self.geo.mark_all();
    }
}

impl<B: AsRef<[u8]>> OriginDimensions for PageFrame<B> {
    fn size(&self) -> Size {
        self.geo.logical_size()
    }
}

impl<B: AsRef<[u8]> + AsMut<[u8]>> DrawTarget for PageFrame<B> {
    type Color = BinaryColor;
    type Error = Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Infallible>
    where
        I: IntoIterator<Item = Pixel<BinaryColor>>,
    {
        for Pixel(p, c) in pixels {
            self.set_pixel(p.x, p.y, c);
        }
        Ok(())
    }

    fn clear(&mut self, color: BinaryColor) -> Result<(), Infallible> {
        self.fill(color);
        Ok(())
    }
}

/// RGB565, 2 bytes per pixel big-endian (the byte order ST77xx expect),
/// row-major. Typically a small tile that a TFT driver writes into a
/// window, so a full-screen buffer is not needed.
pub struct Rgb565Frame<B> {
    buf: B,
    geo: Geometry,
}

impl<B: AsRef<[u8]>> Rgb565Frame<B> {
    /// Wrap a buffer of at least [`rgb565_len`]`(width, height)` bytes.
    pub fn new(buf: B, width: u16, height: u16) -> Option<Self> {
        (buf.as_ref().len() >= rgb565_len(width, height)).then_some(Self {
            buf,
            geo: Geometry::new(width, height),
        })
    }

    common_frame_methods!();

    /// The frame bytes, row 0 first.
    pub fn as_bytes(&self) -> &[u8] {
        &self.buf.as_ref()[..rgb565_len(self.geo.width, self.geo.height)]
    }

    /// Pixel at logical coordinates; `None` if outside.
    pub fn pixel(&self, x: i32, y: i32) -> Option<Rgb565> {
        let (nx, ny) = self.geo.map_native(x, y)?;
        let i = rgb_index(self.geo.width, nx, ny);
        let b = self.buf.as_ref();
        Some(RawU16::new(u16::from_be_bytes([b[i], b[i + 1]])).into())
    }
}

fn rgb_index(width: u16, x: u16, y: u16) -> usize {
    (usize::from(y) * usize::from(width) + usize::from(x)) * 2
}

impl<B: AsRef<[u8]> + AsMut<[u8]>> Rgb565Frame<B> {
    /// Set the pixel at logical coordinates; ignored if outside.
    pub fn set_pixel(&mut self, x: i32, y: i32, color: Rgb565) {
        if let Some((nx, ny)) = self.geo.map_native(x, y) {
            let i = rgb_index(self.geo.width, nx, ny);
            let bytes = RawU16::from(color).into_inner().to_be_bytes();
            let b = self.buf.as_mut();
            if b[i..i + 2] != bytes {
                b[i..i + 2].copy_from_slice(&bytes);
                self.geo.mark(nx, ny);
            }
        }
    }

    /// Fill the whole frame and mark it dirty.
    pub fn fill(&mut self, color: Rgb565) {
        let bytes = RawU16::from(color).into_inner().to_be_bytes();
        let n = rgb565_len(self.geo.width, self.geo.height);
        for px in self.buf.as_mut()[..n].chunks_exact_mut(2) {
            px.copy_from_slice(&bytes);
        }
        self.geo.mark_all();
    }
}

impl<B: AsRef<[u8]>> OriginDimensions for Rgb565Frame<B> {
    fn size(&self) -> Size {
        self.geo.logical_size()
    }
}

impl<B: AsRef<[u8]> + AsMut<[u8]>> DrawTarget for Rgb565Frame<B> {
    type Color = Rgb565;
    type Error = Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Infallible>
    where
        I: IntoIterator<Item = Pixel<Rgb565>>,
    {
        for Pixel(p, c) in pixels {
            self.set_pixel(p.x, p.y, c);
        }
        Ok(())
    }

    fn clear(&mut self, color: Rgb565) -> Result<(), Infallible> {
        self.fill(color);
        Ok(())
    }
}
