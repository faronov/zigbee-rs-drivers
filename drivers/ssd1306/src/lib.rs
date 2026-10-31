//! Solomon SSD1306 and Sino Wealth SH1106 monochrome OLED controllers over
//! I2C or 4-wire SPI.
//!
//! Both controllers keep a 1-bit page-organised GDDRAM (8 rows per byte,
//! LSB = top row). The driver writes [`PageFrame`] pages in *page
//! addressing mode*, which both chips support: for every page
//! `0xB0 | page`, the column low/high nibble (`0x0X`, `0x1X`) plus the
//! panel's column offset, then the data. The SH1106 has 132 columns and
//! 128-pixel modules show columns 2..130, hence [`Panel::SH1106_128X64`]
//! uses an offset of 2.
//!
//! OLEDs light up as soon as RAM is written: there is no BUSY and no
//! refresh phase. Expect roughly 10–20 mA with half the pixels lit, about
//! 10 µA in [`Ssd1306::sleep`] (display off and charge pump off).
//!
//! References: Solomon Systech SSD1306 datasheet rev 1.1 (command table,
//! application note "charge pump setting"); Sino Wealth SH1106 datasheet
//! v2.3.
//!
//! ```ignore
//! let di = I2cInterface::new(i2c, ADDRESS);
//! let mut oled = Ssd1306::new(di, Panel::SSD1306_128X64);
//! oled.init(&mut delay)?;
//! let mut fb = PageFrame::new([0u8; page_len(128, 64)], 128, 64).unwrap();
//! Text::new("Hello", Point::new(0, 10), style).draw(&mut fb)?;
//! oled.flush_dirty(&mut fb)?;
//! ```

#![no_std]

use embedded_hal::digital::OutputPin;
use zigbee_display::{Area, DisplayError, Error, NoPin, PageFrame, WaitUs, pin_error};

pub use zigbee_display;
pub use zigbee_display::{I2cInterface, SpiInterface};

/// I2C address with SA0 (D/C) low.
pub const ADDRESS: u8 = 0x3C;
/// I2C address with SA0 (D/C) high.
pub const ADDRESS_ALT: u8 = 0x3D;

/// Reset pulse and post-reset wait used by `init` (datasheet: ≥ 3 µs).
pub const RESET_PULSE_US: u32 = 1_000;

/// Controller family.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Controller {
    /// Solomon SSD1306: 128 columns, internal charge pump (`0x8D`).
    Ssd1306,
    /// Sino Wealth SH1106: 132 columns, DC-DC control (`0xAD`).
    Sh1106,
}

/// Panel geometry and wiring of the COM lines.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Panel {
    controller: Controller,
    width: u16,
    height: u16,
    column_offset: u8,
    com_pins: u8,
}

impl Panel {
    /// 0.96" SSD1306 128x64.
    pub const SSD1306_128X64: Panel = Panel::raw(Controller::Ssd1306, 128, 64, 0, 0x12);
    /// 0.91" SSD1306 128x32 (sequential COM).
    pub const SSD1306_128X32: Panel = Panel::raw(Controller::Ssd1306, 128, 32, 0, 0x02);
    /// 1.3" SH1106 128x64 (column offset 2).
    pub const SH1106_128X64: Panel = Panel::raw(Controller::Sh1106, 128, 64, 2, 0x12);

    const fn raw(controller: Controller, width: u16, height: u16, offset: u8, com: u8) -> Self {
        Self {
            controller,
            width,
            height,
            column_offset: offset,
            com_pins: com,
        }
    }

    /// Custom geometry. `com_pins` is the `0xDA` argument (`0x12`
    /// alternative, `0x02` sequential). `None` unless `height` is a
    /// multiple of 8 up to 64 and `offset + width` fits the controller's
    /// columns (128 / 132).
    pub const fn new(
        controller: Controller,
        width: u16,
        height: u16,
        column_offset: u8,
        com_pins: u8,
    ) -> Option<Self> {
        let columns = match controller {
            Controller::Ssd1306 => 128,
            Controller::Sh1106 => 132,
        };
        if width == 0 || width + column_offset as u16 > columns {
            return None;
        }
        if height == 0 || height % 8 != 0 || height > 64 {
            return None;
        }
        Some(Self::raw(
            controller,
            width,
            height,
            column_offset,
            com_pins,
        ))
    }

    /// Controller family.
    pub const fn controller(&self) -> Controller {
        self.controller
    }

    /// Width in pixels.
    pub const fn width(&self) -> u16 {
        self.width
    }

    /// Height in pixels.
    pub const fn height(&self) -> u16 {
        self.height
    }

    /// Frame buffer bytes ([`zigbee_display::page_len`]).
    pub const fn frame_len(&self) -> usize {
        zigbee_display::page_len(self.width, self.height)
    }
}

/// Init sequence up to (not including) RAM clear and display on.
fn init_sequence(panel: &Panel, contrast: u8, rotate180: bool) -> ([u8; 32], usize) {
    let mut seq = [0u8; 32];
    let (seg, com) = if rotate180 {
        (0xA0, 0xC0)
    } else {
        (0xA1, 0xC8)
    };
    let mux = (panel.height - 1) as u8;
    let bytes: &[u8] = match panel.controller {
        Controller::Ssd1306 => &[
            0xAE, // display off
            0xD5,
            0x80, // clock divide / oscillator
            0xA8,
            mux, // multiplex ratio
            0xD3,
            0x00, // display offset
            0x40, // start line 0
            0x8D,
            0x14, // charge pump on
            0x20,
            0x02, // page addressing mode
            seg,
            com, // segment remap, COM scan direction
            0xDA,
            panel.com_pins, // COM pins
            0x81,
            contrast, // contrast
            0xD9,
            0xF1, // pre-charge
            0xDB,
            0x20, // VCOMH deselect
            0xA4, // display follows RAM
            0xA6, // normal (not inverted)
            0x2E, // scrolling off
        ],
        Controller::Sh1106 => &[
            0xAE, // display off
            0xD5,
            0x80, // clock divide / oscillator
            0xA8,
            mux, // multiplex ratio
            0xD3,
            0x00, // display offset
            0x40, // start line 0
            0xAD,
            0x8B, // DC-DC on
            seg,
            com, // segment remap, COM scan direction
            0xDA,
            panel.com_pins, // COM pads
            0x81,
            contrast, // contrast
            0xD9,
            0x22, // pre-charge
            0xDB,
            0x35, // VCOM deselect
            0xA4, // display follows RAM
            0xA6, // normal (not inverted)
        ],
    };
    seq[..bytes.len()].copy_from_slice(bytes);
    (seq, bytes.len())
}

const ZEROS: [u8; 132] = [0; 132];

macro_rules! driver {
    ($Iface:path, $Delay:path, $($async:tt $await:tt)?) => {
        /// SSD1306 / SH1106 driver over any command/data interface.
        pub struct Ssd1306<DI, RST = NoPin> {
            di: DI,
            rst: Option<RST>,
            panel: Panel,
            contrast: u8,
            rotate180: bool,
        }

        impl<DI: $Iface> Ssd1306<DI> {
            /// Driver without a reset pin (default contrast `0x7F`).
            pub fn new(di: DI, panel: Panel) -> Self {
                Self {
                    di,
                    rst: None,
                    panel,
                    contrast: 0x7F,
                    rotate180: false,
                }
            }
        }

        impl<DI: $Iface, RST: OutputPin> Ssd1306<DI, RST> {
            /// Add a RES# pin, pulsed by [`Self::init`].
            pub fn with_reset<P: OutputPin>(self, rst: P) -> Ssd1306<DI, P> {
                Ssd1306 {
                    di: self.di,
                    rst: Some(rst),
                    panel: self.panel,
                    contrast: self.contrast,
                    rotate180: self.rotate180,
                }
            }

            /// Contrast used by [`Self::init`].
            pub fn with_contrast(mut self, contrast: u8) -> Self {
                self.contrast = contrast;
                self
            }

            /// Mirror both axes in hardware (module mounted upside down).
            pub fn with_rotate180(mut self, rotate180: bool) -> Self {
                self.rotate180 = rotate180;
                self
            }

            /// Panel configuration.
            pub const fn panel(&self) -> Panel {
                self.panel
            }

            /// Give the interface and reset pin back.
            pub fn release(self) -> (DI, Option<RST>) {
                (self.di, self.rst)
            }

            $($async)? fn cmd(&mut self, bytes: &[u8]) -> Result<(), DisplayError<DI::Error>> {
                self.di.command(bytes) $(.$await)?.map_err(Error::Bus)
            }

            $($async)? fn data(&mut self, bytes: &[u8]) -> Result<(), DisplayError<DI::Error>> {
                self.di.data(bytes) $(.$await)?.map_err(Error::Bus)
            }

            $($async)? fn set_page(&mut self, page: u16, x: u16) -> Result<(), DisplayError<DI::Error>> {
                let col = x + u16::from(self.panel.column_offset);
                self.cmd(&[0xB0 | page as u8, (col & 0x0F) as u8, 0x10 | (col >> 4) as u8]) $(.$await)?
            }

            /// Optional hardware reset, init sequence, clear RAM, display on.
            pub $($async)? fn init<D: $Delay>(&mut self, delay: &mut D) -> Result<(), DisplayError<DI::Error>> {
                if let Some(rst) = self.rst.as_mut() {
                    rst.set_low().map_err(pin_error)?;
                    delay.delay_us(RESET_PULSE_US) $(.$await)?;
                    rst.set_high().map_err(pin_error)?;
                    delay.delay_us(RESET_PULSE_US) $(.$await)?;
                }
                let (seq, len) = init_sequence(&self.panel, self.contrast, self.rotate180);
                self.cmd(&seq[..len]) $(.$await)??;
                self.clear_ram() $(.$await)??;
                self.cmd(&[0xAF]) $(.$await)?
            }

            /// Zero the visible GDDRAM (the frame is not touched).
            pub $($async)? fn clear_ram(&mut self) -> Result<(), DisplayError<DI::Error>> {
                let width = usize::from(self.panel.width);
                for page in 0..self.panel.height / 8 {
                    self.set_page(page, 0) $(.$await)??;
                    self.data(&ZEROS[..width]) $(.$await)??;
                }
                Ok(())
            }

            /// Write the pages and columns covering `area` (native coordinates).
            pub $($async)? fn flush_area<B: AsRef<[u8]>>(
                &mut self,
                fb: &PageFrame<B>,
                area: Area,
            ) -> Result<(), DisplayError<DI::Error>> {
                if fb.width() != self.panel.width || fb.height() != self.panel.height {
                    return Err(Error::InvalidArgument);
                }
                let Some(area) = area.clip(self.panel.width, self.panel.height) else {
                    return Ok(());
                };
                for page in area.y / 8..area.bottom().div_ceil(8) {
                    self.set_page(page, area.x) $(.$await)??;
                    self.data(fb.page(page, area.x, area.right())) $(.$await)??;
                }
                Ok(())
            }

            /// Write the whole frame.
            pub $($async)? fn flush<B: AsRef<[u8]>>(
                &mut self,
                fb: &PageFrame<B>,
            ) -> Result<(), DisplayError<DI::Error>> {
                let area = fb.full_area();
                self.flush_area(fb, area) $(.$await)?
            }

            /// Write the frame's dirty rectangle and clear it. Returns
            /// `false` if nothing was dirty.
            pub $($async)? fn flush_dirty<B: AsRef<[u8]>>(
                &mut self,
                fb: &mut PageFrame<B>,
            ) -> Result<bool, DisplayError<DI::Error>> {
                let Some(area) = fb.dirty() else {
                    return Ok(false);
                };
                self.flush_area(fb, area) $(.$await)??;
                fb.clear_dirty();
                Ok(true)
            }

            /// Contrast `0x00..=0xFF` (`0x81`).
            pub $($async)? fn set_contrast(&mut self, contrast: u8) -> Result<(), DisplayError<DI::Error>> {
                self.contrast = contrast;
                self.cmd(&[0x81, contrast]) $(.$await)?
            }

            /// Inverse video (`0xA7`) or normal (`0xA6`).
            pub $($async)? fn set_invert(&mut self, invert: bool) -> Result<(), DisplayError<DI::Error>> {
                self.cmd(&[if invert { 0xA7 } else { 0xA6 }]) $(.$await)?
            }

            /// Display off and charge pump / DC-DC off. RAM is kept.
            pub $($async)? fn sleep(&mut self) -> Result<(), DisplayError<DI::Error>> {
                let seq: &[u8] = match self.panel.controller {
                    Controller::Ssd1306 => &[0xAE, 0x8D, 0x10],
                    Controller::Sh1106 => &[0xAE, 0xAD, 0x8A],
                };
                self.cmd(seq) $(.$await)?
            }

            /// Charge pump / DC-DC on, display on. The panel reaches full
            /// brightness about 100 ms later (no wait here).
            pub $($async)? fn wake(&mut self) -> Result<(), DisplayError<DI::Error>> {
                let seq: &[u8] = match self.panel.controller {
                    Controller::Ssd1306 => &[0x8D, 0x14, 0xAF],
                    Controller::Sh1106 => &[0xAD, 0x8B, 0xAF],
                };
                self.cmd(seq) $(.$await)?
            }
        }
    };
}

driver!(zigbee_display::Interface, embedded_hal::delay::DelayNs,);

impl<DI: zigbee_display::Interface, RST: OutputPin> zigbee_display::blocking::Display
    for Ssd1306<DI, RST>
{
    type BusError = DI::Error;

    fn init<D: embedded_hal::delay::DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<(), DisplayError<DI::Error>> {
        Ssd1306::init(self, delay)
    }

    /// OLED RAM is shown immediately.
    fn start_refresh(&mut self) -> Result<WaitUs, DisplayError<DI::Error>> {
        Ok(0)
    }

    fn is_busy(&mut self) -> Result<bool, DisplayError<DI::Error>> {
        Ok(false)
    }

    fn sleep(&mut self) -> Result<(), DisplayError<DI::Error>> {
        Ssd1306::sleep(self)
    }
}

/// Async driver over [`zigbee_display::AsyncInterface`].
pub mod asynch {
    use super::*;

    driver!(zigbee_display::AsyncInterface, embedded_hal_async::delay::DelayNs, async await);

    impl<DI: zigbee_display::AsyncInterface, RST: OutputPin> zigbee_display::asynch::Display
        for Ssd1306<DI, RST>
    {
        type BusError = DI::Error;

        async fn init<D: embedded_hal_async::delay::DelayNs>(
            &mut self,
            delay: &mut D,
        ) -> Result<(), DisplayError<DI::Error>> {
            Ssd1306::init(self, delay).await
        }

        async fn start_refresh(&mut self) -> Result<WaitUs, DisplayError<DI::Error>> {
            Ok(0)
        }

        async fn is_busy(&mut self) -> Result<bool, DisplayError<DI::Error>> {
            Ok(false)
        }

        async fn sleep(&mut self) -> Result<(), DisplayError<DI::Error>> {
            Ssd1306::sleep(self).await
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_graphics_core::pixelcolor::BinaryColor;
    use embedded_hal_mock::eh1::delay::NoopDelay;
    use embedded_hal_mock::eh1::digital::{Mock as PinMock, State, Transaction as PinTx};
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTx};
    use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiTx};
    use std::vec;
    use std::vec::Vec;

    fn cmd(bytes: &[u8]) -> I2cTx {
        let mut v = vec![0x00];
        v.extend_from_slice(bytes);
        I2cTx::write(ADDRESS, v)
    }

    fn data(bytes: &[u8]) -> Vec<I2cTx> {
        bytes
            .chunks(zigbee_display::interface::I2C_CHUNK)
            .map(|c| {
                let mut v = vec![0x40];
                v.extend_from_slice(c);
                I2cTx::write(ADDRESS, v)
            })
            .collect()
    }

    #[test]
    fn ssd1306_128x32_init_sequence() {
        let mut tx = vec![cmd(&[
            0xAE, 0xD5, 0x80, 0xA8, 0x1F, 0xD3, 0x00, 0x40, 0x8D, 0x14, 0x20, 0x02, 0xA1, 0xC8,
            0xDA, 0x02, 0x81, 0x7F, 0xD9, 0xF1, 0xDB, 0x20, 0xA4, 0xA6, 0x2E,
        ])];
        for page in 0..4u8 {
            tx.push(cmd(&[0xB0 | page, 0x00, 0x10]));
            tx.extend(data(&[0u8; 128]));
        }
        tx.push(cmd(&[0xAF]));
        let mut i2c = I2cMock::new(&tx);
        let mut oled = Ssd1306::new(
            I2cInterface::new(i2c.clone(), ADDRESS),
            Panel::SSD1306_128X32,
        );
        oled.init(&mut NoopDelay).unwrap();
        i2c.done();
    }

    #[test]
    fn sh1106_init_uses_column_offset_and_dcdc() {
        let mut tx = vec![cmd(&[
            0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0xAD, 0x8B, 0xA0, 0xC0, 0xDA, 0x12,
            0x81, 0x20, 0xD9, 0x22, 0xDB, 0x35, 0xA4, 0xA6,
        ])];
        for page in 0..8u8 {
            tx.push(cmd(&[0xB0 | page, 0x02, 0x10]));
            tx.extend(data(&[0u8; 128]));
        }
        tx.push(cmd(&[0xAF]));
        let mut i2c = I2cMock::new(&tx);
        let mut rst = PinMock::new(&[PinTx::set(State::Low), PinTx::set(State::High)]);
        let mut oled = Ssd1306::new(
            I2cInterface::new(i2c.clone(), ADDRESS),
            Panel::SH1106_128X64,
        )
        .with_reset(rst.clone())
        .with_contrast(0x20)
        .with_rotate180(true);
        oled.init(&mut NoopDelay).unwrap();
        i2c.done();
        rst.done();
    }

    fn spi_ops(ops: &[(bool, &[u8])]) -> (Vec<SpiTx<u8>>, Vec<PinTx>) {
        let mut spi = Vec::new();
        let mut dc = Vec::new();
        for (is_data, bytes) in ops {
            dc.push(PinTx::set(if *is_data { State::High } else { State::Low }));
            spi.push(SpiTx::transaction_start());
            spi.push(SpiTx::write_vec(bytes.to_vec()));
            spi.push(SpiTx::transaction_end());
        }
        (spi, dc)
    }

    #[test]
    fn flush_dirty_over_spi_writes_only_touched_pages_and_columns() {
        let panel = Panel::new(Controller::Sh1106, 32, 16, 2, 0x12).unwrap();
        let mut fb = PageFrame::new([0u8; 64], 32, 16).unwrap();
        fb.set_pixel(17, 3, BinaryColor::On);
        fb.set_pixel(18, 9, BinaryColor::On);
        // Columns 17..19 + offset 2 = 19 = 0x13.
        let (spi_tx, dc_tx) = spi_ops(&[
            (false, &[0xB0, 0x03, 0x11]),
            (true, &[0x08, 0x00]),
            (false, &[0xB1, 0x03, 0x11]),
            (true, &[0x00, 0x02]),
        ]);
        let mut spi = SpiMock::new(&spi_tx);
        let mut dc = PinMock::new(&dc_tx);
        let mut oled = Ssd1306::new(SpiInterface::new(spi.clone(), dc.clone()), panel);
        assert!(oled.flush_dirty(&mut fb).unwrap());
        assert!(!oled.flush_dirty(&mut fb).unwrap());
        spi.done();
        dc.done();
    }

    #[test]
    fn sleep_and_wake() {
        let tx = [
            cmd(&[0xAE, 0x8D, 0x10]),
            cmd(&[0x8D, 0x14, 0xAF]),
            cmd(&[0xA7]),
        ];
        let mut i2c = I2cMock::new(&tx);
        let mut oled = Ssd1306::new(
            I2cInterface::new(i2c.clone(), ADDRESS),
            Panel::SSD1306_128X64,
        );
        oled.sleep().unwrap();
        oled.wake().unwrap();
        oled.set_invert(true).unwrap();
        i2c.done();
    }

    #[test]
    fn panel_validation_and_ram() {
        assert_eq!(Panel::SSD1306_128X64.frame_len(), 1024);
        assert_eq!(Panel::SSD1306_128X32.frame_len(), 512);
        assert!(Panel::new(Controller::Ssd1306, 128, 64, 2, 0x12).is_none());
        assert!(Panel::new(Controller::Sh1106, 128, 64, 4, 0x12).is_some());
        assert!(Panel::new(Controller::Ssd1306, 64, 20, 0, 0x12).is_none());
        let mut i2c = I2cMock::new(&[]);
        let fb = PageFrame::new([0u8; 512], 128, 32).unwrap();
        let mut oled = Ssd1306::new(
            I2cInterface::new(i2c.clone(), ADDRESS),
            Panel::SSD1306_128X64,
        );
        assert_eq!(oled.flush(&fb), Err(Error::InvalidArgument));
        i2c.done();
    }

    #[test]
    fn async_flush_over_i2c() {
        let panel = Panel::new(Controller::Ssd1306, 4, 8, 0, 0x02).unwrap();
        let mut fb = PageFrame::new([0u8; 4], 4, 8).unwrap();
        fb.set_pixel(3, 7, BinaryColor::On);
        let mut tx = vec![cmd(&[0xB0, 0x00, 0x10])];
        tx.extend(data(&[0, 0, 0, 0x80]));
        let mut i2c = I2cMock::new(&tx);
        let mut oled = asynch::Ssd1306::new(I2cInterface::new(i2c.clone(), ADDRESS), panel);
        block_on(oled.flush(&fb)).unwrap();
        i2c.done();
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
