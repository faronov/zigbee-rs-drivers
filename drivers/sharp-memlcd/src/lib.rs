//! Sharp Memory LCD (LS013B7DH03, LS027B7DH01 and siblings).
//!
//! Each pixel stores its own value (memory-in-pixel), so a static image
//! costs a few µW and needs no refresh. The panel needs only:
//!
//! * a write-only SPI bus, **MSB first**, mode 0, ≤ 1.1 MHz
//!   (LS013B7DH03; LS027B7DH01 allows 2 MHz). CS is **active high**, so the
//!   driver owns a raw [`SpiBus`](embedded_hal::spi::SpiBus) plus the CS pin
//!   and times CS itself (setup/hold delays) instead of using `SpiDevice`;
//! * VCOM inversion at ≥ 1 Hz to avoid DC bias across the liquid crystal.
//!   Either set the M1 bit in software ([`Vcom::Software`], EXTMODE pin
//!   tied low) or drive EXTCOMIN ([`Vcom::ExtComIn`], EXTMODE tied high)
//!   from a GPIO through [`SharpMemLcd::toggle_vcom`] or from a hardware
//!   timer/PRS (Silicon Labs WSTK default: 60 Hz on PA0).
//!
//! Protocol (LS013B7DH03 datasheet, "Data update mode", sent MSB first):
//!
//! | Byte | Value |
//! |---|---|
//! | mode | `0x80` write, `0x20` clear-all, `0x00` display; `0x40` = VCOM (M1) |
//! | per line | gate address (1-based, bit-reversed because the panel takes it LSB first), `width / 8` data bytes (1 = white), `0x00` dummy |
//! | trailer | `0x00` |
//!
//! Pixel data matches [`MonoFrame`] (MSB = leftmost, 1 = white/paper), so
//! lines are sent without conversion.
//!
//! ```ignore
//! let mut buf = [0xFFu8; mono_len(128, 128)];
//! let mut fb = MonoFrame::new(&mut buf[..], 128, 128).unwrap();
//! let mut lcd = SharpMemLcd::new(spi, cs, delay, Panel::LS013B7DH03)
//!     .with_hardware_extcomin();
//! lcd.init()?;
//! Text::new("21.5 °C", Point::new(4, 20), style).draw(&mut fb)?;
//! lcd.flush_dirty(&mut fb)?; // only the changed lines
//! ```

#![no_std]

use embedded_hal::digital::OutputPin;
use zigbee_display::{BusError, DisplayError, Error, MonoFrame, NoPin, WaitUs, pin_error};

pub use zigbee_display;

/// Mode byte: write lines (M0).
pub const CMD_WRITE: u8 = 0x80;
/// Mode byte: VCOM bit (M1).
pub const CMD_VCOM: u8 = 0x40;
/// Mode byte: clear all pixels to white (M2).
pub const CMD_CLEAR: u8 = 0x20;

/// Widest supported line in bytes (512 pixels).
pub const MAX_STRIDE: usize = 64;

/// Panel geometry. Every panel listed here uses the 8-bit gate address
/// protocol; LS032B7DD02 (10-bit addresses) is not supported.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Panel {
    width: u16,
    height: u16,
}

impl Panel {
    /// 1.33" 128x128 (Silicon Labs WSTK BRD4001A, Adafruit 1393).
    pub const LS013B7DH03: Panel = Panel::raw(128, 128);
    /// 1.28" 144x168.
    pub const LS013B7DH05: Panel = Panel::raw(144, 168);
    /// 1.08" 160x68.
    pub const LS011B7DH03: Panel = Panel::raw(160, 68);
    /// 2.7" 400x240 (also LS027B7DH01A).
    pub const LS027B7DH01: Panel = Panel::raw(400, 240);

    const fn raw(width: u16, height: u16) -> Self {
        Self { width, height }
    }

    /// Custom geometry. `None` unless `width` is a multiple of 8 and at
    /// most 512, and `height` is 1..=255 (8-bit gate address).
    pub const fn new(width: u16, height: u16) -> Option<Self> {
        if width == 0 || width % 8 != 0 || width as usize > MAX_STRIDE * 8 {
            return None;
        }
        if height == 0 || height > 255 {
            return None;
        }
        Some(Self::raw(width, height))
    }

    /// Width in pixels.
    pub const fn width(&self) -> u16 {
        self.width
    }

    /// Height in pixels (lines).
    pub const fn height(&self) -> u16 {
        self.height
    }

    /// Bytes per line.
    pub const fn stride(&self) -> usize {
        self.width as usize / 8
    }

    /// Frame buffer bytes ([`zigbee_display::mono_len`]).
    pub const fn frame_len(&self) -> usize {
        self.stride() * self.height as usize
    }
}

/// How VCOM inversion is produced.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Vcom {
    /// M1 bit in the mode byte (EXTMODE pin low). Call
    /// [`SharpMemLcd::toggle_vcom`] at 1–10 Hz.
    #[default]
    Software,
    /// EXTCOMIN pin (EXTMODE pin high). Toggle it with
    /// [`SharpMemLcd::toggle_vcom`] (each call is one edge) or from a
    /// hardware timer; the M1 bit is sent as 0.
    ExtComIn,
}

/// CS timing in microseconds (LS013B7DH03: tsSCS 6 µs, thSCS 2 µs).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Timing {
    /// CS high to first SCLK edge.
    pub setup_us: u32,
    /// Last SCLK edge to CS low.
    pub hold_us: u32,
}

impl Default for Timing {
    fn default() -> Self {
        Self {
            setup_us: 6,
            hold_us: 2,
        }
    }
}

const fn line_address(y: u16) -> u8 {
    // 1-based gate line, transmitted LSB first.
    ((y + 1) as u8).reverse_bits()
}

fn spi_error<E>(error: E) -> DisplayError<E> {
    Error::Bus(BusError::Bus(error))
}

macro_rules! driver {
    ($SpiBus:path, $Delay:path, $($async:tt $await:tt)?) => {
        /// Sharp memory LCD driver. Owns the SPI bus, the active-high CS pin,
        /// a delay for CS timing, and optional EXTCOMIN and DISP pins.
        pub struct SharpMemLcd<SPI, CS, D, EXT = NoPin, DISP = NoPin> {
            spi: SPI,
            cs: CS,
            delay: D,
            extcomin: Option<EXT>,
            disp: Option<DISP>,
            panel: Panel,
            mode: Vcom,
            vcom: bool,
            timing: Timing,
        }

        impl<SPI, CS, D> SharpMemLcd<SPI, CS, D>
        where
            SPI: $SpiBus,
            CS: OutputPin,
            D: $Delay,
        {
            /// Driver with software VCOM and no EXTCOMIN/DISP pins.
            pub fn new(spi: SPI, cs: CS, delay: D, panel: Panel) -> Self {
                Self {
                    spi,
                    cs,
                    delay,
                    extcomin: None,
                    disp: None,
                    panel,
                    mode: Vcom::Software,
                    vcom: false,
                    timing: Timing::default(),
                }
            }
        }

        impl<SPI, CS, D, EXT, DISP> SharpMemLcd<SPI, CS, D, EXT, DISP>
        where
            SPI: $SpiBus,
            CS: OutputPin,
            D: $Delay,
            EXT: OutputPin,
            DISP: OutputPin,
        {
            /// Use the EXTCOMIN pin, toggled by [`Self::toggle_vcom`].
            pub fn with_extcomin<P: OutputPin>(
                self,
                pin: P,
            ) -> SharpMemLcd<SPI, CS, D, P, DISP> {
                SharpMemLcd {
                    spi: self.spi,
                    cs: self.cs,
                    delay: self.delay,
                    extcomin: Some(pin),
                    disp: self.disp,
                    panel: self.panel,
                    mode: Vcom::ExtComIn,
                    vcom: false,
                    timing: self.timing,
                }
            }

            /// EXTCOMIN is driven by hardware (timer/PRS); the driver sends
            /// M1 = 0 and [`Self::toggle_vcom`] does nothing.
            pub fn with_hardware_extcomin(mut self) -> Self {
                self.mode = Vcom::ExtComIn;
                self
            }

            /// Use a DISP (display on/off) pin; [`Self::init`] drives it high.
            pub fn with_disp<P: OutputPin>(self, pin: P) -> SharpMemLcd<SPI, CS, D, EXT, P> {
                SharpMemLcd {
                    spi: self.spi,
                    cs: self.cs,
                    delay: self.delay,
                    extcomin: self.extcomin,
                    disp: Some(pin),
                    panel: self.panel,
                    mode: self.mode,
                    vcom: self.vcom,
                    timing: self.timing,
                }
            }

            /// Override the CS setup/hold timing.
            pub fn with_timing(mut self, timing: Timing) -> Self {
                self.timing = timing;
                self
            }

            /// Panel geometry.
            pub const fn panel(&self) -> Panel {
                self.panel
            }

            /// VCOM mode.
            pub const fn vcom_mode(&self) -> Vcom {
                self.mode
            }

            /// Give the peripherals back.
            pub fn release(self) -> (SPI, CS, D, Option<EXT>, Option<DISP>) {
                (self.spi, self.cs, self.delay, self.extcomin, self.disp)
            }

            fn mode_bits(&self) -> u8 {
                if self.mode == Vcom::Software && self.vcom {
                    CMD_VCOM
                } else {
                    0
                }
            }

            /// CS low, EXTCOMIN low, DISP high, then clear the panel memory.
            pub $($async)? fn init(&mut self) -> Result<(), DisplayError<SPI::Error>> {
                self.cs.set_low().map_err(pin_error)?;
                if let Some(pin) = self.extcomin.as_mut() {
                    pin.set_low().map_err(pin_error)?;
                }
                self.set_display_on(true)?;
                self.clear_all() $(.$await)?
            }

            /// Drive the DISP pin (no-op without one). Off blanks the
            /// panel but keeps its memory.
            pub fn set_display_on(&mut self, on: bool) -> Result<(), DisplayError<SPI::Error>> {
                if let Some(pin) = self.disp.as_mut() {
                    pin.set_state(on.into()).map_err(pin_error)?;
                }
                Ok(())
            }

            $($async)? fn frame(&mut self, bytes: &[u8]) -> Result<(), DisplayError<SPI::Error>> {
                self.cs.set_high().map_err(pin_error)?;
                self.delay.delay_us(self.timing.setup_us) $(.$await)?;
                let written = self.spi.write(bytes) $(.$await)?;
                let flushed = self.spi.flush() $(.$await)?;
                self.delay.delay_us(self.timing.hold_us) $(.$await)?;
                self.cs.set_low().map_err(pin_error)?;
                written.and(flushed).map_err(spi_error)
            }

            /// Clear the whole panel to white (does not touch the frame).
            pub $($async)? fn clear_all(&mut self) -> Result<(), DisplayError<SPI::Error>> {
                let cmd = [CMD_CLEAR | self.mode_bits(), 0x00];
                self.frame(&cmd) $(.$await)?
            }

            /// Invert VCOM: software mode flips M1 and sends a display
            /// command; EXTCOMIN mode toggles the pin (one edge per call; a
            /// rising edge inverts VCOM). Call at least once per second
            /// (software) or at 2x the EXTCOMIN frequency.
            pub $($async)? fn toggle_vcom(&mut self) -> Result<(), DisplayError<SPI::Error>> {
                self.vcom = !self.vcom;
                match self.mode {
                    Vcom::Software => {
                        let cmd = [self.mode_bits(), 0x00];
                        self.frame(&cmd) $(.$await)?
                    }
                    Vcom::ExtComIn => {
                        if let Some(pin) = self.extcomin.as_mut() {
                            pin.set_state(self.vcom.into()).map_err(pin_error)?;
                        }
                        Ok(())
                    }
                }
            }

            /// Write lines `first .. first + count` of `fb`.
            pub $($async)? fn flush_rows<B: AsRef<[u8]>>(
                &mut self,
                fb: &MonoFrame<B>,
                first: u16,
                count: u16,
            ) -> Result<(), DisplayError<SPI::Error>> {
                if fb.width() != self.panel.width || fb.height() != self.panel.height {
                    return Err(Error::InvalidArgument);
                }
                let end = first.checked_add(count).ok_or(Error::InvalidArgument)?;
                if end > self.panel.height {
                    return Err(Error::InvalidArgument);
                }
                if count == 0 {
                    return Ok(());
                }
                let stride = self.panel.stride();
                let mode = [CMD_WRITE | self.mode_bits()];
                self.cs.set_high().map_err(pin_error)?;
                self.delay.delay_us(self.timing.setup_us) $(.$await)?;
                let mut result = self.spi.write(&mode) $(.$await)?;
                let mut line = [0u8; MAX_STRIDE + 2];
                for y in first..end {
                    if result.is_err() {
                        break;
                    }
                    line[0] = line_address(y);
                    line[1..=stride].copy_from_slice(fb.row(y));
                    line[stride + 1] = 0x00;
                    result = self.spi.write(&line[..stride + 2]) $(.$await)?;
                }
                if result.is_ok() {
                    result = self.spi.write(&[0x00]) $(.$await)?;
                }
                let flushed = self.spi.flush() $(.$await)?;
                self.delay.delay_us(self.timing.hold_us) $(.$await)?;
                self.cs.set_low().map_err(pin_error)?;
                result.and(flushed).map_err(spi_error)
            }

            /// Write the whole frame.
            pub $($async)? fn flush<B: AsRef<[u8]>>(
                &mut self,
                fb: &MonoFrame<B>,
            ) -> Result<(), DisplayError<SPI::Error>> {
                let height = self.panel.height;
                self.flush_rows(fb, 0, height) $(.$await)?
            }

            /// Write the lines covered by the frame's dirty rectangle and
            /// clear it. Returns `false` if nothing was dirty.
            pub $($async)? fn flush_dirty<B: AsRef<[u8]>>(
                &mut self,
                fb: &mut MonoFrame<B>,
            ) -> Result<bool, DisplayError<SPI::Error>> {
                let Some(area) = fb.dirty() else {
                    return Ok(false);
                };
                self.flush_rows(fb, area.y, area.height) $(.$await)??;
                fb.clear_dirty();
                Ok(true)
            }
        }
    };
}

driver!(embedded_hal::spi::SpiBus, embedded_hal::delay::DelayNs,);

impl<SPI, CS, D, EXT, DISP> zigbee_display::blocking::Display for SharpMemLcd<SPI, CS, D, EXT, DISP>
where
    SPI: embedded_hal::spi::SpiBus,
    CS: OutputPin,
    D: embedded_hal::delay::DelayNs,
    EXT: OutputPin,
    DISP: OutputPin,
{
    type BusError = SPI::Error;

    /// Same as [`SharpMemLcd::init`]; the driver uses its own delay.
    fn init<DL: embedded_hal::delay::DelayNs>(
        &mut self,
        _delay: &mut DL,
    ) -> Result<(), DisplayError<SPI::Error>> {
        SharpMemLcd::init(self)
    }

    /// Lines are shown as soon as they are written.
    fn start_refresh(&mut self) -> Result<WaitUs, DisplayError<SPI::Error>> {
        Ok(0)
    }

    fn is_busy(&mut self) -> Result<bool, DisplayError<SPI::Error>> {
        Ok(false)
    }

    /// A static memory LCD already draws only µW; nothing to do. Keep
    /// toggling VCOM. Use [`SharpMemLcd::set_display_on`] to blank it.
    fn sleep(&mut self) -> Result<(), DisplayError<SPI::Error>> {
        Ok(())
    }
}

/// Async driver (embedded-hal-async `SpiBus` and `DelayNs`).
pub mod asynch {
    use super::*;

    driver!(embedded_hal_async::spi::SpiBus, embedded_hal_async::delay::DelayNs, async await);

    impl<SPI, CS, D, EXT, DISP> zigbee_display::asynch::Display for SharpMemLcd<SPI, CS, D, EXT, DISP>
    where
        SPI: embedded_hal_async::spi::SpiBus,
        CS: OutputPin,
        D: embedded_hal_async::delay::DelayNs,
        EXT: OutputPin,
        DISP: OutputPin,
    {
        type BusError = SPI::Error;

        async fn init<DL: embedded_hal_async::delay::DelayNs>(
            &mut self,
            _delay: &mut DL,
        ) -> Result<(), DisplayError<SPI::Error>> {
            SharpMemLcd::init(self).await
        }

        async fn start_refresh(&mut self) -> Result<WaitUs, DisplayError<SPI::Error>> {
            Ok(0)
        }

        async fn is_busy(&mut self) -> Result<bool, DisplayError<SPI::Error>> {
            Ok(false)
        }

        async fn sleep(&mut self) -> Result<(), DisplayError<SPI::Error>> {
            Ok(())
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
    use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiTx};
    use std::vec;
    use std::vec::Vec;
    use zigbee_display::mono_len;

    fn cs_frame(n: usize) -> Vec<PinTx> {
        let mut v = Vec::new();
        for _ in 0..n {
            v.push(PinTx::set(State::High));
            v.push(PinTx::set(State::Low));
        }
        v
    }

    #[test]
    fn panel_presets_and_ram() {
        assert_eq!(Panel::LS013B7DH03.frame_len(), 2048);
        assert_eq!(Panel::LS027B7DH01.frame_len(), 12_000);
        assert_eq!(Panel::LS013B7DH05.frame_len(), mono_len(144, 168));
        assert!(Panel::new(100, 10).is_none());
        assert!(Panel::new(64, 256).is_none());
        assert_eq!(Panel::new(64, 64).map(|p| p.stride()), Some(8));
    }

    #[test]
    fn line_address_is_bit_reversed_one_based() {
        assert_eq!(line_address(0), 0x80);
        assert_eq!(line_address(1), 0x40);
        assert_eq!(line_address(2), 0xC0);
        assert_eq!(line_address(127), 0x01);
        assert_eq!(line_address(239), 0x0F);
    }

    #[test]
    fn init_clears_with_cs_active_high() {
        let spi_tx = [SpiTx::write_vec(vec![0x20, 0x00]), SpiTx::flush()];
        let mut cs_tx = vec![PinTx::set(State::Low)];
        cs_tx.extend(cs_frame(1));
        let disp_tx = [PinTx::set(State::High)];
        let mut spi = SpiMock::new(&spi_tx);
        let mut cs = PinMock::new(&cs_tx);
        let mut disp = PinMock::new(&disp_tx);
        let mut lcd = SharpMemLcd::new(spi.clone(), cs.clone(), NoopDelay, Panel::LS013B7DH03)
            .with_disp(disp.clone());
        lcd.init().unwrap();
        spi.done();
        cs.done();
        disp.done();
    }

    #[test]
    fn software_vcom_toggles_m1() {
        let spi_tx = [
            SpiTx::write_vec(vec![0x40, 0x00]),
            SpiTx::flush(),
            SpiTx::write_vec(vec![0x00, 0x00]),
            SpiTx::flush(),
            SpiTx::write_vec(vec![0x20, 0x00]),
            SpiTx::flush(),
        ];
        let mut spi = SpiMock::new(&spi_tx);
        let mut cs = PinMock::new(&cs_frame(3));
        let mut lcd = SharpMemLcd::new(spi.clone(), cs.clone(), NoopDelay, Panel::LS013B7DH03);
        lcd.toggle_vcom().unwrap();
        lcd.toggle_vcom().unwrap();
        lcd.clear_all().unwrap();
        spi.done();
        cs.done();
    }

    #[test]
    fn extcomin_pin_mode_toggles_pin_and_sends_m1_zero() {
        let spi_tx = [SpiTx::write_vec(vec![0x20, 0x00]), SpiTx::flush()];
        let ext_tx = [PinTx::set(State::High), PinTx::set(State::Low)];
        let mut spi = SpiMock::new(&spi_tx);
        let mut cs = PinMock::new(&cs_frame(1));
        let mut ext = PinMock::new(&ext_tx);
        let mut lcd = SharpMemLcd::new(spi.clone(), cs.clone(), NoopDelay, Panel::LS013B7DH03)
            .with_extcomin(ext.clone());
        assert_eq!(lcd.vcom_mode(), Vcom::ExtComIn);
        lcd.toggle_vcom().unwrap();
        // VCOM high, yet the M1 bit stays 0 in EXTCOMIN mode.
        lcd.clear_all().unwrap();
        lcd.toggle_vcom().unwrap();
        spi.done();
        cs.done();
        ext.done();
    }

    #[test]
    fn flush_dirty_writes_only_changed_lines() {
        let panel = Panel::new(16, 4).unwrap();
        let mut fb = MonoFrame::new([0xFFu8; 8], 16, 4).unwrap();
        fb.set_pixel(0, 1, BinaryColor::On);
        fb.set_pixel(15, 2, BinaryColor::On);
        let spi_tx = [
            SpiTx::write_vec(vec![0x80]),
            SpiTx::write_vec(vec![line_address(1), 0x7F, 0xFF, 0x00]),
            SpiTx::write_vec(vec![line_address(2), 0xFF, 0xFE, 0x00]),
            SpiTx::write_vec(vec![0x00]),
            SpiTx::flush(),
        ];
        let mut spi = SpiMock::new(&spi_tx);
        let mut cs = PinMock::new(&cs_frame(1));
        let mut lcd = SharpMemLcd::new(spi.clone(), cs.clone(), NoopDelay, panel);
        assert!(lcd.flush_dirty(&mut fb).unwrap());
        assert!(fb.dirty().is_none());
        assert!(!lcd.flush_dirty(&mut fb).unwrap());
        spi.done();
        cs.done();
    }

    #[test]
    fn flush_rejects_wrong_frame_size() {
        let mut spi = SpiMock::new(&[]);
        let mut cs = PinMock::new(&[]);
        let fb = MonoFrame::new([0xFFu8; 8], 16, 4).unwrap();
        let mut lcd = SharpMemLcd::new(spi.clone(), cs.clone(), NoopDelay, Panel::LS013B7DH03);
        assert_eq!(lcd.flush(&fb), Err(Error::InvalidArgument));
        let mut lcd2 = SharpMemLcd::new(
            spi.clone(),
            cs.clone(),
            NoopDelay,
            Panel::new(16, 4).unwrap(),
        );
        assert_eq!(lcd2.flush_rows(&fb, 3, 2), Err(Error::InvalidArgument));
        spi.done();
        cs.done();
    }

    #[test]
    fn async_full_flush() {
        let panel = Panel::new(8, 2).unwrap();
        let fb = MonoFrame::new([0xAAu8, 0x55], 8, 2).unwrap();
        let spi_tx = [
            SpiTx::write_vec(vec![0x40, 0x00]),
            SpiTx::flush(),
            SpiTx::write_vec(vec![0x80 | 0x40]),
            SpiTx::write_vec(vec![0x80, 0xAA, 0x00]),
            SpiTx::write_vec(vec![0x40, 0x55, 0x00]),
            SpiTx::write_vec(vec![0x00]),
            SpiTx::flush(),
            // toggle back
            SpiTx::write_vec(vec![0x00, 0x00]),
            SpiTx::flush(),
        ];
        let mut spi = SpiMock::new(&spi_tx);
        let mut cs = PinMock::new(&cs_frame(3));
        let mut lcd = asynch::SharpMemLcd::new(spi.clone(), cs.clone(), NoopDelay, panel);
        block_on(lcd.toggle_vcom()).unwrap();
        block_on(lcd.flush(&fb)).unwrap();
        block_on(lcd.toggle_vcom()).unwrap();
        spi.done();
        cs.done();
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
