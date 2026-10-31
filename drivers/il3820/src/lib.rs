//! Good Display / Waveshare **IL3820** (2.9" 128x296, Waveshare 2.9" V1),
//! **IL3895** (2.13" 122x250, Waveshare 2.13" V1) and **IL3829** (1.54"
//! 200x200 V1) black/white e-paper controllers: the older "type A"
//! generation that predates the SSD1675/SSD1680.
//!
//! * 4-wire SPI, D/C low = command, **BUSY high = busy**.
//! * No OTP waveform: the driver always loads a 30-byte register LUT
//!   (`0x32`). The full and partial LUTs bundled here are the Waveshare
//!   waveforms as published in the `epd-waveshare` crate (ISC licence, see
//!   NOTICE). Override them with [`Il3820::with_luts`].
//! * One write command (`0x24`), but **two internal frame buffers that
//!   swap after every refresh**. To keep the next partial refresh correct
//!   the same data must be written again after the refresh:
//!   [`Il3820::finish_refresh`] does this for the window of the last
//!   refresh (full or partial).
//!
//! ```ignore
//! epd.init(&mut delay)?;
//! let wait = epd.update(&fb)?;            // write + full refresh
//! // ... after `wait` and !epd.is_busy()?:
//! epd.finish_refresh(&fb)?;               // sync the second buffer
//! fb.set_pixel(...);
//! if let Some(wait) = epd.start_partial_refresh_dirty(&mut fb)? {
//!     // ... after `wait` and !is_busy():
//!     epd.finish_refresh(&fb)?;
//! }
//! epd.sleep()?;
//! ```
//!
//! References: IL3820 and IL3895 datasheets (command table, RAM
//! addressing); command sequence cross-checked with `epd-waveshare`
//! `epd2in9` (ISC).

#![no_std]

use embedded_hal::digital::{InputPin, OutputPin};
use zigbee_display::frame::window_rows;
use zigbee_display::{Area, DisplayError, Error, MonoFrame, NoPin, WaitUs, pin_error};

pub use zigbee_display;
pub use zigbee_display::SpiInterface;

/// Command bytes.
pub mod cmd {
    /// Driver output control (gate count, scan).
    pub const DRIVER_OUTPUT: u8 = 0x01;
    /// Booster soft start control.
    pub const BOOSTER_SOFT_START: u8 = 0x0C;
    /// Deep sleep mode.
    pub const DEEP_SLEEP: u8 = 0x10;
    /// Data entry mode.
    pub const DATA_ENTRY: u8 = 0x11;
    /// Master activation.
    pub const MASTER_ACTIVATION: u8 = 0x20;
    /// Display update control 2.
    pub const UPDATE_CONTROL_2: u8 = 0x22;
    /// Write RAM.
    pub const WRITE_RAM: u8 = 0x24;
    /// Write VCOM register.
    pub const VCOM: u8 = 0x2C;
    /// Write LUT register (30 bytes).
    pub const WRITE_LUT: u8 = 0x32;
    /// Dummy line period.
    pub const DUMMY_LINE: u8 = 0x3A;
    /// Gate line width.
    pub const GATE_LINE_WIDTH: u8 = 0x3B;
    /// RAM X start/end (bytes).
    pub const RAM_X_RANGE: u8 = 0x44;
    /// RAM Y start/end.
    pub const RAM_Y_RANGE: u8 = 0x45;
    /// RAM X counter.
    pub const RAM_X: u8 = 0x4E;
    /// RAM Y counter.
    pub const RAM_Y: u8 = 0x4F;
    /// No-operation; terminates the master activation write.
    pub const NOP: u8 = 0xFF;
}

/// Display update control 2 value: clock on, charge pump on, display
/// pattern (`0xC4`, as used by Waveshare). The datasheet's `0xC7` also
/// switches the analog block off again after the update.
pub const UPDATE_SEQUENCE: u8 = 0xC4;

/// HW reset pulse and wait.
pub const RESET_US: u32 = 10_000;
/// Longest BUSY wait inside `init`.
pub const INIT_BUSY_TIMEOUT_US: u32 = 500_000;
const INIT_POLL_US: u32 = 1_000;

/// Waveshare full-update waveform for IL3820/IL3829 panels (from
/// `epd-waveshare` `type_a::constants::LUT_FULL_UPDATE`, ISC licence,
/// Copyright (c) 2018 Christoph Groß; see NOTICE).
#[rustfmt::skip]
pub const LUT_FULL: [u8; 30] = [
    0x02, 0x02, 0x01, 0x11, 0x12, 0x12, 0x22, 0x22,
    0x66, 0x69, 0x69, 0x59, 0x58, 0x99, 0x99, 0x88,
    0x00, 0x00, 0x00, 0x00, 0xF8, 0xB4, 0x13, 0x51,
    0x35, 0x51, 0x51, 0x19, 0x01, 0x00,
];

/// Waveshare partial-update waveform (from `epd-waveshare`
/// `type_a::constants::LUT_PARTIAL_UPDATE`, ISC licence, Copyright (c)
/// 2018 Christoph Groß; see NOTICE).
#[rustfmt::skip]
pub const LUT_PARTIAL: [u8; 30] = [
    0x10, 0x18, 0x18, 0x08, 0x18, 0x18, 0x08, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x13, 0x14, 0x44, 0x12,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
];

/// Full and partial register LUTs (30 bytes each on IL3820/IL3895).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Luts {
    /// Full-update waveform.
    pub full: &'static [u8],
    /// Partial-update waveform.
    pub partial: &'static [u8],
}

impl Luts {
    /// The bundled Waveshare waveforms ([`LUT_FULL`], [`LUT_PARTIAL`]).
    pub const WAVESHARE: Luts = Luts {
        full: &LUT_FULL,
        partial: &LUT_PARTIAL,
    };
}

/// Panel description.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Panel {
    width: u16,
    height: u16,
    full_us: WaitUs,
    partial_us: WaitUs,
}

impl Panel {
    /// 2.9" 128x296 (IL3820, Waveshare 2.9" V1).
    pub const IL3820_2IN9: Panel = Panel::raw(128, 296);
    /// 2.13" 122x250 (IL3895, Waveshare 2.13" V1).
    pub const IL3895_2IN13: Panel = Panel::raw(122, 250);
    /// 1.54" 200x200 (IL3829, Waveshare 1.54" V1).
    pub const IL3829_1IN54: Panel = Panel::raw(200, 200);

    const fn raw(width: u16, height: u16) -> Self {
        Self {
            width,
            height,
            full_us: 2_000_000,
            partial_us: 400_000,
        }
    }

    /// Custom geometry; `None` unless 1..=200 x 1..=320.
    pub const fn new(width: u16, height: u16) -> Option<Self> {
        if width == 0 || width > 200 || height == 0 || height > 320 {
            return None;
        }
        Some(Self::raw(width, height))
    }

    /// Override the typical refresh times returned as wait hints.
    pub const fn with_times(mut self, full_us: WaitUs, partial_us: WaitUs) -> Self {
        self.full_us = full_us;
        self.partial_us = partial_us;
        self
    }

    /// Width in pixels.
    pub const fn width(&self) -> u16 {
        self.width
    }

    /// Height in pixels.
    pub const fn height(&self) -> u16 {
        self.height
    }

    /// Typical full-refresh time (wait hint; BUSY is authoritative).
    pub const fn full_refresh_us(&self) -> WaitUs {
        self.full_us
    }

    /// Typical partial-refresh time.
    pub const fn partial_refresh_us(&self) -> WaitUs {
        self.partial_us
    }

    /// Frame bytes ([`zigbee_display::mono_len`]).
    pub const fn frame_len(&self) -> usize {
        zigbee_display::mono_len(self.width, self.height)
    }

    const fn stride(&self) -> usize {
        (self.width as usize).div_ceil(8)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Lut {
    None,
    Full,
    Partial,
}

macro_rules! driver {
    ($Iface:path, $Delay:path, $($async:tt $await:tt)?) => {
        /// IL3820 / IL3895 / IL3829 driver. BUSY (active high) is
        /// required; RST is optional but needed to leave deep sleep.
        pub struct Il3820<DI, BUSY, RST = NoPin> {
            di: DI,
            busy: BUSY,
            rst: Option<RST>,
            panel: Panel,
            luts: Luts,
            update: u8,
            lut: Lut,
            written: Option<Area>,
            pending: Option<Area>,
            partials: u16,
        }

        impl<DI: $Iface, BUSY: InputPin> Il3820<DI, BUSY> {
            /// Driver without a reset pin, bundled LUTs.
            pub fn new(di: DI, busy: BUSY, panel: Panel) -> Self {
                Self {
                    di,
                    busy,
                    rst: None,
                    panel,
                    luts: Luts::WAVESHARE,
                    update: UPDATE_SEQUENCE,
                    lut: Lut::None,
                    written: None,
                    pending: None,
                    partials: 0,
                }
            }
        }

        impl<DI: $Iface, BUSY: InputPin, RST: OutputPin> Il3820<DI, BUSY, RST> {
            /// Add the RST pin, pulsed by [`Self::init`].
            pub fn with_reset<P: OutputPin>(self, rst: P) -> Il3820<DI, BUSY, P> {
                Il3820 {
                    di: self.di,
                    busy: self.busy,
                    rst: Some(rst),
                    panel: self.panel,
                    luts: self.luts,
                    update: self.update,
                    lut: self.lut,
                    written: self.written,
                    pending: self.pending,
                    partials: self.partials,
                }
            }

            /// Use panel-specific waveforms instead of the bundled ones.
            pub fn with_luts(mut self, luts: Luts) -> Self {
                self.luts = luts;
                self.lut = Lut::None;
                self
            }

            /// Display update control 2 value (default
            /// [`UPDATE_SEQUENCE`] `0xC4`; `0xC7` per datasheet).
            pub fn with_update_control(mut self, value: u8) -> Self {
                self.update = value;
                self
            }

            /// Panel description.
            pub const fn panel(&self) -> Panel {
                self.panel
            }

            /// Partial refreshes since the last full refresh.
            pub const fn partial_count(&self) -> u16 {
                self.partials
            }

            /// `true` if [`Self::finish_refresh`] still has to rewrite a
            /// window into the second buffer.
            pub const fn needs_finish(&self) -> bool {
                self.pending.is_some()
            }

            /// Give the peripherals back.
            pub fn release(self) -> (DI, BUSY, Option<RST>) {
                (self.di, self.busy, self.rst)
            }

            /// BUSY level (high = refreshing).
            pub fn is_busy(&mut self) -> Result<bool, DisplayError<DI::Error>> {
                self.busy.is_high().map_err(pin_error)
            }

            fn ensure_idle(&mut self) -> Result<(), DisplayError<DI::Error>> {
                if self.is_busy()? {
                    return Err(Error::NotReady);
                }
                Ok(())
            }

            $($async)? fn cmd(&mut self, command: u8, data: &[u8]) -> Result<(), DisplayError<DI::Error>> {
                self.di.command(&[command]) $(.$await)?.map_err(Error::Bus)?;
                self.di.data(data) $(.$await)?.map_err(Error::Bus)
            }

            /// Send a raw command with parameters (BUSY must be low).
            pub $($async)? fn command(&mut self, command: u8, data: &[u8]) -> Result<(), DisplayError<DI::Error>> {
                self.ensure_idle()?;
                self.cmd(command, data) $(.$await)?
            }

            $($async)? fn wait_init<D: $Delay>(&mut self, delay: &mut D) -> Result<(), DisplayError<DI::Error>> {
                for _ in 0..INIT_BUSY_TIMEOUT_US / INIT_POLL_US {
                    if !self.is_busy()? {
                        return Ok(());
                    }
                    delay.delay_us(INIT_POLL_US) $(.$await)?;
                }
                Err(Error::Timeout)
            }

            /// Hardware reset (if RST is connected) and panel setup, then
            /// load the full LUT. Blocks for ~20 ms of reset plus short
            /// BUSY polls. Also wakes the controller from deep sleep.
            pub $($async)? fn init<D: $Delay>(&mut self, delay: &mut D) -> Result<(), DisplayError<DI::Error>> {
                if let Some(rst) = self.rst.as_mut() {
                    rst.set_low().map_err(pin_error)?;
                    delay.delay_us(RESET_US) $(.$await)?;
                    rst.set_high().map_err(pin_error)?;
                    delay.delay_us(RESET_US) $(.$await)?;
                }
                self.wait_init(delay) $(.$await)??;
                let last_gate = self.panel.height - 1;
                self.cmd(cmd::DRIVER_OUTPUT, &[last_gate as u8, (last_gate >> 8) as u8, 0x00]) $(.$await)??;
                self.cmd(cmd::BOOSTER_SOFT_START, &[0xD7, 0xD6, 0x9D]) $(.$await)??;
                self.cmd(cmd::VCOM, &[0xA8]) $(.$await)??;
                self.cmd(cmd::DUMMY_LINE, &[0x1A]) $(.$await)??;
                self.cmd(cmd::GATE_LINE_WIDTH, &[0x08]) $(.$await)??;
                self.cmd(cmd::DATA_ENTRY, &[0x03]) $(.$await)??;
                self.lut = Lut::None;
                self.written = None;
                self.pending = None;
                self.load_lut(Lut::Full) $(.$await)?
            }

            $($async)? fn load_lut(&mut self, want: Lut) -> Result<(), DisplayError<DI::Error>> {
                if self.lut == want {
                    return Ok(());
                }
                let lut = if want == Lut::Partial { self.luts.partial } else { self.luts.full };
                self.cmd(cmd::WRITE_LUT, lut) $(.$await)??;
                self.lut = want;
                Ok(())
            }

            $($async)? fn write_window(&mut self, plane: &[u8], area: Area) -> Result<(), DisplayError<DI::Error>> {
                let stride = self.panel.stride();
                let x0 = (area.x / 8) as u8;
                let x1 = ((area.right() - 1) / 8) as u8;
                let y0 = area.y;
                let y1 = area.bottom() - 1;
                self.cmd(cmd::RAM_X_RANGE, &[x0, x1]) $(.$await)??;
                self.cmd(cmd::RAM_Y_RANGE, &[y0 as u8, (y0 >> 8) as u8, y1 as u8, (y1 >> 8) as u8]) $(.$await)??;
                self.cmd(cmd::RAM_X, &[x0]) $(.$await)??;
                self.cmd(cmd::RAM_Y, &[y0 as u8, (y0 >> 8) as u8]) $(.$await)??;
                self.di.command(&[cmd::WRITE_RAM]) $(.$await)?.map_err(Error::Bus)?;
                if x0 == 0 && usize::from(x1) + 1 == stride {
                    let rows = &plane[usize::from(y0) * stride..usize::from(area.bottom()) * stride];
                    self.di.data(rows) $(.$await)?.map_err(Error::Bus)
                } else {
                    for row in window_rows(plane, stride, area) {
                        self.di.data(row) $(.$await)?.map_err(Error::Bus)?;
                    }
                    Ok(())
                }
            }

            fn check<B: AsRef<[u8]>>(&self, fb: &MonoFrame<B>) -> Result<(), DisplayError<DI::Error>> {
                if fb.width() != self.panel.width || fb.height() != self.panel.height {
                    return Err(Error::InvalidArgument);
                }
                Ok(())
            }

            $($async)? fn activate(&mut self) -> Result<(), DisplayError<DI::Error>> {
                let update = self.update;
                self.cmd(cmd::UPDATE_CONTROL_2, &[update]) $(.$await)??;
                self.cmd(cmd::MASTER_ACTIVATION, &[]) $(.$await)??;
                // Terminate the frame write so the update is not disturbed.
                self.cmd(cmd::NOP, &[]) $(.$await)?
            }

            /// Write the whole frame into RAM (no refresh).
            pub $($async)? fn write_frame<B: AsRef<[u8]>>(&mut self, fb: &MonoFrame<B>) -> Result<(), DisplayError<DI::Error>> {
                self.check(fb)?;
                self.ensure_idle()?;
                let full = fb.full_area();
                self.write_window(fb.as_bytes(), full) $(.$await)??;
                self.written = Some(full);
                Ok(())
            }

            /// Start a full refresh with the full LUT. Returns the typical
            /// duration; poll [`Self::is_busy`], then call
            /// [`Self::finish_refresh`].
            pub $($async)? fn start_full_refresh(&mut self) -> Result<WaitUs, DisplayError<DI::Error>> {
                self.ensure_idle()?;
                self.load_lut(Lut::Full) $(.$await)??;
                self.activate() $(.$await)??;
                self.pending = self.written.take();
                self.partials = 0;
                Ok(self.panel.full_us)
            }

            /// [`Self::write_frame`] + [`Self::start_full_refresh`].
            pub $($async)? fn update<B: AsRef<[u8]>>(&mut self, fb: &MonoFrame<B>) -> Result<WaitUs, DisplayError<DI::Error>> {
                self.write_frame(fb) $(.$await)??;
                self.start_full_refresh() $(.$await)?
            }

            /// Write `area` of `fb` (widened to 8-pixel columns) and start
            /// a partial refresh with the partial LUT. Call
            /// [`Self::finish_refresh`] with the unchanged frame once BUSY
            /// is released.
            pub $($async)? fn start_partial_refresh<B: AsRef<[u8]>>(
                &mut self,
                fb: &MonoFrame<B>,
                area: Area,
            ) -> Result<WaitUs, DisplayError<DI::Error>> {
                self.check(fb)?;
                let p = self.panel;
                let area = area.clip(p.width, p.height).ok_or(Error::InvalidArgument)?.align_x8(p.width);
                self.ensure_idle()?;
                self.load_lut(Lut::Partial) $(.$await)??;
                self.write_window(fb.as_bytes(), area) $(.$await)??;
                self.activate() $(.$await)??;
                self.written = None;
                self.pending = Some(area);
                self.partials = self.partials.saturating_add(1);
                Ok(p.partial_us)
            }

            /// Partial refresh of the frame's dirty rectangle (clears it).
            pub $($async)? fn start_partial_refresh_dirty<B: AsRef<[u8]> + AsMut<[u8]>>(
                &mut self,
                fb: &mut MonoFrame<B>,
            ) -> Result<Option<WaitUs>, DisplayError<DI::Error>> {
                let Some(area) = fb.dirty() else {
                    return Ok(None);
                };
                let wait = self.start_partial_refresh(fb, area) $(.$await)??;
                fb.clear_dirty();
                Ok(Some(wait))
            }

            /// After any refresh: write the last refreshed window again so
            /// both internal buffers match. [`Error::NotReady`] while busy.
            pub $($async)? fn finish_refresh<B: AsRef<[u8]>>(&mut self, fb: &MonoFrame<B>) -> Result<(), DisplayError<DI::Error>> {
                let Some(area) = self.pending else {
                    return Ok(());
                };
                self.check(fb)?;
                self.ensure_idle()?;
                self.write_window(fb.as_bytes(), area) $(.$await)??;
                self.pending = None;
                Ok(())
            }

            /// Deep sleep (`0x10 0x01`). Wake with [`Self::init`] (needs
            /// RST, or a power cycle).
            pub $($async)? fn sleep(&mut self) -> Result<(), DisplayError<DI::Error>> {
                self.ensure_idle()?;
                self.cmd(cmd::DEEP_SLEEP, &[0x01]) $(.$await)??;
                self.lut = Lut::None;
                Ok(())
            }
        }
    };
}

driver!(zigbee_display::Interface, embedded_hal::delay::DelayNs,);

impl<DI, BUSY, RST> zigbee_display::blocking::Display for Il3820<DI, BUSY, RST>
where
    DI: zigbee_display::Interface,
    BUSY: InputPin,
    RST: OutputPin,
{
    type BusError = DI::Error;

    fn init<D: embedded_hal::delay::DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<(), DisplayError<DI::Error>> {
        Il3820::init(self, delay)
    }

    fn start_refresh(&mut self) -> Result<WaitUs, DisplayError<DI::Error>> {
        self.start_full_refresh()
    }

    fn is_busy(&mut self) -> Result<bool, DisplayError<DI::Error>> {
        Il3820::is_busy(self)
    }

    fn sleep(&mut self) -> Result<(), DisplayError<DI::Error>> {
        Il3820::sleep(self)
    }
}

/// Async driver over [`zigbee_display::AsyncInterface`].
pub mod asynch {
    use super::*;

    driver!(zigbee_display::AsyncInterface, embedded_hal_async::delay::DelayNs, async await);

    impl<DI, BUSY, RST> Il3820<DI, BUSY, RST>
    where
        DI: zigbee_display::AsyncInterface,
        BUSY: InputPin + embedded_hal_async::digital::Wait,
        RST: OutputPin,
    {
        /// Await BUSY going low (no polling).
        pub async fn wait_idle(&mut self) -> Result<(), DisplayError<DI::Error>> {
            self.busy.wait_for_low().await.map_err(pin_error)
        }
    }

    impl<DI, BUSY, RST> zigbee_display::asynch::Display for Il3820<DI, BUSY, RST>
    where
        DI: zigbee_display::AsyncInterface,
        BUSY: InputPin,
        RST: OutputPin,
    {
        type BusError = DI::Error;

        async fn init<D: embedded_hal_async::delay::DelayNs>(
            &mut self,
            delay: &mut D,
        ) -> Result<(), DisplayError<DI::Error>> {
            Il3820::init(self, delay).await
        }

        async fn start_refresh(&mut self) -> Result<WaitUs, DisplayError<DI::Error>> {
            self.start_full_refresh().await
        }

        async fn is_busy(&mut self) -> Result<bool, DisplayError<DI::Error>> {
            Il3820::is_busy(self)
        }

        async fn sleep(&mut self) -> Result<(), DisplayError<DI::Error>> {
            Il3820::sleep(self).await
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
        fn init(&mut self, h: u16) -> &mut Self {
            let g = h - 1;
            self.cmd(0x01, &[g as u8, (g >> 8) as u8, 0])
                .cmd(0x0C, &[0xD7, 0xD6, 0x9D])
                .cmd(0x2C, &[0xA8])
                .cmd(0x3A, &[0x1A])
                .cmd(0x3B, &[0x08])
                .cmd(0x11, &[0x03])
                .cmd(0x32, &LUT_FULL)
        }
        fn window(&mut self, x0: u8, x1: u8, y0: u16, y1: u16) -> &mut Self {
            self.cmd(0x44, &[x0, x1])
                .cmd(
                    0x45,
                    &[y0 as u8, (y0 >> 8) as u8, y1 as u8, (y1 >> 8) as u8],
                )
                .cmd(0x4E, &[x0])
                .cmd(0x4F, &[y0 as u8, (y0 >> 8) as u8])
                .write(false, &[0x24])
        }
        fn activate(&mut self) -> &mut Self {
            self.cmd(0x22, &[0xC4]).cmd(0x20, &[]).cmd(0xFF, &[])
        }
        fn mocks(&self) -> (SpiMock<u8>, PinMock) {
            (SpiMock::new(&self.spi), PinMock::new(&self.dc))
        }
    }

    fn idle(n: usize) -> Vec<PinTx> {
        vec![PinTx::get(State::Low); n]
    }

    #[test]
    fn init_sequence_2in9() {
        let mut s = Script::default();
        s.init(296);
        let (mut spi, mut dc) = s.mocks();
        let mut busy = PinMock::new(&[PinTx::get(State::High), PinTx::get(State::Low)]);
        let mut rst = PinMock::new(&[PinTx::set(State::Low), PinTx::set(State::High)]);
        let mut epd = Il3820::new(
            SpiInterface::new(spi.clone(), dc.clone()),
            busy.clone(),
            Panel::IL3820_2IN9,
        )
        .with_reset(rst.clone());
        epd.init(&mut NoopDelay).unwrap();
        spi.done();
        dc.done();
        busy.done();
        rst.done();
    }

    #[test]
    fn full_update_then_finish_rewrites_frame() {
        let panel = Panel::new(16, 2).unwrap();
        let mut fb = MonoFrame::new([0xFFu8; 4], 16, 2).unwrap();
        fb.set_pixel(0, 0, BinaryColor::On);
        let mut s = Script::default();
        s.init(2);
        s.window(0, 1, 0, 1)
            .write(true, &[0x7F, 0xFF, 0xFF, 0xFF])
            .activate()
            .window(0, 1, 0, 1)
            .write(true, &[0x7F, 0xFF, 0xFF, 0xFF]);
        let (mut spi, mut dc) = s.mocks();
        let mut busy_tx = idle(3);
        busy_tx.push(PinTx::get(State::High)); // finish while busy
        busy_tx.push(PinTx::get(State::Low));
        let mut busy = PinMock::new(&busy_tx);
        let mut epd = Il3820::new(
            SpiInterface::new(spi.clone(), dc.clone()),
            busy.clone(),
            panel,
        );
        epd.init(&mut NoopDelay).unwrap();
        assert_eq!(epd.update(&fb).unwrap(), 2_000_000);
        assert!(epd.needs_finish());
        assert_eq!(epd.finish_refresh(&fb), Err(Error::NotReady));
        epd.finish_refresh(&fb).unwrap();
        assert!(!epd.needs_finish());
        // Nothing pending: no traffic.
        epd.finish_refresh(&fb).unwrap();
        spi.done();
        dc.done();
        busy.done();
    }

    #[test]
    fn partial_switches_lut_and_full_switches_back() {
        let mut buf = vec![0xFFu8; Panel::IL3895_2IN13.frame_len()];
        let mut fb = MonoFrame::new(&mut buf[..], 122, 250).unwrap();
        fb.set_pixel(121, 249, BinaryColor::On);
        let mut s = Script::default();
        s.init(250);
        s.cmd(0x32, &LUT_PARTIAL)
            .window(15, 15, 249, 249)
            .write(true, &[0xBF])
            .activate()
            .window(15, 15, 249, 249)
            .write(true, &[0xBF])
            .cmd(0x32, &LUT_FULL)
            .activate();
        let (mut spi, mut dc) = s.mocks();
        let mut busy = PinMock::new(&idle(4));
        let mut epd = Il3820::new(
            SpiInterface::new(spi.clone(), dc.clone()),
            busy.clone(),
            Panel::IL3895_2IN13,
        );
        epd.init(&mut NoopDelay).unwrap();
        assert_eq!(
            epd.start_partial_refresh_dirty(&mut fb).unwrap(),
            Some(400_000)
        );
        assert_eq!(epd.start_partial_refresh_dirty(&mut fb).unwrap(), None);
        assert_eq!(epd.partial_count(), 1);
        epd.finish_refresh(&fb).unwrap();
        // Full refresh without a new write: nothing to finish afterwards.
        epd.start_full_refresh().unwrap();
        assert!(!epd.needs_finish());
        assert_eq!(epd.partial_count(), 0);
        spi.done();
        dc.done();
        busy.done();
    }

    #[test]
    fn busy_custom_luts_update_control_and_sleep() {
        static F: [u8; 2] = [1, 2];
        static P: [u8; 2] = [3, 4];
        let panel = Panel::new(8, 1).unwrap();
        let mut s = Script::default();
        s.cmd(0x01, &[0, 0, 0])
            .cmd(0x0C, &[0xD7, 0xD6, 0x9D])
            .cmd(0x2C, &[0xA8])
            .cmd(0x3A, &[0x1A])
            .cmd(0x3B, &[0x08])
            .cmd(0x11, &[0x03])
            .cmd(0x32, &F)
            .cmd(0x22, &[0xC7])
            .cmd(0x20, &[])
            .cmd(0xFF, &[])
            .cmd(0x10, &[0x01]);
        let (mut spi, mut dc) = s.mocks();
        let mut busy_tx = idle(2);
        busy_tx.push(PinTx::get(State::High));
        busy_tx.push(PinTx::get(State::Low));
        let mut busy = PinMock::new(&busy_tx);
        let mut epd = Il3820::new(
            SpiInterface::new(spi.clone(), dc.clone()),
            busy.clone(),
            panel,
        )
        .with_luts(Luts {
            full: &F,
            partial: &P,
        })
        .with_update_control(0xC7);
        epd.init(&mut NoopDelay).unwrap();
        epd.start_full_refresh().unwrap();
        assert_eq!(epd.sleep(), Err(Error::NotReady));
        epd.sleep().unwrap();
        spi.done();
        dc.done();
        busy.done();
    }

    #[test]
    fn panel_ram_and_validation() {
        assert_eq!(Panel::IL3820_2IN9.frame_len(), 4736);
        assert_eq!(Panel::IL3895_2IN13.frame_len(), 4000);
        assert_eq!(Panel::IL3829_1IN54.frame_len(), 5000);
        assert!(Panel::new(0, 10).is_none());
        assert!(Panel::new(128, 321).is_none());
        let fb = MonoFrame::new([0u8; 1], 8, 1).unwrap();
        let (mut spi, mut dc) = Script::default().mocks();
        let mut busy = PinMock::new(&[]);
        let mut epd = Il3820::new(
            SpiInterface::new(spi.clone(), dc.clone()),
            busy.clone(),
            Panel::IL3820_2IN9,
        );
        assert_eq!(epd.write_frame(&fb), Err(Error::InvalidArgument));
        spi.done();
        dc.done();
        busy.done();
    }

    #[test]
    fn async_partial_and_wait_idle() {
        let panel = Panel::new(8, 2).unwrap();
        let fb = MonoFrame::new([0x55u8, 0xAA], 8, 2).unwrap();
        let mut s = Script::default();
        s.init(2);
        s.cmd(0x32, &LUT_PARTIAL)
            .window(0, 0, 1, 1)
            .write(true, &[0xAA])
            .activate();
        let (mut spi, mut dc) = s.mocks();
        let mut busy_tx = idle(2);
        busy_tx.push(PinTx::wait_for_state(State::Low));
        let mut busy = PinMock::new(&busy_tx);
        let mut epd = asynch::Il3820::new(
            SpiInterface::new(spi.clone(), dc.clone()),
            busy.clone(),
            panel,
        );
        block_on(async {
            epd.init(&mut NoopDelay).await?;
            epd.start_partial_refresh(&fb, Area::new(0, 1, 8, 1))
                .await?;
            epd.wait_idle().await
        })
        .unwrap();
        spi.done();
        dc.done();
        busy.done();
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
