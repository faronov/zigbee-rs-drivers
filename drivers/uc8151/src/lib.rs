//! UltraChip UC8151 (UC8151C/D) and Good Display IL0373 e-paper
//! controllers: 2.9" 128x296 and 2.13" 104x212 black/white (Pimoroni
//! Badger 2040, Waveshare 2.9"/2.13" D) and black/white/red (Waveshare
//! 2.9"/2.13" B/C, Adafruit tricolour FeatherWings) panels.
//!
//! Both chips share one command set: PSR `0x00`, POF `0x02`, PON `0x04`,
//! BTST `0x06`, DSLP `0x07`, DTM1 `0x10`, DRF `0x12`, DTM2 `0x13`, CDI
//! `0x50`, TRES `0x61`, PTL `0x90`, PTIN `0x91`, PTOUT `0x92`.
//!
//! * **BUSY_N is active low**: low = busy.
//! * B/W (KW mode): DTM1 = previous image, DTM2 = new image, 1 = white
//!   (CDI `0x97`), identical to [`MonoFrame`].
//! * Tricolour (KWR mode): DTM1 = black plane (1 = white), DTM2 = red
//!   plane with **0 = red**. The driver inverts [`TriFrame`]'s red plane
//!   (1 = red) on the fly in a 64-byte stack chunk.
//!
//! # Refresh without blocking
//!
//! ```ignore
//! epd.init(&mut delay)?;                 // reset, booster, power on
//! let wait = epd.update(&fb)?;           // DTM1/DTM2 + DRF
//! // ... after `wait`, when !epd.is_busy()?:
//! loop {
//!     match epd.sleep() {                // CDI + POF, then DSLP
//!         Err(Error::Pending(us)) => { /* come back after `us` */ }
//!         other => break other?,
//!     }
//! }
//! ```
//!
//! Partial refresh (B/W): `start_partial_refresh(&fb, area)` sends PTIN,
//! PTL with the window, DTM2 with the window and DRF. Once BUSY is
//! released, `finish_refresh(&fb)` writes the same window to DTM1 (the
//! previous image) and leaves partial mode. Without custom LUTs the panel
//! runs its OTP full waveform inside the window (flashes, ~full-refresh
//! time but only the window changes). Faster waveforms can be supplied
//! as register LUTs via [`Uc8151::with_partial_luts`]; none are bundled.
//!
//! References: UltraChip UC8151C datasheet v0.3 and Good Display IL0373
//! datasheet (command tables, power on/off flow, partial window).

#![no_std]

use embedded_hal::digital::{InputPin, OutputPin};
use zigbee_display::frame::window_rows;
use zigbee_display::{Area, DisplayError, Error, MonoFrame, NoPin, TriFrame, WaitUs, pin_error};

pub use zigbee_display;
pub use zigbee_display::SpiInterface;

/// Command bytes.
pub mod cmd {
    /// Panel setting.
    pub const PSR: u8 = 0x00;
    /// Power off.
    pub const POF: u8 = 0x02;
    /// Power on.
    pub const PON: u8 = 0x04;
    /// Booster soft start.
    pub const BTST: u8 = 0x06;
    /// Deep sleep (`0xA5` check code).
    pub const DSLP: u8 = 0x07;
    /// Data start transmission 1 (old / black).
    pub const DTM1: u8 = 0x10;
    /// Display refresh.
    pub const DRF: u8 = 0x12;
    /// Data start transmission 2 (new / red).
    pub const DTM2: u8 = 0x13;
    /// VCOM LUT.
    pub const LUT_VCOM: u8 = 0x20;
    /// White-to-white LUT.
    pub const LUT_WW: u8 = 0x21;
    /// Black-to-white LUT.
    pub const LUT_BW: u8 = 0x22;
    /// White-to-black LUT.
    pub const LUT_WB: u8 = 0x23;
    /// Black-to-black LUT.
    pub const LUT_BB: u8 = 0x24;
    /// VCOM and data interval.
    pub const CDI: u8 = 0x50;
    /// Resolution.
    pub const TRES: u8 = 0x61;
    /// Partial window.
    pub const PTL: u8 = 0x90;
    /// Partial in.
    pub const PTIN: u8 = 0x91;
    /// Partial out.
    pub const PTOUT: u8 = 0x92;
}

/// PSR, B/W: KW mode, LUT from OTP, scan up, shift right, booster on, no reset.
pub const PSR_BW: u8 = 0x1F;
/// PSR, tricolour: KWR mode, LUT from OTP.
pub const PSR_BWR: u8 = 0x0F;
/// PSR bit: LUT from registers.
pub const PSR_REG_LUT: u8 = 0x20;
/// CDI, B/W: border white, data polarity 1 = white, interval 10.
pub const CDI_BW: u8 = 0x97;
/// CDI, tricolour: border white, interval 10.
pub const CDI_BWR: u8 = 0x77;
/// CDI before power off: border floating.
pub const CDI_SLEEP: u8 = 0xF7;

/// HW reset pulse and wait.
pub const RESET_US: u32 = 10_000;
/// Longest BUSY wait inside `init` (power on).
pub const INIT_BUSY_TIMEOUT_US: u32 = 500_000;
const INIT_POLL_US: u32 = 1_000;
/// Typical power-off (POF) duration, returned as `Error::Pending` by
/// `sleep`.
pub const POWER_OFF_US: WaitUs = 50_000;

/// Register LUTs for partial refresh (`0x20`..`0x24`), panel specific.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Luts {
    /// VCOM LUT (`0x20`).
    pub vcom: &'static [u8],
    /// White to white (`0x21`).
    pub ww: &'static [u8],
    /// Black to white (`0x22`).
    pub bw: &'static [u8],
    /// White to black (`0x23`).
    pub wb: &'static [u8],
    /// Black to black (`0x24`).
    pub bb: &'static [u8],
}

/// Panel description.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Panel {
    width: u16,
    height: u16,
    tricolor: bool,
    full_us: WaitUs,
    partial_us: WaitUs,
}

impl Panel {
    /// 2.9" 128x296 B/W (UC8151: Pimoroni Badger 2040, Waveshare 2.9" D).
    pub const UC8151_2IN9: Panel = Panel::raw(128, 296, false);
    /// 2.13" 104x212 B/W (UC8151D: Waveshare 2.13" D flexible).
    pub const UC8151_2IN13: Panel = Panel::raw(104, 212, false);
    /// 2.9" 128x296 black/white/red (IL0373: Waveshare 2.9" B/C).
    pub const IL0373_2IN9_RED: Panel = Panel::raw(128, 296, true);
    /// 2.13" 104x212 black/white/red (IL0373: Waveshare 2.13" B/C,
    /// Adafruit tricolour FeatherWing).
    pub const IL0373_2IN13_RED: Panel = Panel::raw(104, 212, true);

    const fn raw(width: u16, height: u16, tricolor: bool) -> Self {
        Self {
            width,
            height,
            tricolor,
            full_us: if tricolor { 15_000_000 } else { 3_000_000 },
            partial_us: 3_000_000,
        }
    }

    /// Custom geometry. `None` unless `width` is a multiple of 8 up to
    /// 160 and `height` is 1..=296.
    pub const fn new(width: u16, height: u16, tricolor: bool) -> Option<Self> {
        if width == 0 || width % 8 != 0 || width > 160 || height == 0 || height > 296 {
            return None;
        }
        Some(Self::raw(width, height, tricolor))
    }

    /// Override the typical refresh times returned as wait hints (e.g.
    /// with fast partial LUTs).
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

    /// Black/white/red panel.
    pub const fn is_tricolor(&self) -> bool {
        self.tricolor
    }

    /// Typical full-refresh time (wait hint; BUSY is authoritative).
    pub const fn full_refresh_us(&self) -> WaitUs {
        self.full_us
    }

    /// Frame bytes ([`zigbee_display::mono_len`] / [`zigbee_display::tri_len`]).
    pub const fn frame_len(&self) -> usize {
        if self.tricolor {
            zigbee_display::tri_len(self.width, self.height)
        } else {
            zigbee_display::mono_len(self.width, self.height)
        }
    }

    const fn stride(&self) -> usize {
        self.width as usize / 8
    }

    const fn psr(&self) -> u8 {
        if self.tricolor { PSR_BWR } else { PSR_BW }
    }

    const fn cdi(&self) -> u8 {
        if self.tricolor { CDI_BWR } else { CDI_BW }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Power {
    On,
    PoweringOff,
    Asleep,
}

const CHUNK: usize = 64;

/// PTL parameters for a byte-aligned window.
fn ptl(area: Area) -> [u8; 7] {
    let x0 = area.x & !7;
    let x1 = (area.right() - 1) | 7;
    let y0 = area.y;
    let y1 = area.bottom() - 1;
    [
        x0 as u8,
        x1 as u8,
        (y0 >> 8) as u8,
        y0 as u8,
        (y1 >> 8) as u8,
        y1 as u8,
        0x01,
    ]
}

macro_rules! driver {
    ($Iface:path, $Delay:path, $($async:tt $await:tt)?) => {
        /// UC8151 / IL0373 driver. BUSY_N (active low) is required; RST
        /// is optional but needed to leave deep sleep.
        pub struct Uc8151<DI, BUSY, RST = NoPin> {
            di: DI,
            busy: BUSY,
            rst: Option<RST>,
            panel: Panel,
            partial_luts: Option<Luts>,
            reg_lut: bool,
            in_partial: bool,
            pending: Option<Area>,
            partials: u16,
            power: Power,
        }

        impl<DI: $Iface, BUSY: InputPin> Uc8151<DI, BUSY> {
            /// Driver without a reset pin.
            pub fn new(di: DI, busy: BUSY, panel: Panel) -> Self {
                Self {
                    di,
                    busy,
                    rst: None,
                    panel,
                    partial_luts: None,
                    reg_lut: false,
                    in_partial: false,
                    pending: None,
                    partials: 0,
                    power: Power::Asleep,
                }
            }
        }

        impl<DI: $Iface, BUSY: InputPin, RST: OutputPin> Uc8151<DI, BUSY, RST> {
            /// Add the RST pin, pulsed by [`Self::init`].
            pub fn with_reset<P: OutputPin>(self, rst: P) -> Uc8151<DI, BUSY, P> {
                Uc8151 {
                    di: self.di,
                    busy: self.busy,
                    rst: Some(rst),
                    panel: self.panel,
                    partial_luts: self.partial_luts,
                    reg_lut: self.reg_lut,
                    in_partial: self.in_partial,
                    pending: self.pending,
                    partials: self.partials,
                    power: self.power,
                }
            }

            /// Register LUTs used for partial refresh (full refresh stays
            /// on the OTP waveform).
            pub fn with_partial_luts(mut self, luts: Luts) -> Self {
                self.partial_luts = Some(luts);
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

            /// Give the peripherals back.
            pub fn release(self) -> (DI, BUSY, Option<RST>) {
                (self.di, self.busy, self.rst)
            }

            /// `true` while BUSY_N is low.
            pub fn is_busy(&mut self) -> Result<bool, DisplayError<DI::Error>> {
                self.busy.is_low().map_err(pin_error)
            }

            fn ensure_ready(&mut self) -> Result<(), DisplayError<DI::Error>> {
                if self.power != Power::On || self.is_busy()? {
                    return Err(Error::NotReady);
                }
                Ok(())
            }

            $($async)? fn cmd(&mut self, command: u8, data: &[u8]) -> Result<(), DisplayError<DI::Error>> {
                self.di.command(&[command]) $(.$await)?.map_err(Error::Bus)?;
                self.di.data(data) $(.$await)?.map_err(Error::Bus)
            }

            /// Send a raw command with parameters (panel powered, BUSY_N high).
            pub $($async)? fn command(&mut self, command: u8, data: &[u8]) -> Result<(), DisplayError<DI::Error>> {
                self.ensure_ready()?;
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

            /// Reset, booster soft start, power on, panel setting,
            /// resolution. Blocks for the 20 ms reset plus the power-on
            /// BUSY (≤ [`INIT_BUSY_TIMEOUT_US`]). Also leaves deep sleep.
            pub $($async)? fn init<D: $Delay>(&mut self, delay: &mut D) -> Result<(), DisplayError<DI::Error>> {
                if let Some(rst) = self.rst.as_mut() {
                    rst.set_low().map_err(pin_error)?;
                    delay.delay_us(RESET_US) $(.$await)?;
                    rst.set_high().map_err(pin_error)?;
                    delay.delay_us(RESET_US) $(.$await)?;
                }
                self.wait_init(delay) $(.$await)??;
                let p = self.panel;
                self.cmd(cmd::BTST, &[0x17, 0x17, 0x17]) $(.$await)??;
                self.cmd(cmd::PON, &[]) $(.$await)??;
                self.wait_init(delay) $(.$await)??;
                self.cmd(cmd::PSR, &[p.psr()]) $(.$await)??;
                self.cmd(cmd::CDI, &[p.cdi()]) $(.$await)??;
                self.cmd(cmd::TRES, &[p.width as u8, (p.height >> 8) as u8, p.height as u8]) $(.$await)??;
                self.power = Power::On;
                self.reg_lut = false;
                self.in_partial = false;
                self.pending = None;
                Ok(())
            }

            $($async)? fn leave_partial(&mut self) -> Result<(), DisplayError<DI::Error>> {
                if self.in_partial {
                    self.cmd(cmd::PTOUT, &[]) $(.$await)??;
                    self.in_partial = false;
                }
                Ok(())
            }

            $($async)? fn send_plane(&mut self, plane: &[u8], area: Area, invert: bool) -> Result<(), DisplayError<DI::Error>> {
                let stride = self.panel.stride();
                let full_rows = area.x == 0 && usize::from(area.right()).div_ceil(8) == stride;
                if full_rows && !invert {
                    let rows = &plane[usize::from(area.y) * stride..usize::from(area.bottom()) * stride];
                    return self.di.data(rows) $(.$await)?.map_err(Error::Bus);
                }
                let mut chunk = [0u8; CHUNK];
                for row in window_rows(plane, stride, area) {
                    for part in row.chunks(CHUNK) {
                        let out = &mut chunk[..part.len()];
                        for (o, i) in out.iter_mut().zip(part) {
                            *o = if invert { !*i } else { *i };
                        }
                        self.di.data(out) $(.$await)?.map_err(Error::Bus)?;
                    }
                }
                Ok(())
            }

            $($async)? fn send_fill(&mut self, value: u8) -> Result<(), DisplayError<DI::Error>> {
                let chunk = [value; CHUNK];
                let mut left = self.panel.frame_len() / if self.panel.tricolor { 2 } else { 1 };
                while left > 0 {
                    let n = left.min(CHUNK);
                    self.di.data(&chunk[..n]) $(.$await)?.map_err(Error::Bus)?;
                    left -= n;
                }
                Ok(())
            }

            fn check_size(&self, width: u16, height: u16) -> Result<(), DisplayError<DI::Error>> {
                if width != self.panel.width || height != self.panel.height {
                    return Err(Error::InvalidArgument);
                }
                Ok(())
            }

            /// Write a B/W frame to DTM1 (previous) and DTM2 (new). On a
            /// tricolour panel DTM1 gets the frame and DTM2 "no red".
            pub $($async)? fn write_frame<B: AsRef<[u8]>>(&mut self, fb: &MonoFrame<B>) -> Result<(), DisplayError<DI::Error>> {
                self.check_size(fb.width(), fb.height())?;
                self.ensure_ready()?;
                self.leave_partial() $(.$await)??;
                let full = fb.full_area();
                self.cmd(cmd::DTM1, &[]) $(.$await)??;
                self.send_plane(fb.as_bytes(), full, false) $(.$await)??;
                self.cmd(cmd::DTM2, &[]) $(.$await)??;
                if self.panel.tricolor {
                    self.send_fill(0xFF) $(.$await)??;
                } else {
                    self.send_plane(fb.as_bytes(), full, false) $(.$await)??;
                }
                self.pending = None;
                Ok(())
            }

            /// Write a black/white/red frame (tricolour panels only).
            pub $($async)? fn write_tri_frame<B: AsRef<[u8]>>(&mut self, fb: &TriFrame<B>) -> Result<(), DisplayError<DI::Error>> {
                if !self.panel.tricolor {
                    return Err(Error::Unsupported);
                }
                self.check_size(fb.width(), fb.height())?;
                self.ensure_ready()?;
                self.leave_partial() $(.$await)??;
                let full = fb.full_area();
                self.cmd(cmd::DTM1, &[]) $(.$await)??;
                self.send_plane(fb.black_plane(), full, false) $(.$await)??;
                self.cmd(cmd::DTM2, &[]) $(.$await)??;
                self.send_plane(fb.red_plane(), full, true) $(.$await)??;
                self.pending = None;
                Ok(())
            }

            /// Start a full refresh (OTP waveform). Returns the typical
            /// duration; poll [`Self::is_busy`].
            pub $($async)? fn start_full_refresh(&mut self) -> Result<WaitUs, DisplayError<DI::Error>> {
                self.ensure_ready()?;
                self.leave_partial() $(.$await)??;
                if self.reg_lut {
                    let psr = self.panel.psr();
                    self.cmd(cmd::PSR, &[psr]) $(.$await)??;
                    self.reg_lut = false;
                }
                self.cmd(cmd::DRF, &[]) $(.$await)??;
                self.partials = 0;
                Ok(self.panel.full_us)
            }

            /// [`Self::write_frame`] + [`Self::start_full_refresh`].
            pub $($async)? fn update<B: AsRef<[u8]>>(&mut self, fb: &MonoFrame<B>) -> Result<WaitUs, DisplayError<DI::Error>> {
                self.write_frame(fb) $(.$await)??;
                self.start_full_refresh() $(.$await)?
            }

            /// [`Self::write_tri_frame`] + [`Self::start_full_refresh`].
            pub $($async)? fn update_tri<B: AsRef<[u8]>>(&mut self, fb: &TriFrame<B>) -> Result<WaitUs, DisplayError<DI::Error>> {
                self.write_tri_frame(fb) $(.$await)??;
                self.start_full_refresh() $(.$await)?
            }

            /// Partial refresh of `area` (widened to 8-pixel columns): PTIN,
            /// PTL, DTM2 window, DRF. Call [`Self::finish_refresh`] with
            /// the unchanged frame once BUSY is released. B/W only.
            pub $($async)? fn start_partial_refresh<B: AsRef<[u8]>>(
                &mut self,
                fb: &MonoFrame<B>,
                area: Area,
            ) -> Result<WaitUs, DisplayError<DI::Error>> {
                if self.panel.tricolor {
                    return Err(Error::Unsupported);
                }
                self.check_size(fb.width(), fb.height())?;
                let p = self.panel;
                let area = area.clip(p.width, p.height).ok_or(Error::InvalidArgument)?.align_x8(p.width);
                self.ensure_ready()?;
                if let Some(luts) = self.partial_luts {
                    if !self.reg_lut {
                        self.cmd(cmd::PSR, &[p.psr() | PSR_REG_LUT]) $(.$await)??;
                        self.cmd(cmd::LUT_VCOM, luts.vcom) $(.$await)??;
                        self.cmd(cmd::LUT_WW, luts.ww) $(.$await)??;
                        self.cmd(cmd::LUT_BW, luts.bw) $(.$await)??;
                        self.cmd(cmd::LUT_WB, luts.wb) $(.$await)??;
                        self.cmd(cmd::LUT_BB, luts.bb) $(.$await)??;
                        self.reg_lut = true;
                    }
                }
                self.cmd(cmd::PTIN, &[]) $(.$await)??;
                self.in_partial = true;
                self.cmd(cmd::PTL, &ptl(area)) $(.$await)??;
                self.cmd(cmd::DTM2, &[]) $(.$await)??;
                self.send_plane(fb.as_bytes(), area, false) $(.$await)??;
                self.cmd(cmd::DRF, &[]) $(.$await)??;
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

            /// After a partial refresh: write the window to DTM1 (previous
            /// image) and leave partial mode. [`Error::NotReady`] while busy.
            pub $($async)? fn finish_refresh<B: AsRef<[u8]>>(&mut self, fb: &MonoFrame<B>) -> Result<(), DisplayError<DI::Error>> {
                let Some(area) = self.pending else {
                    return Ok(());
                };
                self.check_size(fb.width(), fb.height())?;
                self.ensure_ready()?;
                self.cmd(cmd::DTM1, &[]) $(.$await)??;
                self.send_plane(fb.as_bytes(), area, false) $(.$await)??;
                self.leave_partial() $(.$await)??;
                self.pending = None;
                Ok(())
            }

            /// Two-phase deep sleep. First call: floating border + power off
            /// (POF), returns `Err(Error::Pending(POWER_OFF_US))`. Call
            /// again once BUSY is released: deep sleep (DSLP `0xA5`),
            /// returns `Ok(())`. Leave deep sleep with [`Self::init`].
            pub $($async)? fn sleep(&mut self) -> Result<(), DisplayError<DI::Error>> {
                match self.power {
                    Power::Asleep => Ok(()),
                    Power::On => {
                        if self.is_busy()? {
                            return Err(Error::NotReady);
                        }
                        self.leave_partial() $(.$await)??;
                        self.cmd(cmd::CDI, &[CDI_SLEEP]) $(.$await)??;
                        self.cmd(cmd::POF, &[]) $(.$await)??;
                        self.power = Power::PoweringOff;
                        Err(Error::Pending(POWER_OFF_US))
                    }
                    Power::PoweringOff => {
                        if self.is_busy()? {
                            return Err(Error::NotReady);
                        }
                        self.cmd(cmd::DSLP, &[0xA5]) $(.$await)??;
                        self.power = Power::Asleep;
                        Ok(())
                    }
                }
            }
        }
    };
}

driver!(zigbee_display::Interface, embedded_hal::delay::DelayNs,);

impl<DI, BUSY, RST> zigbee_display::blocking::Display for Uc8151<DI, BUSY, RST>
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
        Uc8151::init(self, delay)
    }

    fn start_refresh(&mut self) -> Result<WaitUs, DisplayError<DI::Error>> {
        self.start_full_refresh()
    }

    fn is_busy(&mut self) -> Result<bool, DisplayError<DI::Error>> {
        Uc8151::is_busy(self)
    }

    fn sleep(&mut self) -> Result<(), DisplayError<DI::Error>> {
        Uc8151::sleep(self)
    }
}

/// Async driver over [`zigbee_display::AsyncInterface`].
pub mod asynch {
    use super::*;

    driver!(zigbee_display::AsyncInterface, embedded_hal_async::delay::DelayNs, async await);

    impl<DI, BUSY, RST> Uc8151<DI, BUSY, RST>
    where
        DI: zigbee_display::AsyncInterface,
        BUSY: InputPin + embedded_hal_async::digital::Wait,
        RST: OutputPin,
    {
        /// Await BUSY_N going high (no polling).
        pub async fn wait_idle(&mut self) -> Result<(), DisplayError<DI::Error>> {
            self.busy.wait_for_high().await.map_err(pin_error)
        }
    }

    impl<DI, BUSY, RST> zigbee_display::asynch::Display for Uc8151<DI, BUSY, RST>
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
            Uc8151::init(self, delay).await
        }

        async fn start_refresh(&mut self) -> Result<WaitUs, DisplayError<DI::Error>> {
            self.start_full_refresh().await
        }

        async fn is_busy(&mut self) -> Result<bool, DisplayError<DI::Error>> {
            Uc8151::is_busy(self)
        }

        async fn sleep(&mut self) -> Result<(), DisplayError<DI::Error>> {
            Uc8151::sleep(self).await
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
    use zigbee_display::TriColor;

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
        fn init(&mut self, psr: u8, cdi: u8, w: u8, h: u16) -> &mut Self {
            self.cmd(0x06, &[0x17, 0x17, 0x17])
                .cmd(0x04, &[])
                .cmd(0x00, &[psr])
                .cmd(0x50, &[cdi])
                .cmd(0x61, &[w, (h >> 8) as u8, h as u8])
        }
        fn mocks(&self) -> (SpiMock<u8>, PinMock) {
            (SpiMock::new(&self.spi), PinMock::new(&self.dc))
        }
    }

    /// BUSY_N reads: high = idle.
    fn idle(n: usize) -> Vec<PinTx> {
        vec![PinTx::get(State::High); n]
    }

    #[test]
    fn bw_init_sequence() {
        let mut s = Script::default();
        s.init(0x1F, 0x97, 128, 296);
        let (mut spi, mut dc) = s.mocks();
        // Reset idle, then busy once during power on.
        let mut busy = PinMock::new(&[
            PinTx::get(State::High),
            PinTx::get(State::Low),
            PinTx::get(State::High),
        ]);
        let mut rst = PinMock::new(&[PinTx::set(State::Low), PinTx::set(State::High)]);
        let mut epd = Uc8151::new(
            SpiInterface::new(spi.clone(), dc.clone()),
            busy.clone(),
            Panel::UC8151_2IN9,
        )
        .with_reset(rst.clone());
        epd.init(&mut NoopDelay).unwrap();
        spi.done();
        dc.done();
        busy.done();
        rst.done();
    }

    #[test]
    fn not_initialised_or_busy_is_not_ready() {
        let fb = MonoFrame::new([0xFFu8; 2], 8, 2).unwrap();
        let panel = Panel::new(8, 2, false).unwrap();
        let mut s = Script::default();
        s.init(0x1F, 0x97, 8, 2);
        let (mut spi, mut dc) = s.mocks();
        let mut tx = idle(2);
        tx.push(PinTx::get(State::Low));
        let mut busy = PinMock::new(&tx);
        let mut epd = Uc8151::new(
            SpiInterface::new(spi.clone(), dc.clone()),
            busy.clone(),
            panel,
        );
        assert_eq!(epd.write_frame(&fb), Err(Error::NotReady));
        epd.init(&mut NoopDelay).unwrap();
        // BUSY_N low = busy.
        assert_eq!(epd.write_frame(&fb), Err(Error::NotReady));
        spi.done();
        dc.done();
        busy.done();
    }

    fn ready(panel: Panel, s: &mut Script) -> Vec<PinTx> {
        let (psr, cdi) = (panel.psr(), panel.cdi());
        s.init(psr, cdi, panel.width() as u8, panel.height());
        idle(2)
    }

    #[test]
    fn bw_full_update_writes_old_and_new() {
        let panel = Panel::new(16, 2, false).unwrap();
        let mut fb = MonoFrame::new([0xFFu8; 4], 16, 2).unwrap();
        fb.set_pixel(15, 1, BinaryColor::On);
        let mut s = Script::default();
        let mut busy_tx = ready(panel, &mut s);
        s.cmd(0x10, &[0xFF, 0xFF, 0xFF, 0xFE])
            .cmd(0x13, &[0xFF, 0xFF, 0xFF, 0xFE])
            .cmd(0x12, &[]);
        busy_tx.extend(idle(2));
        let (mut spi, mut dc) = s.mocks();
        let mut busy = PinMock::new(&busy_tx);
        let mut epd = Uc8151::new(
            SpiInterface::new(spi.clone(), dc.clone()),
            busy.clone(),
            panel,
        );
        epd.init(&mut NoopDelay).unwrap();
        assert_eq!(epd.update(&fb).unwrap(), 3_000_000);
        spi.done();
        dc.done();
        busy.done();
    }

    #[test]
    fn tricolor_red_plane_is_inverted() {
        let panel = Panel::new(8, 2, true).unwrap();
        let mut fb = TriFrame::new([0u8; 4], 8, 2).unwrap();
        fb.fill(TriColor::White);
        fb.set_pixel(0, 0, TriColor::Black);
        fb.set_pixel(1, 1, TriColor::Red);
        let mut s = Script::default();
        let mut busy_tx = ready(panel, &mut s);
        s.cmd(0x10, &[0x7F, 0xFF])
            .write(false, &[0x13])
            .write(true, &[0xFF])
            .write(true, &[0xBF])
            .cmd(0x12, &[]);
        busy_tx.extend(idle(2));
        let (mut spi, mut dc) = s.mocks();
        let mut busy = PinMock::new(&busy_tx);
        let mut epd = Uc8151::new(
            SpiInterface::new(spi.clone(), dc.clone()),
            busy.clone(),
            panel,
        );
        epd.init(&mut NoopDelay).unwrap();
        assert_eq!(epd.update_tri(&fb).unwrap(), 15_000_000);
        spi.done();
        dc.done();
        busy.done();
    }

    #[test]
    fn partial_window_ptl_and_finish() {
        let panel = Panel::new(24, 300 - 4, false).unwrap();
        let mut buf = std::vec![0xFFu8; panel.frame_len()];
        let mut fb = MonoFrame::new(&mut buf[..], 24, 296).unwrap();
        fb.set_pixel(9, 260, BinaryColor::On);
        let mut s = Script::default();
        let mut busy_tx = ready(panel, &mut s);
        s.cmd(0x91, &[])
            .cmd(0x90, &[8, 15, 0x01, 0x04, 0x01, 0x04, 0x01])
            .write(false, &[0x13])
            .write(true, &[0xBF])
            .cmd(0x12, &[])
            .write(false, &[0x10])
            .write(true, &[0xBF])
            .cmd(0x92, &[]);
        busy_tx.extend(idle(2));
        let (mut spi, mut dc) = s.mocks();
        let mut busy = PinMock::new(&busy_tx);
        let mut epd = Uc8151::new(
            SpiInterface::new(spi.clone(), dc.clone()),
            busy.clone(),
            panel,
        );
        epd.init(&mut NoopDelay).unwrap();
        assert_eq!(
            epd.start_partial_refresh_dirty(&mut fb).unwrap(),
            Some(3_000_000)
        );
        assert_eq!(epd.partial_count(), 1);
        epd.finish_refresh(&fb).unwrap();
        spi.done();
        dc.done();
        busy.done();
    }

    static L: [u8; 2] = [0xAA, 0x55];

    #[test]
    fn partial_luts_loaded_once_and_otp_restored_for_full() {
        let panel = Panel::new(8, 1, false).unwrap();
        let fb = MonoFrame::new([0x0Fu8], 8, 1).unwrap();
        let luts = Luts {
            vcom: &L,
            ww: &L,
            bw: &L,
            wb: &L,
            bb: &L,
        };
        let mut s = Script::default();
        let mut busy_tx = ready(panel, &mut s);
        s.cmd(0x00, &[0x3F])
            .cmd(0x20, &L)
            .cmd(0x21, &L)
            .cmd(0x22, &L)
            .cmd(0x23, &L)
            .cmd(0x24, &L)
            .cmd(0x91, &[])
            .cmd(0x90, &[0, 7, 0, 0, 0, 0, 1])
            .cmd(0x13, &[0x0F])
            .cmd(0x12, &[])
            // second partial: LUTs already loaded
            .cmd(0x91, &[])
            .cmd(0x90, &[0, 7, 0, 0, 0, 0, 1])
            .cmd(0x13, &[0x0F])
            .cmd(0x12, &[])
            // full refresh: leave partial, back to OTP
            .cmd(0x92, &[])
            .cmd(0x00, &[0x1F])
            .cmd(0x12, &[]);
        busy_tx.extend(idle(3));
        let (mut spi, mut dc) = s.mocks();
        let mut busy = PinMock::new(&busy_tx);
        let mut epd = Uc8151::new(
            SpiInterface::new(spi.clone(), dc.clone()),
            busy.clone(),
            panel,
        )
        .with_partial_luts(luts);
        epd.init(&mut NoopDelay).unwrap();
        epd.start_partial_refresh(&fb, fb.full_area()).unwrap();
        epd.start_partial_refresh(&fb, fb.full_area()).unwrap();
        epd.start_full_refresh().unwrap();
        assert_eq!(epd.partial_count(), 0);
        spi.done();
        dc.done();
        busy.done();
    }

    #[test]
    fn sleep_is_two_phase_and_helper_follows_it() {
        let panel = Panel::new(8, 1, false).unwrap();
        let mut s = Script::default();
        let mut busy_tx = ready(panel, &mut s);
        s.cmd(0x50, &[0xF7]).cmd(0x02, &[]).cmd(0x07, &[0xA5]);
        // phase 1 idle; helper wait_idle: busy then idle; phase 2 idle.
        busy_tx.extend([
            PinTx::get(State::High),
            PinTx::get(State::Low),
            PinTx::get(State::High),
            PinTx::get(State::High),
        ]);
        let (mut spi, mut dc) = s.mocks();
        let mut busy = PinMock::new(&busy_tx);
        let mut epd = Uc8151::new(
            SpiInterface::new(spi.clone(), dc.clone()),
            busy.clone(),
            panel,
        );
        epd.init(&mut NoopDelay).unwrap();
        zigbee_display::blocking::sleep(&mut epd, &mut NoopDelay).unwrap();
        // Already asleep: no traffic.
        epd.sleep().unwrap();
        spi.done();
        dc.done();
        busy.done();
    }

    #[test]
    fn ptl_bytes_and_panel_ram() {
        assert_eq!(ptl(Area::new(3, 0, 2, 1)), [0, 7, 0, 0, 0, 0, 1]);
        assert_eq!(ptl(Area::new(8, 255, 16, 2)), [8, 23, 0, 255, 1, 0, 1]);
        assert_eq!(Panel::UC8151_2IN9.frame_len(), 4736);
        assert_eq!(Panel::UC8151_2IN13.frame_len(), 2756);
        assert_eq!(Panel::IL0373_2IN13_RED.frame_len(), 5512);
        assert_eq!(Panel::IL0373_2IN9_RED.frame_len(), 9472);
        assert!(Panel::new(122, 250, false).is_none());
    }

    #[test]
    fn async_update_and_wait_idle() {
        let panel = Panel::new(8, 1, false).unwrap();
        let fb = MonoFrame::new([0x00u8], 8, 1).unwrap();
        let mut s = Script::default();
        let mut busy_tx = ready(panel, &mut s);
        s.cmd(0x10, &[0x00]).cmd(0x13, &[0x00]).cmd(0x12, &[]);
        busy_tx.extend(idle(2));
        busy_tx.push(PinTx::wait_for_state(State::High));
        let (mut spi, mut dc) = s.mocks();
        let mut busy = PinMock::new(&busy_tx);
        let mut epd = asynch::Uc8151::new(
            SpiInterface::new(spi.clone(), dc.clone()),
            busy.clone(),
            panel,
        );
        block_on(async {
            epd.init(&mut NoopDelay).await?;
            epd.update(&fb).await?;
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
