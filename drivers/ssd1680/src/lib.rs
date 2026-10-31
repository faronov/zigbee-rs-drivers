//! Solomon SSD1680 / SSD1681 / SSD1675 e-paper controllers: 2.13"
//! (122x250), 2.9" (128x296) and 1.54" (200x200) black/white panels, and
//! black/white/red panels.
//!
//! # Refresh without blocking
//!
//! A refresh runs for 0.3 s (partial) to ~15 s (tricolour) and BUSY stays
//! high meanwhile. No method here waits for it:
//!
//! ```ignore
//! epd.init(&mut delay)?;                     // ~30 ms of short waits
//! epd.write_frame(&fb)?;                     // SPI only
//! let wait = epd.start_full_refresh()?;      // returns typical µs
//! // ... return to the Zigbee stack; after `wait` poll:
//! if !epd.is_busy()? { epd.sleep()?; }
//! ```
//!
//! Every command first samples BUSY and returns [`Error::NotReady`] while
//! the panel is still refreshing. The async driver adds `wait_idle()`,
//! which awaits the BUSY falling edge.
//!
//! # Partial refresh (black/white panels)
//!
//! The controller computes the waveform per pixel from the *new* image in
//! RAM `0x24` and the *previous* image in RAM `0x26`:
//!
//! 1. [`Ssd1680::start_partial_refresh`]`(&fb, area)` writes the window to
//!    `0x24` and runs the partial waveform (OTP display mode 2 `0x22 0xFF`
//!    on SSD1680/SSD1681, or the register partial LUT on SSD1675);
//! 2. once BUSY is released, [`Ssd1680::finish_refresh`]`(&fb)` copies the
//!    same window into `0x26`, so it becomes the base of the next partial
//!    update. **Do not draw into the frame between the two calls.**
//!
//! Partial updates accumulate ghosting; [`Ssd1680::partial_count`] counts
//! them, and a full refresh every 5–10 partials is recommended. Tricolour
//! panels have no partial waveform ([`Error::Unsupported`]).
//!
//! References: Solomon Systech SSD1680 v0.14, SSD1681 v0.13 and SSD1675A
//! v0.11 datasheets (command tables, "typical operating sequence"). The
//! SSD1675 2.13" register LUTs and voltages are Waveshare's, taken from
//! the ISC-licensed `epd-waveshare` crate (see NOTICE).

#![no_std]

use embedded_hal::digital::{InputPin, OutputPin};
use zigbee_display::frame::window_rows;
use zigbee_display::{Area, DisplayError, Error, MonoFrame, NoPin, TriFrame, WaitUs, pin_error};

pub use zigbee_display;
pub use zigbee_display::SpiInterface;

/// Command bytes used by the driver.
pub mod cmd {
    /// Driver output control (gate count, scan).
    pub const DRIVER_OUTPUT: u8 = 0x01;
    /// Gate driving voltage.
    pub const GATE_VOLTAGE: u8 = 0x03;
    /// Source driving voltage.
    pub const SOURCE_VOLTAGE: u8 = 0x04;
    /// Deep sleep.
    pub const DEEP_SLEEP: u8 = 0x10;
    /// Data entry mode.
    pub const DATA_ENTRY: u8 = 0x11;
    /// Software reset.
    pub const SW_RESET: u8 = 0x12;
    /// Temperature sensor selection.
    pub const TEMP_SENSOR: u8 = 0x18;
    /// Master activation (run the update sequence).
    pub const MASTER_ACTIVATION: u8 = 0x20;
    /// Display update control 1 (RAM options, source range).
    pub const UPDATE_CONTROL_1: u8 = 0x21;
    /// Display update control 2 (update sequence).
    pub const UPDATE_CONTROL_2: u8 = 0x22;
    /// Write black/white RAM.
    pub const WRITE_RAM_BW: u8 = 0x24;
    /// Write red RAM (previous image on B/W panels).
    pub const WRITE_RAM_RED: u8 = 0x26;
    /// VCOM register.
    pub const VCOM: u8 = 0x2C;
    /// Write LUT register.
    pub const WRITE_LUT: u8 = 0x32;
    /// Dummy line period.
    pub const DUMMY_LINE: u8 = 0x3A;
    /// Gate line width.
    pub const GATE_LINE_WIDTH: u8 = 0x3B;
    /// Border waveform.
    pub const BORDER: u8 = 0x3C;
    /// RAM X start/end.
    pub const RAM_X_RANGE: u8 = 0x44;
    /// RAM Y start/end.
    pub const RAM_Y_RANGE: u8 = 0x45;
    /// RAM X counter.
    pub const RAM_X: u8 = 0x4E;
    /// RAM Y counter.
    pub const RAM_Y: u8 = 0x4F;
    /// SSD1675 analog block control.
    pub const ANALOG_BLOCK: u8 = 0x74;
    /// SSD1675 digital block control.
    pub const DIGITAL_BLOCK: u8 = 0x7E;
}

/// Display update control 2: full refresh with the OTP waveform (clock,
/// analog, load temperature + LUT, display mode 1, power down).
pub const UPDATE_FULL_OTP: u8 = 0xF7;
/// Display update control 2: partial refresh with the OTP waveform
/// (display mode 2).
pub const UPDATE_PARTIAL_OTP: u8 = 0xFF;
/// Display update control 2: refresh with the LUT in the register.
pub const UPDATE_REGISTER_LUT: u8 = 0xC7;

/// HW reset pulse and wait (10 ms each, as in the vendor sequences).
pub const RESET_US: u32 = 10_000;
/// Longest BUSY wait inside `init` (reset / SW reset take a few ms).
pub const INIT_BUSY_TIMEOUT_US: u32 = 500_000;
const INIT_POLL_US: u32 = 1_000;

/// Waveshare 2.13" V2 (SSD1675) full-refresh LUT, 70 bytes.
/// From `epd-waveshare` (ISC), see NOTICE.
#[rustfmt::skip]
pub const LUT_SSD1675_2IN13_FULL: [u8; 70] = [
    0x80, 0x60, 0x40, 0x00, 0x00, 0x00, 0x00,
    0x10, 0x60, 0x20, 0x00, 0x00, 0x00, 0x00,
    0x80, 0x60, 0x40, 0x00, 0x00, 0x00, 0x00,
    0x10, 0x60, 0x20, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x03, 0x03, 0x00, 0x00, 0x02,
    0x09, 0x09, 0x00, 0x00, 0x02,
    0x03, 0x03, 0x00, 0x00, 0x02,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
];

/// Waveshare 2.13" V2 (SSD1675) partial-refresh LUT, 70 bytes.
/// From `epd-waveshare` (ISC), see NOTICE.
#[rustfmt::skip]
pub const LUT_SSD1675_2IN13_PARTIAL: [u8; 70] = [
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x0A, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
];

/// Controller variant.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Controller {
    /// SSD1680: 176 sources x 296 gates, B/W/R, OTP waveforms.
    Ssd1680,
    /// SSD1681: 200 x 200, B/W/R, OTP waveforms.
    Ssd1681,
    /// SSD1675(A): B/W, waveforms loaded by the host (Waveshare 2.13" V2).
    Ssd1675,
}

/// Waveform source.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Waveform {
    /// Panel OTP: full `0x22 0xF7`, partial `0x22 0xFF` (display mode 2).
    /// Not every SSD1680/SSD1681 panel ships a mode-2 waveform; use
    /// `Custom` if partial updates look wrong.
    Otp,
    /// LUTs written with `0x32` before each refresh kind, refresh with
    /// `0x22 0xC7`. Voltages that go with the LUT can be set with
    /// [`Ssd1680::command`] after `init`.
    Custom {
        /// Full-refresh LUT.
        full: &'static [u8],
        /// Partial-refresh LUT; `None` makes partial refresh
        /// [`Error::Unsupported`].
        partial: Option<&'static [u8]>,
    },
}

/// Panel description.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Panel {
    controller: Controller,
    width: u16,
    height: u16,
    tricolor: bool,
    full_us: WaitUs,
    partial_us: WaitUs,
}

impl Panel {
    /// 2.13" 122x250 B/W (SSD1680: Waveshare 2.13" V3/V4, WeAct, GDEY0213B74).
    pub const SSD1680_2IN13: Panel = Panel::raw(Controller::Ssd1680, 122, 250, false);
    /// 2.13" 122x250 black/white/red (SSD1680).
    pub const SSD1680_2IN13_RED: Panel = Panel::raw(Controller::Ssd1680, 122, 250, true);
    /// 2.9" 128x296 B/W (SSD1680: Waveshare 2.9" V2, GDEY029T94).
    pub const SSD1680_2IN9: Panel = Panel::raw(Controller::Ssd1680, 128, 296, false);
    /// 2.9" 128x296 black/white/red (SSD1680).
    pub const SSD1680_2IN9_RED: Panel = Panel::raw(Controller::Ssd1680, 128, 296, true);
    /// 1.54" 200x200 B/W (SSD1681: Waveshare 1.54" V2, GDEH0154D67).
    pub const SSD1681_1IN54: Panel = Panel::raw(Controller::Ssd1681, 200, 200, false);
    /// 1.54" 200x200 black/white/red (SSD1681).
    pub const SSD1681_1IN54_RED: Panel = Panel::raw(Controller::Ssd1681, 200, 200, true);
    /// 2.13" 122x250 B/W (SSD1675: Waveshare 2.13" V2). Uses the bundled
    /// Waveshare LUTs.
    pub const SSD1675_2IN13: Panel = Panel::raw(Controller::Ssd1675, 122, 250, false);

    const fn raw(controller: Controller, width: u16, height: u16, tricolor: bool) -> Self {
        Self {
            controller,
            width,
            height,
            tricolor,
            full_us: if tricolor { 15_000_000 } else { 2_500_000 },
            partial_us: 400_000,
        }
    }

    /// Custom geometry: `width` ≤ 200 source pixels, `height` ≤ 300 gate
    /// lines (stay within the controller's limits). `None` if out of
    /// range.
    pub const fn new(
        controller: Controller,
        width: u16,
        height: u16,
        tricolor: bool,
    ) -> Option<Self> {
        if width == 0 || height == 0 || width > 200 || height > 300 {
            return None;
        }
        Some(Self::raw(controller, width, height, tricolor))
    }

    /// Override the typical refresh times returned as wait hints.
    pub const fn with_times(mut self, full_us: WaitUs, partial_us: WaitUs) -> Self {
        self.full_us = full_us;
        self.partial_us = partial_us;
        self
    }

    /// Controller variant.
    pub const fn controller(&self) -> Controller {
        self.controller
    }

    /// Width in pixels (source direction).
    pub const fn width(&self) -> u16 {
        self.width
    }

    /// Height in pixels (gate direction).
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

    /// Typical partial-refresh time (wait hint).
    pub const fn partial_refresh_us(&self) -> WaitUs {
        self.partial_us
    }

    /// Frame bytes: [`zigbee_display::mono_len`] or, for tricolour,
    /// [`zigbee_display::tri_len`].
    pub const fn frame_len(&self) -> usize {
        if self.tricolor {
            zigbee_display::tri_len(self.width, self.height)
        } else {
            zigbee_display::mono_len(self.width, self.height)
        }
    }

    const fn stride(&self) -> usize {
        (self.width as usize).div_ceil(8)
    }

    const fn default_waveform(&self) -> Waveform {
        match self.controller {
            Controller::Ssd1675 => Waveform::Custom {
                full: &LUT_SSD1675_2IN13_FULL,
                partial: Some(&LUT_SSD1675_2IN13_PARTIAL),
            },
            _ => Waveform::Otp,
        }
    }
}

/// Which register LUT is loaded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Lut {
    None,
    Full,
    Partial,
}

const ZEROS: [u8; 64] = [0; 64];

macro_rules! driver {
    ($Iface:path, $Delay:path, $($async:tt $await:tt)?) => {
        /// SSD1680 / SSD1681 / SSD1675 driver. BUSY (active high) is
        /// required; RST is optional.
        pub struct Ssd1680<DI, BUSY, RST = NoPin> {
            di: DI,
            busy: BUSY,
            rst: Option<RST>,
            panel: Panel,
            waveform: Waveform,
            lut: Lut,
            pending: Option<Area>,
            partials: u16,
        }

        impl<DI: $Iface, BUSY: InputPin> Ssd1680<DI, BUSY> {
            /// Driver without a reset pin; the waveform defaults to OTP
            /// (SSD1680/1681) or the bundled Waveshare LUTs (SSD1675).
            pub fn new(di: DI, busy: BUSY, panel: Panel) -> Self {
                Self {
                    di,
                    busy,
                    rst: None,
                    panel,
                    waveform: panel.default_waveform(),
                    lut: Lut::None,
                    pending: None,
                    partials: 0,
                }
            }
        }

        impl<DI: $Iface, BUSY: InputPin, RST: OutputPin> Ssd1680<DI, BUSY, RST> {
            /// Add the RES# pin, pulsed by [`Self::init`].
            pub fn with_reset<P: OutputPin>(self, rst: P) -> Ssd1680<DI, BUSY, P> {
                Ssd1680 {
                    di: self.di,
                    busy: self.busy,
                    rst: Some(rst),
                    panel: self.panel,
                    waveform: self.waveform,
                    lut: self.lut,
                    pending: self.pending,
                    partials: self.partials,
                }
            }

            /// Select the waveform source.
            pub fn with_waveform(mut self, waveform: Waveform) -> Self {
                self.waveform = waveform;
                self.lut = Lut::None;
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

            /// Send a raw command with parameters (BUSY must be low), e.g.
            /// voltage registers that go with a custom LUT.
            pub $($async)? fn command(&mut self, command: u8, data: &[u8]) -> Result<(), DisplayError<DI::Error>> {
                self.ensure_idle()?;
                self.cmd(command, data) $(.$await)?
            }

            $($async)? fn cmd(&mut self, command: u8, data: &[u8]) -> Result<(), DisplayError<DI::Error>> {
                self.di.command(&[command]) $(.$await)?.map_err(Error::Bus)?;
                self.di.data(data) $(.$await)?.map_err(Error::Bus)
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

            /// Hardware reset (if RST is connected), software reset and
            /// panel setup. Blocks for ~20 ms of reset pulses plus short
            /// BUSY polls (≤ [`INIT_BUSY_TIMEOUT_US`]). Also wakes the
            /// controller from deep sleep.
            pub $($async)? fn init<D: $Delay>(&mut self, delay: &mut D) -> Result<(), DisplayError<DI::Error>> {
                if let Some(rst) = self.rst.as_mut() {
                    rst.set_low().map_err(pin_error)?;
                    delay.delay_us(RESET_US) $(.$await)?;
                    rst.set_high().map_err(pin_error)?;
                    delay.delay_us(RESET_US) $(.$await)?;
                }
                self.wait_init(delay) $(.$await)??;
                self.cmd(cmd::SW_RESET, &[]) $(.$await)??;
                self.wait_init(delay) $(.$await)??;
                let p = self.panel;
                let last_gate = p.height - 1;
                if p.controller == Controller::Ssd1675 {
                    self.cmd(cmd::ANALOG_BLOCK, &[0x54]) $(.$await)??;
                    self.cmd(cmd::DIGITAL_BLOCK, &[0x3B]) $(.$await)??;
                }
                self.cmd(cmd::DRIVER_OUTPUT, &[last_gate as u8, (last_gate >> 8) as u8, 0x00]) $(.$await)??;
                self.cmd(cmd::DATA_ENTRY, &[0x03]) $(.$await)??;
                self.set_window(Area::full(p.width, p.height)) $(.$await)??;
                match p.controller {
                    Controller::Ssd1680 | Controller::Ssd1681 => {
                        self.cmd(cmd::BORDER, &[0x05]) $(.$await)??;
                        if p.controller == Controller::Ssd1680 {
                            self.cmd(cmd::UPDATE_CONTROL_1, &[0x00, 0x80]) $(.$await)??;
                        }
                        self.cmd(cmd::TEMP_SENSOR, &[0x80]) $(.$await)??;
                    }
                    Controller::Ssd1675 => {
                        // Waveshare 2.13" V2 voltages (epd-waveshare, ISC):
                        // VCOM -2.1 V, VGH 19 V, VSH1 15 V, VSH2 5 V, VSL -15 V.
                        self.cmd(cmd::BORDER, &[0x03]) $(.$await)??;
                        self.cmd(cmd::VCOM, &[0x54]) $(.$await)??;
                        self.cmd(cmd::GATE_VOLTAGE, &[0x15]) $(.$await)??;
                        self.cmd(cmd::SOURCE_VOLTAGE, &[0x41, 0xA8, 0x32]) $(.$await)??;
                        self.cmd(cmd::DUMMY_LINE, &[0x30]) $(.$await)??;
                        self.cmd(cmd::GATE_LINE_WIDTH, &[0x0A]) $(.$await)??;
                    }
                }
                self.lut = Lut::None;
                self.pending = None;
                self.load_lut(Lut::Full) $(.$await)??;
                self.wait_init(delay) $(.$await)?
            }

            $($async)? fn load_lut(&mut self, want: Lut) -> Result<(), DisplayError<DI::Error>> {
                let Waveform::Custom { full, partial } = self.waveform else {
                    return Ok(());
                };
                if self.lut == want {
                    return Ok(());
                }
                let (lut, border) = match want {
                    Lut::Partial => (partial.ok_or(Error::Unsupported)?, 0x01),
                    _ => (full, if self.panel.controller == Controller::Ssd1675 { 0x03 } else { 0x05 }),
                };
                self.cmd(cmd::WRITE_LUT, lut) $(.$await)??;
                self.cmd(cmd::BORDER, &[border]) $(.$await)??;
                self.lut = want;
                Ok(())
            }

            $($async)? fn set_window(&mut self, area: Area) -> Result<(), DisplayError<DI::Error>> {
                let x0 = (area.x / 8) as u8;
                let x1 = ((area.right() - 1) / 8) as u8;
                let y0 = area.y;
                let y1 = area.bottom() - 1;
                self.cmd(cmd::RAM_X_RANGE, &[x0, x1]) $(.$await)??;
                self.cmd(cmd::RAM_Y_RANGE, &[y0 as u8, (y0 >> 8) as u8, y1 as u8, (y1 >> 8) as u8]) $(.$await)??;
                self.cmd(cmd::RAM_X, &[x0]) $(.$await)??;
                self.cmd(cmd::RAM_Y, &[y0 as u8, (y0 >> 8) as u8]) $(.$await)?
            }

            /// Write `area` (byte-aligned horizontally) of a 1-bpp plane
            /// into RAM `ram`.
            $($async)? fn write_plane(&mut self, ram: u8, plane: &[u8], area: Area) -> Result<(), DisplayError<DI::Error>> {
                let stride = self.panel.stride();
                self.set_window(area) $(.$await)??;
                self.di.command(&[ram]) $(.$await)?.map_err(Error::Bus)?;
                let first = usize::from(area.x / 8);
                let last = usize::from(area.right()).div_ceil(8);
                if first == 0 && last == stride {
                    let rows = &plane[usize::from(area.y) * stride..usize::from(area.bottom()) * stride];
                    self.di.data(rows) $(.$await)?.map_err(Error::Bus)
                } else {
                    for row in window_rows(plane, stride, area) {
                        self.di.data(row) $(.$await)?.map_err(Error::Bus)?;
                    }
                    Ok(())
                }
            }

            $($async)? fn write_zeros(&mut self, ram: u8) -> Result<(), DisplayError<DI::Error>> {
                let p = self.panel;
                self.set_window(Area::full(p.width, p.height)) $(.$await)??;
                self.di.command(&[ram]) $(.$await)?.map_err(Error::Bus)?;
                let mut left = p.stride() * usize::from(p.height);
                while left > 0 {
                    let n = left.min(ZEROS.len());
                    self.di.data(&ZEROS[..n]) $(.$await)?.map_err(Error::Bus)?;
                    left -= n;
                }
                Ok(())
            }

            fn check_mono<B: AsRef<[u8]>>(&self, fb: &MonoFrame<B>) -> Result<(), DisplayError<DI::Error>> {
                if fb.width() != self.panel.width || fb.height() != self.panel.height {
                    return Err(Error::InvalidArgument);
                }
                Ok(())
            }

            /// Write a black/white frame into both RAMs (new and previous
            /// image). On a tricolour panel the red RAM is cleared instead.
            pub $($async)? fn write_frame<B: AsRef<[u8]>>(&mut self, fb: &MonoFrame<B>) -> Result<(), DisplayError<DI::Error>> {
                self.check_mono(fb)?;
                self.ensure_idle()?;
                let full = fb.full_area();
                self.write_plane(cmd::WRITE_RAM_BW, fb.as_bytes(), full) $(.$await)??;
                if self.panel.tricolor {
                    self.write_zeros(cmd::WRITE_RAM_RED) $(.$await)??;
                } else {
                    self.write_plane(cmd::WRITE_RAM_RED, fb.as_bytes(), full) $(.$await)??;
                }
                self.pending = None;
                Ok(())
            }

            /// Write a black/white/red frame (tricolour panels only).
            pub $($async)? fn write_tri_frame<B: AsRef<[u8]>>(&mut self, fb: &TriFrame<B>) -> Result<(), DisplayError<DI::Error>> {
                if !self.panel.tricolor {
                    return Err(Error::Unsupported);
                }
                if fb.width() != self.panel.width || fb.height() != self.panel.height {
                    return Err(Error::InvalidArgument);
                }
                self.ensure_idle()?;
                let full = fb.full_area();
                self.write_plane(cmd::WRITE_RAM_BW, fb.black_plane(), full) $(.$await)??;
                self.write_plane(cmd::WRITE_RAM_RED, fb.red_plane(), full) $(.$await)??;
                self.pending = None;
                Ok(())
            }

            $($async)? fn activate(&mut self, mode: u8) -> Result<(), DisplayError<DI::Error>> {
                self.cmd(cmd::UPDATE_CONTROL_2, &[mode]) $(.$await)??;
                self.cmd(cmd::MASTER_ACTIVATION, &[]) $(.$await)?
            }

            /// Start a full refresh of what is in RAM. Returns the typical
            /// duration; poll [`Self::is_busy`] afterwards.
            pub $($async)? fn start_full_refresh(&mut self) -> Result<WaitUs, DisplayError<DI::Error>> {
                self.ensure_idle()?;
                let mode = match self.waveform {
                    Waveform::Otp => UPDATE_FULL_OTP,
                    Waveform::Custom { .. } => {
                        self.load_lut(Lut::Full) $(.$await)??;
                        UPDATE_REGISTER_LUT
                    }
                };
                self.activate(mode) $(.$await)??;
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

            /// Write `area` of `fb` (widened to byte boundaries) to the new
            /// image RAM and start a partial refresh. Call
            /// [`Self::finish_refresh`] with the unchanged frame once BUSY
            /// is released. B/W panels only.
            pub $($async)? fn start_partial_refresh<B: AsRef<[u8]>>(
                &mut self,
                fb: &MonoFrame<B>,
                area: Area,
            ) -> Result<WaitUs, DisplayError<DI::Error>> {
                if self.panel.tricolor {
                    return Err(Error::Unsupported);
                }
                self.check_mono(fb)?;
                let p = self.panel;
                let Some(area) = area.clip(p.width, p.height) else {
                    return Err(Error::InvalidArgument);
                };
                let area = area.align_x8(p.width);
                self.ensure_idle()?;
                let mode = match self.waveform {
                    Waveform::Otp => {
                        self.cmd(cmd::BORDER, &[0x80]) $(.$await)??;
                        UPDATE_PARTIAL_OTP
                    }
                    Waveform::Custom { .. } => {
                        self.load_lut(Lut::Partial) $(.$await)??;
                        UPDATE_REGISTER_LUT
                    }
                };
                self.write_plane(cmd::WRITE_RAM_BW, fb.as_bytes(), area) $(.$await)??;
                self.activate(mode) $(.$await)??;
                if self.waveform == Waveform::Otp {
                    // Restore the full-refresh border for the next full update.
                    self.cmd(cmd::BORDER, &[0x05]) $(.$await)??;
                }
                self.pending = Some(area);
                self.partials = self.partials.saturating_add(1);
                Ok(p.partial_us)
            }

            /// Partial refresh of the frame's dirty rectangle (clears it).
            /// `Ok(None)` if nothing was dirty.
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

            /// After a partial refresh: copy the refreshed window into the
            /// previous-image RAM. No-op after a full refresh. Returns
            /// [`Error::NotReady`] while BUSY is still high.
            pub $($async)? fn finish_refresh<B: AsRef<[u8]>>(&mut self, fb: &MonoFrame<B>) -> Result<(), DisplayError<DI::Error>> {
                let Some(area) = self.pending else {
                    return Ok(());
                };
                self.check_mono(fb)?;
                self.ensure_idle()?;
                self.write_plane(cmd::WRITE_RAM_RED, fb.as_bytes(), area) $(.$await)??;
                self.pending = None;
                Ok(())
            }

            /// Deep sleep mode 1 (RAM retained, ~1 µA). Wake with
            /// [`Self::init`] (needs RST, or a power cycle).
            pub $($async)? fn sleep(&mut self) -> Result<(), DisplayError<DI::Error>> {
                self.ensure_idle()?;
                self.cmd(cmd::DEEP_SLEEP, &[0x01]) $(.$await)?
            }
        }
    };
}

driver!(zigbee_display::Interface, embedded_hal::delay::DelayNs,);

impl<DI, BUSY, RST> zigbee_display::blocking::Display for Ssd1680<DI, BUSY, RST>
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
        Ssd1680::init(self, delay)
    }

    fn start_refresh(&mut self) -> Result<WaitUs, DisplayError<DI::Error>> {
        self.start_full_refresh()
    }

    fn is_busy(&mut self) -> Result<bool, DisplayError<DI::Error>> {
        Ssd1680::is_busy(self)
    }

    fn sleep(&mut self) -> Result<(), DisplayError<DI::Error>> {
        Ssd1680::sleep(self)
    }
}

/// Async driver over [`zigbee_display::AsyncInterface`].
pub mod asynch {
    use super::*;

    driver!(zigbee_display::AsyncInterface, embedded_hal_async::delay::DelayNs, async await);

    impl<DI, BUSY, RST> Ssd1680<DI, BUSY, RST>
    where
        DI: zigbee_display::AsyncInterface,
        BUSY: InputPin + embedded_hal_async::digital::Wait,
        RST: OutputPin,
    {
        /// Await the BUSY pin going low (edge/level interrupt, no polling).
        pub async fn wait_idle(&mut self) -> Result<(), DisplayError<DI::Error>> {
            self.busy.wait_for_low().await.map_err(pin_error)
        }
    }

    impl<DI, BUSY, RST> zigbee_display::asynch::Display for Ssd1680<DI, BUSY, RST>
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
            Ssd1680::init(self, delay).await
        }

        async fn start_refresh(&mut self) -> Result<WaitUs, DisplayError<DI::Error>> {
            self.start_full_refresh().await
        }

        async fn is_busy(&mut self) -> Result<bool, DisplayError<DI::Error>> {
            Ssd1680::is_busy(self)
        }

        async fn sleep(&mut self) -> Result<(), DisplayError<DI::Error>> {
            Ssd1680::sleep(self).await
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

    /// Expected bus traffic: commands (DC low) and data (DC high).
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
        fn window(&mut self, x0: u8, x1: u8, y0: u16, y1: u16) -> &mut Self {
            self.cmd(0x44, &[x0, x1])
                .cmd(
                    0x45,
                    &[y0 as u8, (y0 >> 8) as u8, y1 as u8, (y1 >> 8) as u8],
                )
                .cmd(0x4E, &[x0])
                .cmd(0x4F, &[y0 as u8, (y0 >> 8) as u8])
        }
        fn mocks(&self) -> (SpiMock<u8>, PinMock) {
            (SpiMock::new(&self.spi), PinMock::new(&self.dc))
        }
    }

    fn idle(n: usize) -> Vec<PinTx> {
        vec![PinTx::get(State::Low); n]
    }

    #[test]
    fn ssd1680_init_sequence() {
        let panel = Panel::new(Controller::Ssd1680, 16, 300, false).unwrap();
        let mut s = Script::default();
        s.cmd(0x12, &[])
            .cmd(0x01, &[0x2B, 0x01, 0x00])
            .cmd(0x11, &[0x03])
            .window(0, 1, 0, 299)
            .cmd(0x3C, &[0x05])
            .cmd(0x21, &[0x00, 0x80])
            .cmd(0x18, &[0x80]);
        let (mut spi, mut dc) = s.mocks();
        // BUSY: high once after reset, then idle.
        let mut busy_tx = vec![PinTx::get(State::High)];
        busy_tx.extend(idle(3));
        let mut busy = PinMock::new(&busy_tx);
        let mut rst = PinMock::new(&[PinTx::set(State::Low), PinTx::set(State::High)]);
        let mut epd = Ssd1680::new(
            SpiInterface::new(spi.clone(), dc.clone()),
            busy.clone(),
            panel,
        )
        .with_reset(rst.clone());
        epd.init(&mut NoopDelay).unwrap();
        spi.done();
        dc.done();
        busy.done();
        rst.done();
    }

    #[test]
    fn ssd1675_init_loads_waveshare_lut() {
        let mut s = Script::default();
        s.cmd(0x12, &[])
            .cmd(0x74, &[0x54])
            .cmd(0x7E, &[0x3B])
            .cmd(0x01, &[0xF9, 0x00, 0x00])
            .cmd(0x11, &[0x03])
            .window(0, 15, 0, 249)
            .cmd(0x3C, &[0x03])
            .cmd(0x2C, &[0x54])
            .cmd(0x03, &[0x15])
            .cmd(0x04, &[0x41, 0xA8, 0x32])
            .cmd(0x3A, &[0x30])
            .cmd(0x3B, &[0x0A])
            .cmd(0x32, &LUT_SSD1675_2IN13_FULL)
            .cmd(0x3C, &[0x03]);
        let (mut spi, mut dc) = s.mocks();
        let mut busy = PinMock::new(&idle(3));
        let mut epd = Ssd1680::new(
            SpiInterface::new(spi.clone(), dc.clone()),
            busy.clone(),
            Panel::SSD1675_2IN13,
        );
        epd.init(&mut NoopDelay).unwrap();
        spi.done();
        dc.done();
        busy.done();
    }

    #[test]
    fn full_update_writes_both_rams_and_returns_wait() {
        let panel = Panel::new(Controller::Ssd1681, 16, 2, false).unwrap();
        let mut fb = MonoFrame::new([0xFFu8; 4], 16, 2).unwrap();
        fb.set_pixel(0, 0, BinaryColor::On);
        let mut s = Script::default();
        s.window(0, 1, 0, 1)
            .cmd(0x24, &[0x7F, 0xFF, 0xFF, 0xFF])
            .window(0, 1, 0, 1)
            .cmd(0x26, &[0x7F, 0xFF, 0xFF, 0xFF])
            .cmd(0x22, &[0xF7])
            .cmd(0x20, &[]);
        let (mut spi, mut dc) = s.mocks();
        let mut busy = PinMock::new(&idle(2));
        let mut epd = Ssd1680::new(
            SpiInterface::new(spi.clone(), dc.clone()),
            busy.clone(),
            panel,
        );
        assert_eq!(epd.update(&fb).unwrap(), 2_500_000);
        spi.done();
        dc.done();
        busy.done();
    }

    #[test]
    fn busy_panel_returns_not_ready() {
        let fb = MonoFrame::new([0xFFu8; 4], 16, 2).unwrap();
        let panel = Panel::new(Controller::Ssd1680, 16, 2, false).unwrap();
        let (mut spi, mut dc) = Script::default().mocks();
        let mut busy = PinMock::new(&[
            PinTx::get(State::High),
            PinTx::get(State::High),
            PinTx::get(State::High),
        ]);
        let mut epd = Ssd1680::new(
            SpiInterface::new(spi.clone(), dc.clone()),
            busy.clone(),
            panel,
        );
        assert_eq!(epd.write_frame(&fb), Err(Error::NotReady));
        assert_eq!(epd.start_full_refresh(), Err(Error::NotReady));
        assert!(zigbee_display::blocking::Display::is_busy(&mut epd).unwrap());
        spi.done();
        dc.done();
        busy.done();
    }

    #[test]
    fn partial_refresh_window_then_finish_copies_to_previous_ram() {
        let panel = Panel::new(Controller::Ssd1680, 24, 4, false).unwrap();
        let mut fb = MonoFrame::new([0xFFu8; 12], 24, 4).unwrap();
        fb.set_pixel(9, 2, BinaryColor::On);
        let mut s = Script::default();
        s.cmd(0x3C, &[0x80])
            .window(1, 1, 2, 2)
            .write(false, &[0x24])
            .write(true, &[0xBF])
            .cmd(0x22, &[0xFF])
            .cmd(0x20, &[])
            .cmd(0x3C, &[0x05])
            // finish_refresh
            .window(1, 1, 2, 2)
            .write(false, &[0x26])
            .write(true, &[0xBF]);
        let (mut spi, mut dc) = s.mocks();
        // partial: idle; finish: busy once (NotReady), then idle.
        let mut busy = PinMock::new(&[
            PinTx::get(State::Low),
            PinTx::get(State::High),
            PinTx::get(State::Low),
        ]);
        let mut epd = Ssd1680::new(
            SpiInterface::new(spi.clone(), dc.clone()),
            busy.clone(),
            panel,
        );
        assert_eq!(
            epd.start_partial_refresh_dirty(&mut fb).unwrap(),
            Some(400_000)
        );
        assert_eq!(epd.partial_count(), 1);
        assert_eq!(epd.finish_refresh(&fb), Err(Error::NotReady));
        epd.finish_refresh(&fb).unwrap();
        // Nothing pending any more.
        epd.finish_refresh(&fb).unwrap();
        spi.done();
        dc.done();
        busy.done();
    }

    #[test]
    fn ssd1675_partial_switches_lut_and_back() {
        let panel = Panel::new(Controller::Ssd1675, 8, 1, false).unwrap();
        let fb = MonoFrame::new([0x00u8], 8, 1).unwrap();
        let mut s = Script::default();
        s.cmd(0x32, &LUT_SSD1675_2IN13_PARTIAL)
            .cmd(0x3C, &[0x01])
            .window(0, 0, 0, 0)
            .cmd(0x24, &[0x00])
            .cmd(0x22, &[0xC7])
            .cmd(0x20, &[])
            .cmd(0x32, &LUT_SSD1675_2IN13_FULL)
            .cmd(0x3C, &[0x03])
            .cmd(0x22, &[0xC7])
            .cmd(0x20, &[]);
        let (mut spi, mut dc) = s.mocks();
        let mut busy = PinMock::new(&idle(2));
        let mut epd = Ssd1680::new(
            SpiInterface::new(spi.clone(), dc.clone()),
            busy.clone(),
            panel,
        );
        epd.start_partial_refresh(&fb, fb.full_area()).unwrap();
        epd.start_full_refresh().unwrap();
        assert_eq!(epd.partial_count(), 0);
        spi.done();
        dc.done();
        busy.done();
    }

    #[test]
    fn tricolor_planes_and_no_partial() {
        let panel = Panel::new(Controller::Ssd1680, 8, 2, true).unwrap();
        let mut fb = TriFrame::new([0u8; 4], 8, 2).unwrap();
        fb.fill(TriColor::White);
        fb.set_pixel(0, 0, TriColor::Black);
        fb.set_pixel(7, 1, TriColor::Red);
        let mut s = Script::default();
        s.window(0, 0, 0, 1)
            .cmd(0x24, &[0x7F, 0xFF])
            .window(0, 0, 0, 1)
            .cmd(0x26, &[0x00, 0x01])
            .cmd(0x22, &[0xF7])
            .cmd(0x20, &[]);
        let (mut spi, mut dc) = s.mocks();
        let mut busy = PinMock::new(&idle(2));
        let mut epd = Ssd1680::new(
            SpiInterface::new(spi.clone(), dc.clone()),
            busy.clone(),
            panel,
        );
        assert_eq!(epd.update_tri(&fb).unwrap(), 15_000_000);
        let mono = MonoFrame::new([0u8; 2], 8, 2).unwrap();
        assert_eq!(
            epd.start_partial_refresh(&mono, mono.full_area()),
            Err(Error::Unsupported)
        );
        spi.done();
        dc.done();
        busy.done();
    }

    #[test]
    fn deep_sleep_and_blocking_refresh_helper() {
        let panel = Panel::new(Controller::Ssd1681, 8, 1, false).unwrap();
        let mut s = Script::default();
        s.cmd(0x22, &[0xF7]).cmd(0x20, &[]).cmd(0x10, &[0x01]);
        let (mut spi, mut dc) = s.mocks();
        // refresh: idle check, busy twice while polling, then idle; sleep: idle.
        let mut busy = PinMock::new(&[
            PinTx::get(State::Low),
            PinTx::get(State::High),
            PinTx::get(State::High),
            PinTx::get(State::Low),
            PinTx::get(State::Low),
        ]);
        let mut epd = Ssd1680::new(
            SpiInterface::new(spi.clone(), dc.clone()),
            busy.clone(),
            panel,
        );
        zigbee_display::blocking::refresh(&mut epd, &mut NoopDelay).unwrap();
        epd.sleep().unwrap();
        spi.done();
        dc.done();
        busy.done();
    }

    #[test]
    fn panel_ram() {
        assert_eq!(Panel::SSD1680_2IN13.frame_len(), 4000);
        assert_eq!(Panel::SSD1680_2IN9.frame_len(), 4736);
        assert_eq!(Panel::SSD1681_1IN54.frame_len(), 5000);
        assert_eq!(Panel::SSD1680_2IN9_RED.frame_len(), 9472);
        assert_eq!(Panel::SSD1681_1IN54_RED.frame_len(), 10_000);
    }

    #[test]
    fn async_refresh_and_wait_idle() {
        let panel = Panel::new(Controller::Ssd1681, 8, 1, false).unwrap();
        let fb = MonoFrame::new([0xF0u8], 8, 1).unwrap();
        let mut s = Script::default();
        s.window(0, 0, 0, 0)
            .cmd(0x24, &[0xF0])
            .window(0, 0, 0, 0)
            .cmd(0x26, &[0xF0])
            .cmd(0x22, &[0xF7])
            .cmd(0x20, &[]);
        let (mut spi, mut dc) = s.mocks();
        let mut busy = PinMock::new(&[
            PinTx::get(State::Low),
            PinTx::get(State::Low),
            PinTx::wait_for_state(State::Low),
        ]);
        let mut epd = asynch::Ssd1680::new(
            SpiInterface::new(spi.clone(), dc.clone()),
            busy.clone(),
            panel,
        );
        block_on(async {
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
