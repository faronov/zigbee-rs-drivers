//! Shared pieces of the zigbee-rs display drivers.
//!
//! * [`blocking::Display`] / [`asynch::Display`]: the common
//!   init / refresh / busy / sleep API.
//! * [`interface`]: SPI + D/C and I2C command/data transports.
//! * [`frame`]: caller-owned frame buffers that implement the
//!   embedded-graphics [`DrawTarget`](embedded_graphics_core::draw_target::DrawTarget).
//!
//! # Not blocking the Zigbee stack
//!
//! An e-paper refresh takes 0.3–20 s and the controller signals completion
//! on its BUSY pin. Drivers never wait for it. Pushing an image is split
//! into the same kind of phases as a sensor measurement:
//!
//! 1. write the frame buffer into controller RAM (an SPI transfer, a few
//!    milliseconds);
//! 2. [`blocking::Display::start_refresh`] starts the waveform and returns
//!    the typical duration in microseconds ([`WaitUs`]);
//! 3. the application lets the stack run (timer, `await`, sleep) and then
//!    polls [`blocking::Display::is_busy`], or, with the async drivers,
//!    awaits the BUSY pin edge.
//!
//! LCDs and OLEDs show their RAM continuously; their `start_refresh`
//! returns `0` and `is_busy` is always `false`.
//!
//! Hardware reset and controller init use a short delay (tens of
//! milliseconds, documented per driver). Run `init` before the stack
//! starts or use the async driver.
//!
//! # Errors
//!
//! Drivers return [`DisplayError`], i.e. [`zigbee_sensor::Error`] over a
//! [`BusError`] that holds either the SPI/I2C error or a GPIO error kind:
//!
//! * `Error::NotReady`: the controller is busy (BUSY pin asserted); retry
//!   after the wait you were given.
//! * `Error::Pending(wait)`: a multi-phase operation (e.g. UC8151
//!   power-off before deep sleep) started its next phase; call again after
//!   `wait`.
//! * `Error::InvalidArgument`: frame size or area does not match the panel.
//! * `Error::Unsupported`: e.g. partial refresh on a tricolour panel.
//! * `Error::Timeout`: the BUSY wait helpers gave up.
//!
//! embedded-graphics (`embedded-graphics-core` only) is a normal dependency
//! of this crate and of the display drivers, never of `zigbee-sensor`, so
//! sensor-only firmware does not pull it in.

#![no_std]

pub mod frame;
pub mod interface;

pub use embedded_graphics_core;
pub use frame::{
    Area, MonoFrame, PageFrame, Rgb565Frame, Rotation, TriColor, TriFrame, mono_len, page_len,
    rgb565_len, tri_len,
};
pub use interface::{AsyncInterface, BusError, I2cInterface, Interface, NoPin, SpiInterface};
pub use zigbee_sensor::{Error, MAX_PHASES, WaitUs};

/// Error returned by every display driver.
pub type DisplayError<B> = Error<BusError<B>>;

/// Interval at which the BUSY wait helpers poll the pin.
pub const BUSY_POLL_US: WaitUs = 10_000;

/// Longest BUSY wait the helpers accept before returning
/// [`Error::Timeout`] (60 s, covers tricolour panels in the cold).
pub const BUSY_TIMEOUT_US: u32 = 60_000_000;

/// Map a GPIO error into a [`DisplayError`]. Usable as
/// `.map_err(zigbee_display::pin_error)`.
pub fn pin_error<B, E: embedded_hal::digital::Error>(error: E) -> DisplayError<B> {
    Error::Bus(BusError::pin(error))
}

/// Map a transport error into a [`DisplayError`]. Usable as
/// `.map_err(zigbee_display::bus_error)`.
pub fn bus_error<B>(error: BusError<B>) -> DisplayError<B> {
    Error::Bus(error)
}

/// Blocking (embedded-hal 1.0) display trait and helpers.
pub mod blocking {
    use super::{BUSY_POLL_US, BUSY_TIMEOUT_US, DisplayError, Error, MAX_PHASES, WaitUs};
    use embedded_hal::delay::DelayNs;

    /// Operations every display driver has.
    pub trait Display {
        /// Transport (SPI/I2C) error type.
        type BusError;

        /// Hardware reset (if a RST pin is connected) and controller
        /// initialisation. Also the way out of deep sleep for e-paper
        /// controllers. Uses `delay` for the reset pulse and power-up waits.
        fn init<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), DisplayError<Self::BusError>>;

        /// Make the controller RAM visible. E-paper: start the full-refresh
        /// waveform and return its typical duration; poll
        /// [`Self::is_busy`] afterwards. LCD/OLED: no-op, returns `0`.
        fn start_refresh(&mut self) -> Result<WaitUs, DisplayError<Self::BusError>>;

        /// `true` while the controller is refreshing (BUSY asserted).
        fn is_busy(&mut self) -> Result<bool, DisplayError<Self::BusError>>;

        /// Enter the lowest-power state that keeps the image. Returns
        /// [`Error::Pending`] when a power-down phase was started and
        /// `sleep` has to be called again after the wait. Leave it with
        /// [`Self::init`] (e-paper) or the driver's `wake`.
        fn sleep(&mut self) -> Result<(), DisplayError<Self::BusError>>;
    }

    /// Poll [`Display::is_busy`] every [`BUSY_POLL_US`] until it is
    /// released. Blocks; for tools and bring-up, not for a Zigbee event
    /// loop.
    pub fn wait_idle<T: Display, D: DelayNs>(
        display: &mut T,
        delay: &mut D,
    ) -> Result<(), DisplayError<T::BusError>> {
        for _ in 0..BUSY_TIMEOUT_US / BUSY_POLL_US {
            if !display.is_busy()? {
                return Ok(());
            }
            delay.delay_us(BUSY_POLL_US);
        }
        Err(Error::Timeout)
    }

    /// [`Display::start_refresh`], wait the returned time, then
    /// [`wait_idle`]. Blocks for the whole refresh.
    pub fn refresh<T: Display, D: DelayNs>(
        display: &mut T,
        delay: &mut D,
    ) -> Result<(), DisplayError<T::BusError>> {
        let wait = display.start_refresh()?;
        if wait != 0 {
            delay.delay_us(wait);
        }
        wait_idle(display, delay)
    }

    /// [`Display::sleep`], following [`Error::Pending`] phases. Blocks.
    pub fn sleep<T: Display, D: DelayNs>(
        display: &mut T,
        delay: &mut D,
    ) -> Result<(), DisplayError<T::BusError>> {
        for _ in 0..MAX_PHASES {
            match display.sleep() {
                Err(Error::Pending(wait)) => {
                    delay.delay_us(wait);
                    wait_idle(display, delay)?;
                }
                other => return other,
            }
        }
        Err(Error::Timeout)
    }
}

/// Async (embedded-hal-async 1.0) display trait.
pub mod asynch {
    use super::{BUSY_POLL_US, BUSY_TIMEOUT_US, DisplayError, Error, MAX_PHASES, WaitUs};
    use embedded_hal_async::delay::DelayNs;

    /// Async twin of [`crate::blocking::Display`].
    #[allow(async_fn_in_trait)]
    pub trait Display {
        /// Transport (SPI/I2C) error type.
        type BusError;

        /// See [`crate::blocking::Display::init`].
        async fn init<D: DelayNs>(
            &mut self,
            delay: &mut D,
        ) -> Result<(), DisplayError<Self::BusError>>;

        /// See [`crate::blocking::Display::start_refresh`].
        async fn start_refresh(&mut self) -> Result<WaitUs, DisplayError<Self::BusError>>;

        /// See [`crate::blocking::Display::is_busy`].
        async fn is_busy(&mut self) -> Result<bool, DisplayError<Self::BusError>>;

        /// See [`crate::blocking::Display::sleep`].
        async fn sleep(&mut self) -> Result<(), DisplayError<Self::BusError>>;

        /// Poll BUSY every [`BUSY_POLL_US`] on `delay` until released. The
        /// executor keeps running the Zigbee stack meanwhile. Drivers also
        /// offer an edge-triggered `wait_idle` when the BUSY pin implements
        /// `embedded_hal_async::digital::Wait`.
        async fn poll_idle<D: DelayNs>(
            &mut self,
            delay: &mut D,
        ) -> Result<(), DisplayError<Self::BusError>> {
            for _ in 0..BUSY_TIMEOUT_US / BUSY_POLL_US {
                if !self.is_busy().await? {
                    return Ok(());
                }
                delay.delay_us(BUSY_POLL_US).await;
            }
            Err(Error::Timeout)
        }

        /// Start a refresh, await its typical duration, then
        /// [`Self::poll_idle`].
        async fn refresh<D: DelayNs>(
            &mut self,
            delay: &mut D,
        ) -> Result<(), DisplayError<Self::BusError>> {
            let wait = self.start_refresh().await?;
            if wait != 0 {
                delay.delay_us(wait).await;
            }
            self.poll_idle(delay).await
        }

        /// [`Self::sleep`], following [`Error::Pending`] phases.
        async fn sleep_with<D: DelayNs>(
            &mut self,
            delay: &mut D,
        ) -> Result<(), DisplayError<Self::BusError>> {
            for _ in 0..MAX_PHASES {
                match self.sleep().await {
                    Err(Error::Pending(wait)) => {
                        delay.delay_us(wait).await;
                        self.poll_idle(delay).await?;
                    }
                    other => return other,
                }
            }
            Err(Error::Timeout)
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_graphics_core::draw_target::DrawTarget;
    use embedded_graphics_core::geometry::{OriginDimensions, Point, Size};
    use embedded_graphics_core::pixelcolor::{BinaryColor, Rgb565, RgbColor};
    use embedded_graphics_core::primitives::Rectangle;
    use embedded_hal_mock::eh1::digital::{Mock as PinMock, State, Transaction as PinTx};
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTx};
    use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiTx};
    use std::vec;
    use std::vec::Vec;

    #[test]
    fn buffer_lengths() {
        assert_eq!(mono_len(128, 128), 2048);
        assert_eq!(mono_len(122, 250), 16 * 250);
        assert_eq!(mono_len(400, 240), 12_000);
        assert_eq!(tri_len(128, 296), 9472);
        assert_eq!(page_len(128, 64), 1024);
        assert_eq!(page_len(128, 32), 512);
        assert_eq!(rgb565_len(240, 10), 4800);
    }

    #[test]
    fn mono_frame_layout_and_polarity() {
        let mut fb = MonoFrame::new([0xFFu8; 4], 16, 2).unwrap();
        fb.set_pixel(0, 0, BinaryColor::On);
        fb.set_pixel(9, 1, BinaryColor::On);
        assert_eq!(fb.as_bytes(), &[0x7F, 0xFF, 0xFF, 0xBF]);
        assert_eq!(fb.pixel(9, 1), Some(BinaryColor::On));
        assert_eq!(fb.pixel(8, 1), Some(BinaryColor::Off));
        assert_eq!(fb.pixel(16, 0), None);
        assert_eq!(fb.dirty(), Some(Area::new(0, 0, 10, 2)));
        fb.clear_dirty();
        // Writing the same value does not dirty the frame.
        fb.set_pixel(0, 0, BinaryColor::On);
        assert_eq!(fb.dirty(), None);
        assert!(MonoFrame::new([0u8; 3], 16, 2).is_none());
    }

    #[test]
    fn mono_frame_rotation() {
        let mut fb = MonoFrame::new([0xFFu8; 2 * 4], 16, 4)
            .unwrap()
            .with_rotation(Rotation::Deg90);
        assert_eq!(fb.size(), Size::new(4, 16));
        // Logical (0, 0) is the native top-right corner.
        fb.set_pixel(0, 0, BinaryColor::On);
        assert_eq!(fb.dirty(), Some(Area::new(15, 0, 1, 1)));
        fb.set_rotation(Rotation::Deg270);
        fb.clear_dirty();
        // Logical (0, 0) is the native bottom-left corner.
        fb.set_pixel(0, 0, BinaryColor::On);
        assert_eq!(fb.dirty(), Some(Area::new(0, 3, 1, 1)));
        fb.set_rotation(Rotation::Deg180);
        fb.clear_dirty();
        fb.set_pixel(0, 0, BinaryColor::On);
        assert_eq!(fb.dirty(), Some(Area::new(15, 3, 1, 1)));
    }

    #[test]
    fn mono_frame_draw_target() {
        let mut storage = [0u8; 4];
        let mut fb = MonoFrame::new(&mut storage[..], 16, 2).unwrap();
        fb.clear(BinaryColor::Off).unwrap();
        fb.fill_solid(
            &Rectangle::new(Point::new(4, 1), Size::new(8, 1)),
            BinaryColor::On,
        )
        .unwrap();
        assert_eq!(fb.as_bytes(), &[0xFF, 0xFF, 0xF0, 0x0F]);
        let rows: Vec<&[u8]> = fb.window_rows(Area::new(8, 0, 8, 2)).collect();
        assert_eq!(rows, vec![&[0xFF][..], &[0x0F][..]]);
    }

    #[test]
    fn tri_frame_planes() {
        let mut fb = TriFrame::new([0u8; 4], 8, 2).unwrap();
        fb.fill(TriColor::White);
        fb.set_pixel(0, 0, TriColor::Black);
        fb.set_pixel(1, 1, TriColor::Red);
        assert_eq!(fb.black_plane(), &[0x7F, 0xFF]);
        assert_eq!(fb.red_plane(), &[0x00, 0x40]);
        assert_eq!(fb.pixel(1, 1), Some(TriColor::Red));
        assert_eq!(fb.pixel(0, 0), Some(TriColor::Black));
        fb.set_pixel(1, 1, TriColor::White);
        assert_eq!(fb.red_plane(), &[0x00, 0x00]);
    }

    #[test]
    fn page_frame_layout() {
        let mut fb = PageFrame::new([0u8; 8], 4, 16).unwrap();
        fb.set_pixel(1, 0, BinaryColor::On);
        fb.set_pixel(2, 9, BinaryColor::On);
        assert_eq!(fb.as_bytes(), &[0, 0x01, 0, 0, 0, 0, 0x02, 0]);
        assert_eq!(fb.page(1, 1, 3), &[0x00, 0x02]);
        assert_eq!(fb.dirty(), Some(Area::new(1, 0, 2, 10)));
        assert_eq!(fb.pages(), 2);
    }

    #[test]
    fn rgb565_frame_is_big_endian() {
        let mut fb = Rgb565Frame::new([0u8; 8], 2, 2).unwrap();
        fb.set_pixel(1, 0, Rgb565::RED);
        assert_eq!(&fb.as_bytes()[2..4], &[0xF8, 0x00]);
        assert_eq!(fb.pixel(1, 0), Some(Rgb565::RED));
    }

    #[test]
    fn area_helpers() {
        let a = Area::new(3, 5, 7, 2);
        assert_eq!(a.align_x8(128), Area::new(0, 5, 16, 2));
        assert_eq!(
            Area::new(120, 0, 5, 1).align_x8(122),
            Area::new(120, 0, 2, 1)
        );
        assert_eq!(a.align_y8(64), Area::new(3, 0, 7, 8));
        assert_eq!(a.union(Area::new(0, 0, 1, 1)), Area::new(0, 0, 10, 7));
        assert_eq!(
            Area::new(10, 10, 10, 10).clip(15, 12),
            Some(Area::new(10, 10, 5, 2))
        );
        assert_eq!(Area::new(20, 0, 1, 1).clip(15, 12), None);
    }

    #[test]
    fn spi_interface_toggles_dc() {
        let spi_tx = [
            SpiTx::transaction_start(),
            SpiTx::write_vec(vec![0x12]),
            SpiTx::transaction_end(),
            SpiTx::transaction_start(),
            SpiTx::write_vec(vec![0x01, 0x02]),
            SpiTx::transaction_end(),
        ];
        let dc_tx = [PinTx::set(State::Low), PinTx::set(State::High)];
        let mut spi = SpiMock::new(&spi_tx);
        let mut dc = PinMock::new(&dc_tx);
        let mut di = SpiInterface::new(spi.clone(), dc.clone());
        Interface::command(&mut di, &[0x12]).unwrap();
        Interface::data(&mut di, &[0x01, 0x02]).unwrap();
        // Empty payloads cause no traffic.
        Interface::data(&mut di, &[]).unwrap();
        spi.done();
        dc.done();
    }

    #[test]
    fn i2c_interface_prefixes_control_byte_and_chunks() {
        let data: Vec<u8> = (0..40).collect();
        let mut first = vec![0x40];
        first.extend_from_slice(&data[..32]);
        let mut second = vec![0x40];
        second.extend_from_slice(&data[32..]);
        let tx = [
            I2cTx::write(0x3C, vec![0x00, 0xAE, 0xA8, 0x3F]),
            I2cTx::write(0x3C, first),
            I2cTx::write(0x3C, second),
        ];
        let mut i2c = I2cMock::new(&tx);
        let mut di = I2cInterface::new(i2c.clone(), 0x3C);
        Interface::command(&mut di, &[0xAE, 0xA8, 0x3F]).unwrap();
        Interface::data(&mut di, &data).unwrap();
        i2c.done();
    }

    #[test]
    fn async_i2c_interface() {
        let tx = [I2cTx::write(0x3D, vec![0x00, 0xAF])];
        let mut i2c = I2cMock::new(&tx);
        let mut di = I2cInterface::new(i2c.clone(), 0x3D);
        block_on(AsyncInterface::command(&mut di, &[0xAF])).unwrap();
        i2c.done();
    }

    /// Busy for `polls` reads, then idle.
    struct Fake {
        polls: u32,
        sleep_phases: u32,
    }

    impl blocking::Display for Fake {
        type BusError = ();
        fn init<D: embedded_hal::delay::DelayNs>(
            &mut self,
            _: &mut D,
        ) -> Result<(), DisplayError<()>> {
            Ok(())
        }
        fn start_refresh(&mut self) -> Result<WaitUs, DisplayError<()>> {
            Ok(1_000)
        }
        fn is_busy(&mut self) -> Result<bool, DisplayError<()>> {
            if self.polls == 0 {
                return Ok(false);
            }
            self.polls -= 1;
            Ok(true)
        }
        fn sleep(&mut self) -> Result<(), DisplayError<()>> {
            if self.sleep_phases == 0 {
                return Ok(());
            }
            self.sleep_phases -= 1;
            Err(Error::Pending(500))
        }
    }

    #[derive(Default)]
    struct CountDelay(u64);
    impl embedded_hal::delay::DelayNs for CountDelay {
        fn delay_ns(&mut self, ns: u32) {
            self.0 += u64::from(ns);
        }
    }

    #[test]
    fn refresh_helper_polls_busy() {
        let mut d = Fake {
            polls: 3,
            sleep_phases: 0,
        };
        let mut delay = CountDelay::default();
        blocking::refresh(&mut d, &mut delay).unwrap();
        assert_eq!(delay.0, 1_000_000 + 3 * u64::from(BUSY_POLL_US) * 1_000);
    }

    #[test]
    fn wait_idle_times_out() {
        let mut d = Fake {
            polls: u32::MAX,
            sleep_phases: 0,
        };
        let mut delay = CountDelay::default();
        assert_eq!(blocking::wait_idle(&mut d, &mut delay), Err(Error::Timeout));
    }

    #[test]
    fn sleep_helper_follows_pending() {
        let mut d = Fake {
            polls: 0,
            sleep_phases: 1,
        };
        let mut delay = CountDelay::default();
        blocking::sleep(&mut d, &mut delay).unwrap();
        assert_eq!(delay.0, 500_000);
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
