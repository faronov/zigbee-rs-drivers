//! Bit-banged standard-speed master on one open-drain GPIO.

use crate::{OneWire, asynch};
use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use zigbee_sensor::Error;

/// Standard-speed slot timing in µs: the recommended values A..J of
/// Maxim application note 126, table 2.
pub mod timing {
    /// Write-1 / read low pulse (1..15 µs allowed).
    pub const A: u32 = 6;
    /// Write-1 recovery to the end of the slot.
    pub const B: u32 = 64;
    /// Write-0 low pulse (60..120 µs allowed).
    pub const C: u32 = 60;
    /// Write-0 recovery.
    pub const D: u32 = 10;
    /// Read: release to sample (sample must be within 15 µs of the falling edge).
    pub const E: u32 = 9;
    /// Read: sample to end of slot.
    pub const F: u32 = 55;
    /// Reset low pulse (≥ 480 µs; > 960 µs may power-on-reset parasite devices).
    pub const H: u32 = 480;
    /// Reset: release to presence sample.
    pub const I: u32 = 70;
    /// Reset: presence sample to end of the reset sequence.
    pub const J: u32 = 410;
}

/// Runs a timing-critical slot with interrupts masked.
///
/// Implement it with your platform's critical section, e.g.
///
/// ```ignore
/// struct Cs;
/// impl zigbee_onewire::InterruptMask for Cs {
///     fn mask<R>(&mut self, f: impl FnOnce() -> R) -> R {
///         critical_section::with(|_| f())
///     }
/// }
/// ```
///
/// Each masked window is at most ~70 µs (one slot or the presence-detect
/// window).
pub trait InterruptMask {
    /// Run `f` with interrupts that could stretch the slot disabled.
    fn mask<R>(&mut self, f: impl FnOnce() -> R) -> R;
}

/// No masking: only correct if nothing can interrupt a slot (interrupts
/// already disabled by the caller, or a system without interrupts).
#[derive(Debug, Clone, Copy, Default)]
pub struct NoMask;

impl InterruptMask for NoMask {
    fn mask<R>(&mut self, f: impl FnOnce() -> R) -> R {
        f()
    }
}

/// Standard-speed 1-Wire master on an open-drain pin.
///
/// `pin` must be configured open-drain with a pull-up (4.7 kΩ to VDD is
/// typical): `set_low` pulls the bus low, `set_high` releases it, and the
/// input reads the bus level. `delay` must resolve microseconds (a cycle
/// counter or a busy loop, not a millisecond timer).
///
/// Implements both [`crate::OneWire`] and [`crate::asynch::OneWire`]. The
/// async implementation still uses the blocking `delay` inside each slot,
/// and yields to the executor after every reset and every byte. With
/// [`NoMask`] the caller must keep interrupts away during transactions.
///
/// There is no strong pull-up: parasite-powered devices need extra
/// hardware (see [`crate::OneWire::strong_pullup`]).
pub struct BitBang<P, D, M = NoMask> {
    pin: P,
    delay: D,
    mask: M,
}

impl<P, D> BitBang<P, D, NoMask> {
    /// Create a master without interrupt masking; no bus traffic.
    pub const fn new(pin: P, delay: D) -> Self {
        Self {
            pin,
            delay,
            mask: NoMask,
        }
    }
}

impl<P, D, M> BitBang<P, D, M> {
    /// Use `mask` around the timing-critical part of every slot.
    pub fn with_mask<M2: InterruptMask>(self, mask: M2) -> BitBang<P, D, M2> {
        BitBang {
            pin: self.pin,
            delay: self.delay,
            mask,
        }
    }

    /// Give the pin, delay and mask back.
    pub fn release(self) -> (P, D, M) {
        (self.pin, self.delay, self.mask)
    }
}

impl<P, D, M> BitBang<P, D, M>
where
    P: InputPin + OutputPin,
    D: DelayNs,
    M: InterruptMask,
{
    fn reset_slot(&mut self) -> Result<bool, Error<P::Error>> {
        let Self { pin, delay, mask } = self;
        pin.set_low().map_err(Error::bus)?;
        delay.delay_us(timing::H);
        let presence = mask
            .mask(|| {
                pin.set_high()?;
                delay.delay_us(timing::I);
                pin.is_low()
            })
            .map_err(Error::bus)?;
        delay.delay_us(timing::J);
        // Presence pulses end ≤ 300 µs after release; a low line now is a short.
        if pin.is_low().map_err(Error::bus)? {
            return Err(Error::InvalidData);
        }
        Ok(presence)
    }

    fn write_slot(&mut self, bit: bool) -> Result<(), Error<P::Error>> {
        let Self { pin, delay, mask } = self;
        let (low, recovery) = if bit {
            (timing::A, timing::B)
        } else {
            (timing::C, timing::D)
        };
        mask.mask(|| {
            pin.set_low()?;
            delay.delay_us(low);
            pin.set_high()
        })
        .map_err(Error::bus)?;
        delay.delay_us(recovery);
        Ok(())
    }

    fn read_slot(&mut self) -> Result<bool, Error<P::Error>> {
        let Self { pin, delay, mask } = self;
        let bit = mask
            .mask(|| {
                pin.set_low()?;
                delay.delay_us(timing::A);
                pin.set_high()?;
                delay.delay_us(timing::E);
                pin.is_high()
            })
            .map_err(Error::bus)?;
        delay.delay_us(timing::F);
        Ok(bit)
    }
}

impl<P, D, M> OneWire for BitBang<P, D, M>
where
    P: InputPin + OutputPin,
    D: DelayNs,
    M: InterruptMask,
{
    type BusError = P::Error;

    fn reset(&mut self) -> Result<bool, Error<P::Error>> {
        self.reset_slot()
    }

    fn write_bit(&mut self, bit: bool) -> Result<(), Error<P::Error>> {
        self.write_slot(bit)
    }

    fn read_bit(&mut self) -> Result<bool, Error<P::Error>> {
        self.read_slot()
    }
}

/// Returns `Pending` once so the executor can run other tasks.
struct YieldNow(bool);

impl Future for YieldNow {
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<()> {
        let this = self.get_mut();
        if this.0 {
            Poll::Ready(())
        } else {
            this.0 = true;
            cx.waker().wake_by_ref();
            Poll::Pending
        }
    }
}

impl<P, D, M> asynch::OneWire for BitBang<P, D, M>
where
    P: InputPin + OutputPin,
    D: DelayNs,
    M: InterruptMask,
{
    type BusError = P::Error;

    async fn reset(&mut self) -> Result<bool, Error<P::Error>> {
        let presence = self.reset_slot()?;
        YieldNow(false).await;
        Ok(presence)
    }

    async fn write_bit(&mut self, bit: bool) -> Result<(), Error<P::Error>> {
        self.write_slot(bit)
    }

    async fn read_bit(&mut self) -> Result<bool, Error<P::Error>> {
        self.read_slot()
    }

    async fn write_byte(&mut self, byte: u8) -> Result<(), Error<P::Error>> {
        for i in 0..8 {
            self.write_slot(byte >> i & 1 != 0)?;
        }
        YieldNow(false).await;
        Ok(())
    }

    async fn read_byte(&mut self) -> Result<u8, Error<P::Error>> {
        let mut byte = 0;
        for i in 0..8 {
            if self.read_slot()? {
                byte |= 1 << i;
            }
        }
        YieldNow(false).await;
        Ok(byte)
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::delay::{CheckedDelay, Transaction as D};
    use embedded_hal_mock::eh1::digital::{Mock as PinMock, State, Transaction as T};
    use std::vec::Vec;

    #[derive(Default)]
    struct CountingMask(usize);
    impl InterruptMask for CountingMask {
        fn mask<R>(&mut self, f: impl FnOnce() -> R) -> R {
            self.0 += 1;
            f()
        }
    }

    fn reset_pin(presence: bool, line_after: State) -> [T; 4] {
        [
            T::set(State::Low),
            T::set(State::High),
            T::get(if presence { State::Low } else { State::High }),
            T::get(line_after),
        ]
    }

    const RESET_DELAYS: [u32; 3] = [timing::H, timing::I, timing::J];

    fn delays(us: &[u32]) -> Vec<D> {
        us.iter().map(|&u| D::delay_us(u)).collect()
    }

    #[test]
    fn reset_detects_presence_with_an126_timing() {
        let mut pin = PinMock::new(&reset_pin(true, State::High));
        let mut delay = CheckedDelay::new(&delays(&RESET_DELAYS));
        let mut bus = BitBang::new(&mut pin, &mut delay).with_mask(CountingMask::default());
        assert_eq!(OneWire::reset(&mut bus), Ok(true));
        assert_eq!(bus.release().2.0, 1);
        pin.done();
        delay.done();
    }

    #[test]
    fn reset_without_presence_and_shorted_bus() {
        let mut expect = reset_pin(false, State::High).to_vec();
        expect.extend(reset_pin(true, State::Low));
        let mut pin = PinMock::new(&expect);
        let mut delay = CheckedDelay::new(&delays(&[RESET_DELAYS, RESET_DELAYS].concat()));
        let mut bus = BitBang::new(&mut pin, &mut delay);
        assert_eq!(OneWire::reset(&mut bus), Ok(false));
        assert_eq!(OneWire::reset(&mut bus), Err(Error::InvalidData));
        pin.done();
        delay.done();
    }

    #[test]
    fn write_byte_slots_lsb_first() {
        // 0x01: one "write 1" slot, then seven "write 0" slots.
        let mut pins = Vec::new();
        let mut us = Vec::new();
        for i in 0..8 {
            pins.extend([T::set(State::Low), T::set(State::High)]);
            if i == 0 {
                us.extend([timing::A, timing::B]);
            } else {
                us.extend([timing::C, timing::D]);
            }
        }
        let mut pin = PinMock::new(&pins);
        let mut delay = CheckedDelay::new(&delays(&us));
        let mut bus = BitBang::new(&mut pin, &mut delay).with_mask(CountingMask::default());
        OneWire::write_byte(&mut bus, 0x01).unwrap();
        assert_eq!(bus.release().2.0, 8);
        pin.done();
        delay.done();
    }

    #[test]
    fn read_byte_samples_inside_window() {
        // Bus returns 0xA5 = 1,0,1,0,0,1,0,1 (LSB first).
        let mut pins = Vec::new();
        let mut us = Vec::new();
        for i in 0..8 {
            let level = if 0xA5u8 >> i & 1 != 0 {
                State::High
            } else {
                State::Low
            };
            pins.extend([T::set(State::Low), T::set(State::High), T::get(level)]);
            us.extend([timing::A, timing::E, timing::F]);
        }
        let mut pin = PinMock::new(&pins);
        let mut delay = CheckedDelay::new(&delays(&us));
        let mut bus = BitBang::new(&mut pin, &mut delay);
        assert_eq!(OneWire::read_byte(&mut bus), Ok(0xA5));
        pin.done();
        delay.done();
    }

    #[test]
    fn async_byte_yields_once_to_executor() {
        use core::task::{Context, Poll, Waker};

        let mut pins = Vec::new();
        let mut us = Vec::new();
        for _ in 0..8 {
            pins.extend([T::set(State::Low), T::set(State::High)]);
            us.extend([timing::A, timing::B]);
        }
        let mut pin = PinMock::new(&pins);
        let mut delay = CheckedDelay::new(&delays(&us));
        let mut bus = BitBang::new(&mut pin, &mut delay);
        {
            let mut fut = core::pin::pin!(asynch::OneWire::write_byte(&mut bus, 0xFF));
            let mut cx = Context::from_waker(Waker::noop());
            let mut pending = 0;
            loop {
                match fut.as_mut().poll(&mut cx) {
                    Poll::Ready(r) => break r.unwrap(),
                    Poll::Pending => pending += 1,
                }
            }
            assert_eq!(pending, 1);
        }
        pin.done();
        delay.done();
    }
}
