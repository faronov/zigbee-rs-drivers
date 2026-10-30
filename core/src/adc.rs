//! Minimal ADC abstraction.
//!
//! embedded-hal 1.0 has no ADC trait. Analog drivers (NTC thermistors,
//! battery dividers, capacitive soil probes) take any type implementing
//! these traits; each platform HAL provides the implementation.

/// One analog channel read as a calibrated voltage.
pub trait Millivolts {
    /// Transport/ADC error type.
    type Error;

    /// Sample the channel and return the input voltage in mV.
    fn read_mv(&mut self) -> Result<u32, Self::Error>;
}

/// Async twin of [`Millivolts`].
#[allow(async_fn_in_trait)]
pub trait AsyncMillivolts {
    /// Transport/ADC error type.
    type Error;

    /// Sample the channel and return the input voltage in mV.
    async fn read_mv(&mut self) -> Result<u32, Self::Error>;
}

impl<T: Millivolts + ?Sized> Millivolts for &mut T {
    type Error = T::Error;

    fn read_mv(&mut self) -> Result<u32, Self::Error> {
        T::read_mv(self)
    }
}

impl<T: AsyncMillivolts + ?Sized> AsyncMillivolts for &mut T {
    type Error = T::Error;

    async fn read_mv(&mut self) -> Result<u32, Self::Error> {
        T::read_mv(self).await
    }
}
