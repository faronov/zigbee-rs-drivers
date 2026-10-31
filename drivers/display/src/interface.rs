//! Command/data transports shared by the display drivers.
//!
//! Most display controllers distinguish command bytes from data bytes:
//!
//! * 4-wire SPI ([`SpiInterface`]): a D/C GPIO, low for command bytes and
//!   high for data bytes. This is the same on every controller in this
//!   repository (SSD1306, SH1106, SSD1680/1681/1675, UC8151/IL0373,
//!   IL3820/IL3895, ST7789/ST7735). Chip select is handled by the
//!   [`embedded_hal::spi::SpiDevice`].
//! * I2C ([`I2cInterface`], SSD1306/SH1106 only): a control byte before
//!   the payload, `0x00` for a command stream and `0x40` for a data stream.
//!
//! Drivers are generic over [`Interface`] (blocking) or [`AsyncInterface`].
//! 3-wire (9-bit) SPI is not supported.

use embedded_hal::digital::{self, ErrorKind, OutputPin};

/// Error of a display transport: the SPI/I2C bus, or one of the GPIOs
/// (D/C, RST, CS, BUSY, DISP, EXTCOMIN).
///
/// GPIO errors are reduced to their [`ErrorKind`], so drivers whose pins
/// have different error types still have one error type. Most HALs have
/// infallible GPIOs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BusError<B> {
    /// The SPI or I2C transfer failed.
    Bus(B),
    /// A GPIO operation failed.
    Pin(ErrorKind),
}

impl<B> BusError<B> {
    /// Map a GPIO error. Usable as `.map_err(BusError::pin)`.
    pub fn pin<E: digital::Error>(error: E) -> Self {
        Self::Pin(error.kind())
    }
}

/// Blocking command/data transport.
pub trait Interface {
    /// Bus (SPI/I2C) error type.
    type Error;

    /// Send command bytes (D/C low, or I2C control byte `0x00`).
    fn command(&mut self, bytes: &[u8]) -> Result<(), BusError<Self::Error>>;

    /// Send data bytes (D/C high, or I2C control byte `0x40`).
    fn data(&mut self, bytes: &[u8]) -> Result<(), BusError<Self::Error>>;
}

/// Async command/data transport; see [`Interface`].
#[allow(async_fn_in_trait)]
pub trait AsyncInterface {
    /// Bus (SPI/I2C) error type.
    type Error;

    /// Send command bytes (D/C low, or I2C control byte `0x00`).
    async fn command(&mut self, bytes: &[u8]) -> Result<(), BusError<Self::Error>>;

    /// Send data bytes (D/C high, or I2C control byte `0x40`).
    async fn data(&mut self, bytes: &[u8]) -> Result<(), BusError<Self::Error>>;
}

/// 4-wire SPI with a D/C pin (low = command, high = data).
///
/// Every call to [`Interface::command`] / [`Interface::data`] is one
/// `SpiDevice` transaction, so chip select toggles between command and
/// parameters. All supported controllers accept that.
pub struct SpiInterface<SPI, DC> {
    spi: SPI,
    dc: DC,
}

impl<SPI, DC> SpiInterface<SPI, DC> {
    /// Wrap an SPI device and the D/C pin; no bus traffic.
    pub const fn new(spi: SPI, dc: DC) -> Self {
        Self { spi, dc }
    }

    /// Give the SPI device and the D/C pin back.
    pub fn release(self) -> (SPI, DC) {
        (self.spi, self.dc)
    }
}

impl<SPI: embedded_hal::spi::SpiDevice, DC: OutputPin> Interface for SpiInterface<SPI, DC> {
    type Error = SPI::Error;

    fn command(&mut self, bytes: &[u8]) -> Result<(), BusError<SPI::Error>> {
        if bytes.is_empty() {
            return Ok(());
        }
        self.dc.set_low().map_err(BusError::pin)?;
        self.spi.write(bytes).map_err(BusError::Bus)
    }

    fn data(&mut self, bytes: &[u8]) -> Result<(), BusError<SPI::Error>> {
        if bytes.is_empty() {
            return Ok(());
        }
        self.dc.set_high().map_err(BusError::pin)?;
        self.spi.write(bytes).map_err(BusError::Bus)
    }
}

impl<SPI: embedded_hal_async::spi::SpiDevice, DC: OutputPin> AsyncInterface
    for SpiInterface<SPI, DC>
{
    type Error = SPI::Error;

    async fn command(&mut self, bytes: &[u8]) -> Result<(), BusError<SPI::Error>> {
        if bytes.is_empty() {
            return Ok(());
        }
        self.dc.set_low().map_err(BusError::pin)?;
        self.spi.write(bytes).await.map_err(BusError::Bus)
    }

    async fn data(&mut self, bytes: &[u8]) -> Result<(), BusError<SPI::Error>> {
        if bytes.is_empty() {
            return Ok(());
        }
        self.dc.set_high().map_err(BusError::pin)?;
        self.spi.write(bytes).await.map_err(BusError::Bus)
    }
}

/// Payload bytes per I2C write. Each write carries one control byte in
/// front, so a 1024-byte SSD1306 frame takes 32 writes of 33 bytes.
pub const I2C_CHUNK: usize = 32;

const I2C_CONTROL_COMMAND: u8 = 0x00;
const I2C_CONTROL_DATA: u8 = 0x40;

/// I2C transport with a control byte (`0x00` command stream, `0x40` data
/// stream), as used by SSD1306 and SH1106.
///
/// Payloads are split into writes of at most [`I2C_CHUNK`] bytes, each
/// prefixed with the control byte, so no HAL support for merged
/// `transaction` writes is needed.
pub struct I2cInterface<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C> I2cInterface<I2C> {
    /// Wrap an I2C bus; no bus traffic.
    pub const fn new(i2c: I2C, address: u8) -> Self {
        Self { i2c, address }
    }

    /// 7-bit I2C address in use.
    pub const fn address(&self) -> u8 {
        self.address
    }

    /// Give the bus back.
    pub fn release(self) -> I2C {
        self.i2c
    }
}

fn i2c_frame(control: u8, chunk: &[u8], buf: &mut [u8; I2C_CHUNK + 1]) -> usize {
    buf[0] = control;
    buf[1..=chunk.len()].copy_from_slice(chunk);
    chunk.len() + 1
}

impl<I2C: embedded_hal::i2c::I2c> I2cInterface<I2C> {
    fn send(&mut self, control: u8, bytes: &[u8]) -> Result<(), BusError<I2C::Error>> {
        let mut buf = [0u8; I2C_CHUNK + 1];
        for chunk in bytes.chunks(I2C_CHUNK) {
            let n = i2c_frame(control, chunk, &mut buf);
            self.i2c
                .write(self.address, &buf[..n])
                .map_err(BusError::Bus)?;
        }
        Ok(())
    }
}

impl<I2C: embedded_hal::i2c::I2c> Interface for I2cInterface<I2C> {
    type Error = I2C::Error;

    fn command(&mut self, bytes: &[u8]) -> Result<(), BusError<I2C::Error>> {
        self.send(I2C_CONTROL_COMMAND, bytes)
    }

    fn data(&mut self, bytes: &[u8]) -> Result<(), BusError<I2C::Error>> {
        self.send(I2C_CONTROL_DATA, bytes)
    }
}

impl<I2C: embedded_hal_async::i2c::I2c> AsyncInterface for I2cInterface<I2C> {
    type Error = I2C::Error;

    async fn command(&mut self, bytes: &[u8]) -> Result<(), BusError<I2C::Error>> {
        let mut buf = [0u8; I2C_CHUNK + 1];
        for chunk in bytes.chunks(I2C_CHUNK) {
            let n = i2c_frame(I2C_CONTROL_COMMAND, chunk, &mut buf);
            self.i2c
                .write(self.address, &buf[..n])
                .await
                .map_err(BusError::Bus)?;
        }
        Ok(())
    }

    async fn data(&mut self, bytes: &[u8]) -> Result<(), BusError<I2C::Error>> {
        let mut buf = [0u8; I2C_CHUNK + 1];
        for chunk in bytes.chunks(I2C_CHUNK) {
            let n = i2c_frame(I2C_CONTROL_DATA, chunk, &mut buf);
            self.i2c
                .write(self.address, &buf[..n])
                .await
                .map_err(BusError::Bus)?;
        }
        Ok(())
    }
}

/// Placeholder for an optional pin that is not connected (pass
/// `None::<NoPin>`). Reads as low; writes do nothing.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct NoPin;

impl digital::ErrorType for NoPin {
    type Error = core::convert::Infallible;
}

impl OutputPin for NoPin {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

impl digital::InputPin for NoPin {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(false)
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(true)
    }
}

impl embedded_hal_async::digital::Wait for NoPin {
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}
