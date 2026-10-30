//! Register access over I2C or SPI.
//!
//! I2C multi-byte reads auto-increment. SPI (4-wire, mode 3, ≤ 5 MHz):
//! bit 7 of the first byte selects read, bit 6 (MB) enables multi-byte
//! transfers, bits 5:0 are the register address.

use embedded_hal::spi::Operation;

/// Blocking register access.
pub trait Interface {
    /// Transport error.
    type Error;

    /// Read `buf.len()` consecutive registers starting at `reg`.
    fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error>;

    /// Write one register.
    fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error>;
}

/// Async register access.
#[allow(async_fn_in_trait)]
pub trait AsyncInterface {
    /// Transport error.
    type Error;

    /// Read `buf.len()` consecutive registers starting at `reg`.
    async fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error>;

    /// Write one register.
    async fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error>;
}

const fn spi_read(reg: u8, len: usize) -> u8 {
    let mb = if len > 1 { 0x40 } else { 0 };
    0x80 | mb | (reg & 0x3F)
}

/// I2C transport.
pub struct I2cInterface<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C> I2cInterface<I2C> {
    /// Wrap an I2C bus; no bus traffic.
    pub const fn new(i2c: I2C, address: u8) -> Self {
        Self { i2c, address }
    }

    /// Give the bus back.
    pub fn release(self) -> I2C {
        self.i2c
    }
}

impl<I2C: embedded_hal::i2c::I2c> Interface for I2cInterface<I2C> {
    type Error = I2C::Error;

    fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.write_read(self.address, &[reg], buf)
    }

    fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error> {
        self.i2c.write(self.address, &[reg, value])
    }
}

impl<I2C: embedded_hal_async::i2c::I2c> AsyncInterface for I2cInterface<I2C> {
    type Error = I2C::Error;

    async fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.write_read(self.address, &[reg], buf).await
    }

    async fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error> {
        self.i2c.write(self.address, &[reg, value]).await
    }
}

/// SPI transport (4-wire, `SpiDevice` handles chip select).
pub struct SpiInterface<SPI> {
    spi: SPI,
}

impl<SPI> SpiInterface<SPI> {
    /// Wrap an SPI device; no bus traffic.
    pub const fn new(spi: SPI) -> Self {
        Self { spi }
    }

    /// Give the device back.
    pub fn release(self) -> SPI {
        self.spi
    }
}

impl<SPI: embedded_hal::spi::SpiDevice> Interface for SpiInterface<SPI> {
    type Error = SPI::Error;

    fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.spi.transaction(&mut [
            Operation::Write(&[spi_read(reg, buf.len())]),
            Operation::Read(buf),
        ])
    }

    fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error> {
        self.spi.write(&[reg & 0x3F, value])
    }
}

impl<SPI: embedded_hal_async::spi::SpiDevice> AsyncInterface for SpiInterface<SPI> {
    type Error = SPI::Error;

    async fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.spi
            .transaction(&mut [
                Operation::Write(&[spi_read(reg, buf.len())]),
                Operation::Read(buf),
            ])
            .await
    }

    async fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error> {
        self.spi.write(&[reg & 0x3F, value]).await
    }
}
