//! Register access over I2C or SPI.
//!
//! I2C: plain register pointer, auto-incremented on burst reads.
//! SPI (mode 0 or 3, 4-wire): bit 7 of the first byte selects read; the
//! part then clocks out one dummy byte before the data. The part starts in
//! I2C mode and switches to SPI on the first SPI access, which it ignores,
//! so [`Interface::init`] performs one throw-away read on SPI.

use embedded_hal::spi::Operation;

const REG_CHIPID: u8 = 0x00;

/// Blocking register access.
pub trait Interface {
    /// Transport error.
    type Error;

    /// Read `buf.len()` consecutive registers starting at `reg`.
    fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error>;

    /// Write one register.
    fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error>;

    /// Select the bus after power-up or reset (no-op on I2C).
    fn init(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
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

    /// Select the bus after power-up or reset (no-op on I2C).
    async fn init(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
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
        let mut dummy = [0u8];
        self.spi.transaction(&mut [
            Operation::Write(&[reg | 0x80]),
            Operation::Read(&mut dummy),
            Operation::Read(buf),
        ])
    }

    fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error> {
        self.spi.write(&[reg & 0x7F, value])
    }

    fn init(&mut self) -> Result<(), Self::Error> {
        self.read_regs(REG_CHIPID, &mut [0u8])
    }
}

impl<SPI: embedded_hal_async::spi::SpiDevice> AsyncInterface for SpiInterface<SPI> {
    type Error = SPI::Error;

    async fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        let mut dummy = [0u8];
        self.spi
            .transaction(&mut [
                Operation::Write(&[reg | 0x80]),
                Operation::Read(&mut dummy),
                Operation::Read(buf),
            ])
            .await
    }

    async fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error> {
        self.spi.write(&[reg & 0x7F, value]).await
    }

    async fn init(&mut self) -> Result<(), Self::Error> {
        self.read_regs(REG_CHIPID, &mut [0u8]).await
    }
}
