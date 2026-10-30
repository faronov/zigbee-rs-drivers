//! Register access over I2C or SPI.
//!
//! The driver only needs "read N consecutive registers" and "write one
//! register". [`I2cInterface`] and [`SpiInterface`] implement both the
//! blocking [`Interface`] and the [`AsyncInterface`] traits; implement them
//! yourself for anything else (a sensor hub, a shared-bus wrapper ...).
//! Implementations must auto-increment the address on multi-byte reads.

/// Blocking register access.
pub trait Interface {
    /// Transport error.
    type Error;

    /// Read consecutive registers starting at `reg` into `buf`.
    fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error>;

    /// Write one register.
    fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error>;
}

/// Async register access; twin of [`Interface`].
#[allow(async_fn_in_trait)]
pub trait AsyncInterface {
    /// Transport error.
    type Error;

    /// Read consecutive registers starting at `reg` into `buf`.
    async fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error>;

    /// Write one register.
    async fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error>;
}

/// I2C sub-address bit 7: auto-increment on multi-byte reads.
const I2C_AUTO_INCREMENT: u8 = 0x80;

/// I2C transport: sub-address byte (bit 7 = auto-increment on reads), then
/// data.
#[derive(Debug)]
pub struct I2cInterface<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C> I2cInterface<I2C> {
    /// Wrap an I2C bus and 7-bit device address.
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
        self.i2c
            .write_read(self.address, &[reg | I2C_AUTO_INCREMENT], buf)
    }

    fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error> {
        self.i2c.write(self.address, &[reg, value])
    }
}

impl<I2C: embedded_hal_async::i2c::I2c> AsyncInterface for I2cInterface<I2C> {
    type Error = I2C::Error;

    async fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c
            .write_read(self.address, &[reg | I2C_AUTO_INCREMENT], buf)
            .await
    }

    async fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error> {
        self.i2c.write(self.address, &[reg, value]).await
    }
}

/// 4-wire SPI transport (mode 3): address byte with bit 7 = read, then
/// data (reads auto-increment). The part starts in 3-wire mode; the driver
/// enables 4-wire mode (`CFG_REG_C.4WSPI`, a write-only transfer) before
/// its first read. Chip select is handled by the `SpiDevice`.
#[derive(Debug)]
pub struct SpiInterface<SPI> {
    spi: SPI,
}

impl<SPI> SpiInterface<SPI> {
    /// Wrap an SPI device.
    pub const fn new(spi: SPI) -> Self {
        Self { spi }
    }

    /// Give the device back.
    pub fn release(self) -> SPI {
        self.spi
    }
}

const SPI_READ: u8 = 0x80;
const SPI_ADDR: u8 = 0x7F;

impl<SPI: embedded_hal::spi::SpiDevice> Interface for SpiInterface<SPI> {
    type Error = SPI::Error;

    fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        use embedded_hal::spi::Operation;
        let cmd = reg & SPI_ADDR | SPI_READ;
        self.spi
            .transaction(&mut [Operation::Write(&[cmd]), Operation::Read(buf)])
    }

    fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error> {
        self.spi.write(&[reg & SPI_ADDR, value])
    }
}

impl<SPI: embedded_hal_async::spi::SpiDevice> AsyncInterface for SpiInterface<SPI> {
    type Error = SPI::Error;

    async fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        use embedded_hal_async::spi::Operation;
        let cmd = reg & SPI_ADDR | SPI_READ;
        self.spi
            .transaction(&mut [Operation::Write(&[cmd]), Operation::Read(buf)])
            .await
    }

    async fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error> {
        self.spi.write(&[reg & SPI_ADDR, value]).await
    }
}
