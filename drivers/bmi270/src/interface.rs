//! Register access over I2C or SPI.
//!
//! The driver only needs "read N consecutive registers", "write one
//! register" and, for the configuration file and the feature registers,
//! "write N consecutive registers". [`I2cInterface`] and [`SpiInterface`] implement both the
//! blocking [`Interface`] and the [`AsyncInterface`] traits; implement them
//! yourself for anything else (a sensor hub, a shared-bus wrapper ...).

/// Blocking register access.
pub trait Interface {
    /// Transport error.
    type Error;

    /// Read consecutive registers starting at `reg` into `buf`.
    fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error>;

    /// Write one register.
    fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error>;

    /// Write consecutive registers starting at `reg` in one burst.
    fn write_regs(&mut self, reg: u8, data: &[u8]) -> Result<(), Self::Error>;
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

    /// Write consecutive registers starting at `reg` in one burst.
    async fn write_regs(&mut self, reg: u8, data: &[u8]) -> Result<(), Self::Error>;
}

/// I2C transport: register address byte, then data.
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
        self.i2c.write_read(self.address, &[reg], buf)
    }

    fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error> {
        self.i2c.write(self.address, &[reg, value])
    }

    fn write_regs(&mut self, reg: u8, data: &[u8]) -> Result<(), Self::Error> {
        use embedded_hal::i2c::Operation;
        self.i2c.transaction(
            self.address,
            &mut [Operation::Write(&[reg]), Operation::Write(data)],
        )
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

    async fn write_regs(&mut self, reg: u8, data: &[u8]) -> Result<(), Self::Error> {
        use embedded_hal_async::i2c::Operation;
        self.i2c
            .transaction(
                self.address,
                &mut [Operation::Write(&[reg]), Operation::Write(data)],
            )
            .await
    }
}

/// 4-wire SPI transport (mode 0 or 3): address byte with bit 7 set for
/// reads, one dummy byte, then data. Chip select is handled by the
/// `SpiDevice`.
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

impl<SPI: embedded_hal::spi::SpiDevice> Interface for SpiInterface<SPI> {
    type Error = SPI::Error;

    fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        use embedded_hal::spi::Operation;
        self.spi.transaction(&mut [
            Operation::Write(&[reg | SPI_READ]),
            Operation::Read(&mut [0]),
            Operation::Read(buf),
        ])
    }

    fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error> {
        self.spi.write(&[reg & !SPI_READ, value])
    }

    fn write_regs(&mut self, reg: u8, data: &[u8]) -> Result<(), Self::Error> {
        use embedded_hal::spi::Operation;
        self.spi
            .transaction(&mut [Operation::Write(&[reg & !SPI_READ]), Operation::Write(data)])
    }
}

impl<SPI: embedded_hal_async::spi::SpiDevice> AsyncInterface for SpiInterface<SPI> {
    type Error = SPI::Error;

    async fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        use embedded_hal_async::spi::Operation;
        self.spi
            .transaction(&mut [
                Operation::Write(&[reg | SPI_READ]),
                Operation::Read(&mut [0]),
                Operation::Read(buf),
            ])
            .await
    }

    async fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error> {
        self.spi.write(&[reg & !SPI_READ, value]).await
    }

    async fn write_regs(&mut self, reg: u8, data: &[u8]) -> Result<(), Self::Error> {
        use embedded_hal_async::spi::Operation;
        self.spi
            .transaction(&mut [Operation::Write(&[reg & !SPI_READ]), Operation::Write(data)])
            .await
    }
}
