//! Register access over I2C or SPI.
//!
//! I2C: plain register pointer, auto-incremented on burst reads.
//! SPI: a 16-bit instruction (R/W, two length bits W1:W0, 13-bit address)
//! precedes the data. The datasheet does not say clearly whether multi-byte
//! transfers count addresses up or down in the default MSB-first mode, so
//! every SPI access moves one byte (W1:W0 = 00). [`Interface::init`]
//! writes SPI_CONFIG to enable the SDO pin (4-wire mode, MSB first).

use embedded_hal::spi::Operation;

const REG_SPI_CONFIG: u8 = 0x00;
// SDO_active (bits 7 and 0, mirrored), MSB first, no soft reset.
const SPI_CONFIG_4WIRE: u8 = 0x81;

/// Blocking register access.
pub trait Interface {
    /// Transport error.
    type Error;

    /// Read `buf.len()` consecutive registers starting at `reg`.
    fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error>;

    /// Write one register.
    fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error>;

    /// Bus set-up after power-up or reset (no-op on I2C).
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

    /// Bus set-up after power-up or reset (no-op on I2C).
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

/// SPI transport (`SpiDevice` handles chip select).
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

// Instruction for a one-byte transfer: R/W in bit 15, W1:W0 = 00,
// address bits 12..8 are zero for this part.
const fn instruction(read: bool, reg: u8) -> [u8; 2] {
    [if read { 0x80 } else { 0x00 }, reg]
}

impl<SPI: embedded_hal::spi::SpiDevice> Interface for SpiInterface<SPI> {
    type Error = SPI::Error;

    fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        for (i, byte) in buf.iter_mut().enumerate() {
            let ins = instruction(true, reg.wrapping_add(i as u8));
            self.spi.transaction(&mut [
                Operation::Write(&ins),
                Operation::Read(core::slice::from_mut(byte)),
            ])?;
        }
        Ok(())
    }

    fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error> {
        let [a, b] = instruction(false, reg);
        self.spi.write(&[a, b, value])
    }

    fn init(&mut self) -> Result<(), Self::Error> {
        self.write_reg(REG_SPI_CONFIG, SPI_CONFIG_4WIRE)
    }
}

impl<SPI: embedded_hal_async::spi::SpiDevice> AsyncInterface for SpiInterface<SPI> {
    type Error = SPI::Error;

    async fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        for (i, byte) in buf.iter_mut().enumerate() {
            let ins = instruction(true, reg.wrapping_add(i as u8));
            self.spi
                .transaction(&mut [
                    Operation::Write(&ins),
                    Operation::Read(core::slice::from_mut(byte)),
                ])
                .await?;
        }
        Ok(())
    }

    async fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error> {
        let [a, b] = instruction(false, reg);
        self.spi.write(&[a, b, value]).await
    }

    async fn init(&mut self) -> Result<(), Self::Error> {
        self.write_reg(REG_SPI_CONFIG, SPI_CONFIG_4WIRE).await
    }
}
