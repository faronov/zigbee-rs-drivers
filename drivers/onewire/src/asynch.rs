//! Async 1-Wire bus master trait.
//!
//! Same operations and defaults as [`crate::OneWire`]. What "async" buys on
//! 1-Wire is limited: a time slot (≤ 120 µs) is too short to hand to an
//! executor, so an implementation runs each slot synchronously and awaits
//! *between* bits/bytes (where the bus may idle indefinitely), letting other
//! tasks — the Zigbee stack — run during a transaction. A hardware or
//! UART-based master can make every operation truly asynchronous.
//! [`crate::BitBang`] yields to the executor after every byte and reset.

use crate::{Rom, Triplet, command};
use zigbee_sensor::Error;

/// Async twin of [`crate::OneWire`].
#[allow(async_fn_in_trait)]
pub trait OneWire {
    /// Transport error type.
    type BusError;

    /// See [`crate::OneWire::reset`].
    async fn reset(&mut self) -> Result<bool, Error<Self::BusError>>;

    /// See [`crate::OneWire::write_bit`].
    async fn write_bit(&mut self, bit: bool) -> Result<(), Error<Self::BusError>>;

    /// See [`crate::OneWire::read_bit`].
    async fn read_bit(&mut self) -> Result<bool, Error<Self::BusError>>;

    /// See [`crate::OneWire::write_byte`].
    async fn write_byte(&mut self, byte: u8) -> Result<(), Error<Self::BusError>> {
        for i in 0..8 {
            self.write_bit(byte >> i & 1 != 0).await?;
        }
        Ok(())
    }

    /// See [`crate::OneWire::read_byte`].
    async fn read_byte(&mut self) -> Result<u8, Error<Self::BusError>> {
        let mut byte = 0;
        for i in 0..8 {
            if self.read_bit().await? {
                byte |= 1 << i;
            }
        }
        Ok(byte)
    }

    /// See [`crate::OneWire::write_bytes`].
    async fn write_bytes(&mut self, bytes: &[u8]) -> Result<(), Error<Self::BusError>> {
        for &b in bytes {
            self.write_byte(b).await?;
        }
        Ok(())
    }

    /// See [`crate::OneWire::read_bytes`].
    async fn read_bytes(&mut self, buf: &mut [u8]) -> Result<(), Error<Self::BusError>> {
        for b in buf {
            *b = self.read_byte().await?;
        }
        Ok(())
    }

    /// See [`crate::OneWire::triplet`].
    async fn triplet(&mut self, preferred: bool) -> Result<Triplet, Error<Self::BusError>> {
        let id_bit = self.read_bit().await?;
        let complement_bit = self.read_bit().await?;
        let direction = Triplet::direction_for(id_bit, complement_bit, preferred);
        if !(id_bit && complement_bit) {
            self.write_bit(direction).await?;
        }
        Ok(Triplet {
            id_bit,
            complement_bit,
            direction,
        })
    }

    /// See [`crate::OneWire::strong_pullup`].
    async fn strong_pullup(&mut self, enable: bool) -> Result<(), Error<Self::BusError>> {
        if enable {
            Err(Error::Unsupported)
        } else {
            Ok(())
        }
    }

    /// See [`crate::OneWire::reset_presence`].
    async fn reset_presence(&mut self) -> Result<(), Error<Self::BusError>> {
        if self.reset().await? {
            Ok(())
        } else {
            Err(Error::Timeout)
        }
    }

    /// See [`crate::OneWire::select`].
    async fn select(&mut self, rom: Option<&Rom>) -> Result<(), Error<Self::BusError>> {
        self.reset_presence().await?;
        match rom {
            Some(rom) => {
                self.write_byte(command::MATCH_ROM).await?;
                self.write_bytes(&rom.0).await
            }
            None => self.write_byte(command::SKIP_ROM).await,
        }
    }

    /// See [`crate::OneWire::read_rom`].
    async fn read_rom(&mut self) -> Result<Rom, Error<Self::BusError>> {
        self.reset_presence().await?;
        self.write_byte(command::READ_ROM).await?;
        let mut rom = [0u8; 8];
        self.read_bytes(&mut rom).await?;
        Rom(rom).check()
    }
}

impl<T: OneWire + ?Sized> OneWire for &mut T {
    type BusError = T::BusError;

    async fn reset(&mut self) -> Result<bool, Error<Self::BusError>> {
        T::reset(self).await
    }
    async fn write_bit(&mut self, bit: bool) -> Result<(), Error<Self::BusError>> {
        T::write_bit(self, bit).await
    }
    async fn read_bit(&mut self) -> Result<bool, Error<Self::BusError>> {
        T::read_bit(self).await
    }
    async fn write_byte(&mut self, byte: u8) -> Result<(), Error<Self::BusError>> {
        T::write_byte(self, byte).await
    }
    async fn read_byte(&mut self) -> Result<u8, Error<Self::BusError>> {
        T::read_byte(self).await
    }
    async fn write_bytes(&mut self, bytes: &[u8]) -> Result<(), Error<Self::BusError>> {
        T::write_bytes(self, bytes).await
    }
    async fn read_bytes(&mut self, buf: &mut [u8]) -> Result<(), Error<Self::BusError>> {
        T::read_bytes(self, buf).await
    }
    async fn triplet(&mut self, preferred: bool) -> Result<Triplet, Error<Self::BusError>> {
        T::triplet(self, preferred).await
    }
    async fn strong_pullup(&mut self, enable: bool) -> Result<(), Error<Self::BusError>> {
        T::strong_pullup(self, enable).await
    }
}
