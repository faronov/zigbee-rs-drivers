//! Bit-level simulation of devices on a 1-Wire bus (tests only).

extern crate std;

use crate::{OneWire, Rom, command};
use std::vec::Vec;
use zigbee_sensor::Error;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    Idle,
    Command { byte: u8, bits: u8 },
    ReadRom { bit: u8 },
    MatchRom { bit: u8 },
    Search { bit: u8, phase: u8 },
    Function,
}

pub(crate) struct SimBus {
    roms: Vec<Rom>,
    alarm: Vec<bool>,
    active: Vec<bool>,
    state: State,
    pub(crate) resets: usize,
}

fn rom_bit(rom: &Rom, bit: u8) -> bool {
    rom.0[usize::from(bit / 8)] >> (bit % 8) & 1 != 0
}

impl SimBus {
    pub(crate) fn new(roms: &[Rom]) -> Self {
        Self {
            roms: roms.to_vec(),
            alarm: std::vec![false; roms.len()],
            active: std::vec![false; roms.len()],
            state: State::Idle,
            resets: 0,
        }
    }

    pub(crate) fn with_alarm(mut self, index: usize) -> Self {
        self.alarm[index] = true;
        self
    }

    pub(crate) fn selected(&self) -> Vec<bool> {
        self.active.clone()
    }

    fn wired_and(&self, f: impl Fn(&Rom) -> bool) -> bool {
        self.roms
            .iter()
            .zip(&self.active)
            .filter(|(_, a)| **a)
            .all(|(r, _)| f(r))
    }

    fn keep(&mut self, f: impl Fn(&Rom) -> bool) {
        for (rom, active) in self.roms.iter().zip(self.active.iter_mut()) {
            if !f(rom) {
                *active = false;
            }
        }
    }
}

impl OneWire for SimBus {
    type BusError = ();

    fn reset(&mut self) -> Result<bool, Error<()>> {
        self.resets += 1;
        self.active.iter_mut().for_each(|a| *a = true);
        self.state = State::Command { byte: 0, bits: 0 };
        Ok(!self.roms.is_empty())
    }

    fn write_bit(&mut self, bit: bool) -> Result<(), Error<()>> {
        self.state = match self.state {
            State::Command { byte, bits } => {
                let byte = byte | (u8::from(bit) << bits);
                if bits < 7 {
                    State::Command {
                        byte,
                        bits: bits + 1,
                    }
                } else {
                    match byte {
                        command::READ_ROM => State::ReadRom { bit: 0 },
                        command::MATCH_ROM => State::MatchRom { bit: 0 },
                        command::SKIP_ROM => State::Function,
                        command::SEARCH_ROM => State::Search { bit: 0, phase: 0 },
                        command::ALARM_SEARCH => {
                            for (a, alarm) in self.active.iter_mut().zip(&self.alarm) {
                                *a &= *alarm;
                            }
                            State::Search { bit: 0, phase: 0 }
                        }
                        _ => State::Idle,
                    }
                }
            }
            State::MatchRom { bit: b } => {
                self.keep(|r| rom_bit(r, b) == bit);
                if b == 63 {
                    State::Function
                } else {
                    State::MatchRom { bit: b + 1 }
                }
            }
            State::Search { bit: b, phase: 2 } => {
                self.keep(|r| rom_bit(r, b) == bit);
                if b == 63 {
                    State::Function
                } else {
                    State::Search {
                        bit: b + 1,
                        phase: 0,
                    }
                }
            }
            other => other,
        };
        Ok(())
    }

    fn read_bit(&mut self) -> Result<bool, Error<()>> {
        let (value, next) = match self.state {
            State::ReadRom { bit } => (
                self.wired_and(|r| rom_bit(r, bit)),
                if bit == 63 {
                    State::Function
                } else {
                    State::ReadRom { bit: bit + 1 }
                },
            ),
            State::Search { bit, phase: 0 } => (
                self.wired_and(|r| rom_bit(r, bit)),
                State::Search { bit, phase: 1 },
            ),
            State::Search { bit, phase: 1 } => (
                self.wired_and(|r| !rom_bit(r, bit)),
                State::Search { bit, phase: 2 },
            ),
            other => (true, other),
        };
        self.state = next;
        Ok(value)
    }
}

impl crate::asynch::OneWire for SimBus {
    type BusError = ();

    async fn reset(&mut self) -> Result<bool, Error<()>> {
        OneWire::reset(self)
    }

    async fn write_bit(&mut self, bit: bool) -> Result<(), Error<()>> {
        OneWire::write_bit(self, bit)
    }

    async fn read_bit(&mut self) -> Result<bool, Error<()>> {
        OneWire::read_bit(self)
    }
}

pub(crate) fn block_on<F: core::future::Future>(fut: F) -> F::Output {
    let mut cx = core::task::Context::from_waker(core::task::Waker::noop());
    let mut fut = core::pin::pin!(fut);
    loop {
        if let core::task::Poll::Ready(v) = fut.as_mut().poll(&mut cx) {
            return v;
        }
    }
}
