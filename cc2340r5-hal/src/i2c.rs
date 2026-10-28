//! Blocking, polled I2C controller driver for `I2C0`.
//!
//! The command/status protocol mirrors TI's `I2CLPF3.c` interrupt state
//! machine (polled instead of interrupt driven):
//!
//! * a write segment starts with `CTA = addr << 1`, the first byte in `CDR`
//!   and `START|RUN`; further bytes use `RUN`;
//! * a read segment (or the repeated start of a write-then-read) uses
//!   `CTA = addr << 1 | 1` and `START|RUN`, with `ACK` on every byte except
//!   the last of the segment;
//! * the last byte of the transaction adds `STOP`;
//! * on `ERR`/`ARBLST` a bare `STOP` is issued and the error decoded with
//!   `ARBLST` first, then `ADRACKN`, then data NACK. A `DATACKN` seen on a
//!   receive step is our own NACK of the last byte and not an error (the
//!   peripheral quirk TI works around with `writeCount == 0`).
//!
//! Power: the I2C clock is gated (`CLKCTL`) and the pins are returned to
//! GPIO input with pull-up (TI's `GPIO_resetConfig()` state for the
//! SysConfig I2C pins) whenever the bus is not in use. Each transaction
//! powers the controller on demand unless [`I2c::power_up`] was called to
//! batch several transactions.

use embedded_hal::i2c::{self as hal, ErrorKind, NoAcknowledgeSource, Operation, SevenBitAddress};

use crate::gpio::{Dio, PinFunction};
#[cfg(target_os = "none")]
use crate::gpio::{IOC_INPUT_ENABLE, IOC_OPEN_DRAIN, IOC_PULL_UP};

/// `I2C0_BASE` (`hw_memmap.h`).
pub const I2C0_BASE: u32 = 0x4003_8000;
const CTA: u32 = 0x800;
const CSTA_CCTL: u32 = 0x804;
const CDR: u32 = 0x808;
const CTPR: u32 = 0x80C;
const CIMR: u32 = 0x810;
const CICR: u32 = 0x81C;
const CCR: u32 = 0x820;

/// `I2C_CCR_CFE` controller function enable.
const CCR_CFE: u32 = 0x10;

/// `I2C_CSTA_*` status bits.
pub const CSTA_BUSBSY: u32 = 0x40;
pub const CSTA_ARBLST: u32 = 0x10;
pub const CSTA_DATACKN: u32 = 0x08;
pub const CSTA_ADRACKN: u32 = 0x04;
pub const CSTA_ERR: u32 = 0x02;
pub const CSTA_BUSY: u32 = 0x01;

/// `I2C_CCTL_*` command bits.
pub const CCTL_ACK: u32 = 0x08;
pub const CCTL_STOP: u32 = 0x04;
pub const CCTL_START: u32 = 0x02;
pub const CCTL_RUN: u32 = 0x01;

/// Input clock of the I2C peripheral (CLKSVT).
pub const I2C_CLOCK_HZ: u32 = 48_000_000;

/// Bus clock.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Speed {
    Standard100k,
    Fast400k,
}

impl Speed {
    pub const fn hz(self) -> u32 {
        match self {
            Self::Standard100k => 100_000,
            Self::Fast400k => 400_000,
        }
    }

    /// `CTPR` from `I2CControllerInitExpClk()`: the fastest SCL that does
    /// not exceed the requested rate.
    pub const fn timer_period(self) -> u32 {
        let scl = self.hz();
        I2C_CLOCK_HZ.div_ceil(2 * 10 * scl) - 1
    }
}

/// Driver errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// Another controller holds the bus (`CSTA.BUSBSY`) at transaction start.
    BusBusy,
    ArbitrationLost,
    AddressNack,
    DataNack,
    /// The controller stayed busy beyond the bounded poll.
    Timeout,
    /// Zero-length operations cannot be expressed with this controller.
    EmptyOperation,
}

impl hal::Error for Error {
    fn kind(&self) -> ErrorKind {
        match self {
            Self::BusBusy => ErrorKind::Bus,
            Self::ArbitrationLost => ErrorKind::ArbitrationLoss,
            Self::AddressNack => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address),
            Self::DataNack => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data),
            Self::Timeout | Self::EmptyOperation => ErrorKind::Other,
        }
    }
}

/// Register-level controller access; the hardware implementation is
/// [`I2c`], tests use a scripted fake.
pub(crate) trait Controller {
    fn status(&mut self) -> u32;
    fn set_target(&mut self, address: u8, read: bool);
    fn write_data(&mut self, byte: u8);
    fn read_data(&mut self) -> u8;
    fn command(&mut self, command: u32);
    /// Wait until `CSTA.BUSY` clears and return the status.
    fn wait_idle(&mut self) -> Result<u32, Error>;
}

fn check_step(status: u32, read: bool) -> Result<(), Error> {
    if status & (CSTA_ERR | CSTA_ARBLST) == 0 {
        return Ok(());
    }
    if status & CSTA_ARBLST != 0 {
        Err(Error::ArbitrationLost)
    } else if status & CSTA_ADRACKN != 0 {
        Err(Error::AddressNack)
    } else if read {
        // Our own NACK of a final received byte.
        Ok(())
    } else {
        Err(Error::DataNack)
    }
}

/// Run one I2C transaction on `controller` (TI `I2CLPF3` sequence).
pub(crate) fn run_transaction<C: Controller>(
    controller: &mut C,
    address: SevenBitAddress,
    operations: &mut [Operation<'_>],
) -> Result<(), Error> {
    if operations.is_empty() {
        return Ok(());
    }
    if operations.iter().any(|op| match op {
        Operation::Read(buffer) => buffer.is_empty(),
        Operation::Write(bytes) => bytes.is_empty(),
    }) {
        return Err(Error::EmptyOperation);
    }
    if controller.status() & CSTA_BUSBSY != 0 {
        return Err(Error::BusBusy);
    }

    let count = operations.len();
    let mut previous_read: Option<bool> = None;
    for index in 0..count {
        let read = matches!(operations[index], Operation::Read(_));
        // Adjacent operations of one direction form one segment.
        let segment_start = previous_read != Some(read);
        let segment_continues =
            index + 1 < count && matches!(operations[index + 1], Operation::Read(_)) == read;
        let transaction_ends = index + 1 == count;
        previous_read = Some(read);

        let length = match &operations[index] {
            Operation::Read(buffer) => buffer.len(),
            Operation::Write(bytes) => bytes.len(),
        };
        for position in 0..length {
            let first = segment_start && position == 0;
            let last_of_segment = position + 1 == length && !segment_continues;
            let mut command = CCTL_RUN;
            if first {
                command |= CCTL_START;
                controller.set_target(address, read);
            }
            if last_of_segment && transaction_ends {
                command |= CCTL_STOP;
            }
            if read && !last_of_segment {
                command |= CCTL_ACK;
            }
            if let Operation::Write(bytes) = &operations[index] {
                controller.write_data(bytes[position]);
            }
            controller.command(command);
            let status = match controller.wait_idle() {
                Ok(status) => status,
                Err(error) => {
                    controller.command(CCTL_STOP);
                    return Err(error);
                }
            };
            if let Err(error) = check_step(status, read) {
                // No STOP interrupt exists: send STOP and finish (TI).
                controller.command(CCTL_STOP);
                let _ = controller.wait_idle();
                return Err(error);
            }
            if let Operation::Read(buffer) = &mut operations[index] {
                buffer[position] = controller.read_data();
            }
        }
    }
    Ok(())
}

/// Ownership token for the `I2C0` peripheral (from [`crate::Core`]).
pub struct I2c0 {
    _private: (),
}

impl I2c0 {
    pub(crate) const fn new() -> Self {
        Self { _private: () }
    }
}

/// Pins and their pin-mux functions for `I2C0` (board wiring, from the
/// SysConfig pin table).
pub struct I2cPins {
    pub sda: Dio,
    pub sda_function: PinFunction,
    pub scl: Dio,
    pub scl_function: PinFunction,
}

/// Blocking `I2C0` controller.
pub struct I2c {
    _token: I2c0,
    pins: I2cPins,
    speed: Speed,
    powered: bool,
}

/// Bounded `CSTA.BUSY` poll. A 400 kHz/100 kHz byte takes 23/90 µs; this
/// allows a few ms of clock stretching at 48 MHz.
#[cfg(target_os = "none")]
const BUSY_POLL_LIMIT: u32 = 100_000;

impl I2c {
    /// Take ownership of the controller and its pins. The bus stays
    /// unpowered (clock gated, pins idle) until the first transaction.
    pub fn new(token: I2c0, pins: I2cPins, speed: Speed) -> Self {
        let i2c = Self {
            _token: token,
            pins,
            speed,
            powered: false,
        };
        #[cfg(target_os = "none")]
        i2c.idle_pins();
        i2c
    }

    /// Return the token and pins.
    pub fn release(mut self) -> (I2c0, I2cPins) {
        self.power_down();
        (self._token, self.pins)
    }

    pub const fn speed(&self) -> Speed {
        self.speed
    }

    pub const fn is_powered(&self) -> bool {
        self.powered
    }

    /// Clock the controller and mux the pins until [`I2c::power_down`].
    pub fn power_up(&mut self) {
        if self.powered {
            return;
        }
        #[cfg(target_os = "none")]
        {
            crate::clkctl::enable(crate::clkctl::PeripheralClock::I2c0);
            // I2CLPF3_initHw(): mask + clear interrupts, set the rate, enable.
            wr(CIMR, 0);
            wr(CICR, 0xFFFF_FFFF);
            wr(CCR, rd(CCR) | CCR_CFE);
            wr(CSTA_CCTL, CCTL_RUN);
            wr(CTPR, self.speed.timer_period());
            // I2CLPF3_initIO(): open drain, input buffer, pull-up, function.
            for (pin, function) in [
                (&self.pins.scl, self.pins.scl_function),
                (&self.pins.sda, self.pins.sda_function),
            ] {
                pin.set_output(true);
                pin.set_output_enable(true);
                pin.set_ioc(IOC_OPEN_DRAIN | IOC_INPUT_ENABLE | IOC_PULL_UP | function.portcfg());
            }
        }
        self.powered = true;
    }

    /// Return the pins to idle and gate the controller clock.
    pub fn power_down(&mut self) {
        if !self.powered {
            return;
        }
        #[cfg(target_os = "none")]
        {
            self.idle_pins();
            // I2CControllerDisable().
            wr(CSTA_CCTL, 0);
            wr(CCR, rd(CCR) & !CCR_CFE);
            crate::clkctl::disable(crate::clkctl::PeripheralClock::I2c0);
        }
        self.powered = false;
    }

    #[cfg(target_os = "none")]
    fn idle_pins(&self) {
        for pin in [&self.pins.scl, &self.pins.sda] {
            pin.set_output_enable(false);
            pin.set_ioc(IOC_INPUT_ENABLE | IOC_PULL_UP);
        }
    }
}

#[cfg(target_os = "none")]
fn rd(offset: u32) -> u32 {
    // SAFETY: I2C0 registers; the clock is enabled by `power_up`.
    unsafe { ((I2C0_BASE + offset) as *const u32).read_volatile() }
}

#[cfg(target_os = "none")]
fn wr(offset: u32, value: u32) {
    // SAFETY: I2C0 registers; the clock is enabled by `power_up`.
    unsafe { ((I2C0_BASE + offset) as *mut u32).write_volatile(value) }
}

#[cfg(target_os = "none")]
impl Controller for I2c {
    fn status(&mut self) -> u32 {
        rd(CSTA_CCTL)
    }

    fn set_target(&mut self, address: u8, read: bool) {
        wr(CTA, (u32::from(address) << 1) | u32::from(read));
    }

    fn write_data(&mut self, byte: u8) {
        wr(CDR, u32::from(byte));
    }

    fn read_data(&mut self) -> u8 {
        rd(CDR) as u8
    }

    fn command(&mut self, command: u32) {
        wr(CSTA_CCTL, command);
        // BUSY asserts a few cycles after the command write.
        cortex_m::asm::delay(16);
    }

    fn wait_idle(&mut self) -> Result<u32, Error> {
        for _ in 0..BUSY_POLL_LIMIT {
            let status = rd(CSTA_CCTL);
            if status & CSTA_BUSY == 0 {
                return Ok(status);
            }
        }
        Err(Error::Timeout)
    }
}

impl hal::ErrorType for I2c {
    type Error = Error;
}

#[cfg(target_os = "none")]
impl hal::I2c for I2c {
    fn transaction(
        &mut self,
        address: SevenBitAddress,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        let was_powered = self.powered;
        self.power_up();
        let result = run_transaction(self, address, operations);
        if !was_powered {
            self.power_down();
        }
        result
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use std::vec::Vec;

    #[derive(Debug, PartialEq, Eq, Clone, Copy)]
    enum Event {
        Target(u8, bool),
        Data(u8),
        Command(u32),
        Read,
    }

    struct Fake {
        events: Vec<Event>,
        status: u32,
        /// Status returned by the n-th `wait_idle`.
        fail_at: Option<(usize, u32)>,
        waits: usize,
        rx: Vec<u8>,
    }

    impl Fake {
        fn new() -> Self {
            Self {
                events: Vec::new(),
                status: 0x20,
                fail_at: None,
                waits: 0,
                rx: Vec::new(),
            }
        }
    }

    impl Controller for Fake {
        fn status(&mut self) -> u32 {
            self.status
        }
        fn set_target(&mut self, address: u8, read: bool) {
            self.events.push(Event::Target(address, read));
        }
        fn write_data(&mut self, byte: u8) {
            self.events.push(Event::Data(byte));
        }
        fn read_data(&mut self) -> u8 {
            self.events.push(Event::Read);
            self.rx.remove(0)
        }
        fn command(&mut self, command: u32) {
            self.events.push(Event::Command(command));
        }
        fn wait_idle(&mut self) -> Result<u32, Error> {
            let n = self.waits;
            self.waits += 1;
            match self.fail_at {
                Some((at, status)) if at == n => Ok(status),
                _ => Ok(0x20),
            }
        }
    }

    use Event::*;

    #[test]
    fn timer_period_matches_driverlib() {
        assert_eq!(Speed::Standard100k.timer_period(), 23);
        assert_eq!(Speed::Fast400k.timer_period(), 5);
    }

    #[test]
    fn single_byte_write_is_single_send() {
        let mut fake = Fake::new();
        run_transaction(&mut fake, 0x48, &mut [Operation::Write(&[0x01])]).unwrap();
        assert_eq!(fake.events, [Target(0x48, false), Data(0x01), Command(0x7)]);
    }

    #[test]
    fn write_read_matches_the_ti_burst_sequence() {
        let mut fake = Fake::new();
        fake.rx = std::vec![0x12, 0x34];
        let mut buffer = [0u8; 2];
        run_transaction(
            &mut fake,
            0x44,
            &mut [Operation::Write(&[0x7E]), Operation::Read(&mut buffer)],
        )
        .unwrap();
        assert_eq!(buffer, [0x12, 0x34]);
        assert_eq!(
            fake.events,
            [
                Target(0x44, false),
                Data(0x7E),
                Command(0x3), // BURST_SEND_START
                Target(0x44, true),
                Command(0xB), // BURST_RECEIVE_START (ACK)
                Read,
                Command(0x5), // BURST_RECEIVE_FINISH (NACK + STOP)
                Read,
            ]
        );
    }

    #[test]
    fn multi_byte_write_then_one_byte_read() {
        let mut fake = Fake::new();
        fake.rx = std::vec![0xAA];
        let mut buffer = [0u8; 1];
        run_transaction(
            &mut fake,
            0x23,
            &mut [Operation::Write(&[1, 2]), Operation::Read(&mut buffer)],
        )
        .unwrap();
        assert_eq!(
            fake.events,
            [
                Target(0x23, false),
                Data(1),
                Command(0x3),
                Data(2),
                Command(0x1),
                Target(0x23, true),
                Command(0x7),
                Read,
            ]
        );
    }

    #[test]
    fn adjacent_writes_share_one_segment() {
        let mut fake = Fake::new();
        run_transaction(
            &mut fake,
            0x48,
            &mut [Operation::Write(&[1]), Operation::Write(&[2, 3])],
        )
        .unwrap();
        assert_eq!(
            fake.events,
            [
                Target(0x48, false),
                Data(1),
                Command(0x3),
                Data(2),
                Command(0x1),
                Data(3),
                Command(0x5),
            ]
        );
    }

    #[test]
    fn address_nack_sends_stop_and_reports_address() {
        let mut fake = Fake::new();
        fake.fail_at = Some((0, CSTA_ERR | CSTA_ADRACKN | CSTA_DATACKN));
        let error = run_transaction(&mut fake, 0x48, &mut [Operation::Write(&[1, 2])]).unwrap_err();
        assert_eq!(error, Error::AddressNack);
        assert_eq!(fake.events.last(), Some(&Command(CCTL_STOP)));
        assert_eq!(
            hal::Error::kind(&error),
            ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address)
        );
    }

    #[test]
    fn data_nack_on_write_is_an_error_but_not_on_read() {
        let mut fake = Fake::new();
        fake.fail_at = Some((1, CSTA_ERR | CSTA_DATACKN));
        let error = run_transaction(&mut fake, 0x48, &mut [Operation::Write(&[1, 2])]).unwrap_err();
        assert_eq!(error, Error::DataNack);

        let mut fake = Fake::new();
        fake.fail_at = Some((0, CSTA_ERR | CSTA_DATACKN));
        fake.rx = std::vec![9];
        let mut buffer = [0u8; 1];
        run_transaction(&mut fake, 0x48, &mut [Operation::Read(&mut buffer)]).unwrap();
        assert_eq!(buffer, [9]);
    }

    #[test]
    fn arbitration_loss_takes_precedence() {
        let mut fake = Fake::new();
        fake.fail_at = Some((0, CSTA_ARBLST | CSTA_ADRACKN));
        assert_eq!(
            run_transaction(&mut fake, 0x48, &mut [Operation::Write(&[1])]),
            Err(Error::ArbitrationLost)
        );
    }

    #[test]
    fn busy_bus_and_empty_operations_are_rejected_without_traffic() {
        let mut fake = Fake::new();
        fake.status = CSTA_BUSBSY;
        assert_eq!(
            run_transaction(&mut fake, 0x48, &mut [Operation::Write(&[1])]),
            Err(Error::BusBusy)
        );
        let mut fake = Fake::new();
        assert_eq!(
            run_transaction(&mut fake, 0x48, &mut [Operation::Write(&[])]),
            Err(Error::EmptyOperation)
        );
        assert!(fake.events.is_empty());
    }
}
