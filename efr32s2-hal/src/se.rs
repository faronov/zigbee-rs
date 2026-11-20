//! Secure Engine mailbox (xG24 HSE): the TRNG.
//!
//! The application core asks the Secure Engine for random bytes over the
//! SEMAILBOX host interface (GSDK 4.5.0 `em_se.c` `SE_executeCommand`,
//! `sl_se_manager_entropy.c` `sl_se_get_random`):
//!
//! 1. wait for `TX_STATUS.TXINT` (mailbox free);
//! 2. `TX_HEADER` = message size in bytes, `4 * (4 + parameters)`;
//! 3. FIFO ← command word, input descriptor (0: none), output descriptor
//!    address, parameters (here the byte count);
//! 4. wait for `RX_STATUS.RXINT`; the result code is
//!    `RX_HEADER & 0x000F_0000` (0 = OK). The SE has written the output
//!    through the descriptor (`{data, next = STOP, length | REALIGN}`) by
//!    DMA into our RAM.
//!
//! `SLI_SE_COMMAND_TRNG_GET_RANDOM` = `0x0700_0000`. The SE's TRNG passes
//! its own health tests; a non-OK result is reported, never replaced by a
//! weaker source. Not yet run on hardware.

use core::sync::atomic::{AtomicBool, Ordering};

use crate::chip::BusClock;

/// `SEMAILBOX_S_HOST_BASE` (efr32mg24a410f1536im48.h).
const SEMAILBOX: u32 = 0x4C00_0000;
const FIFO: u32 = SEMAILBOX;
const TX_STATUS: u32 = SEMAILBOX + 0x40;
const RX_STATUS: u32 = SEMAILBOX + 0x44;
const TX_HEADER: u32 = SEMAILBOX + 0x50;
const RX_HEADER: u32 = SEMAILBOX + 0x54;
const TX_STATUS_TXINT: u32 = 1 << 20;
const RX_STATUS_RXINT: u32 = 1 << 20;
const RESPONSE_MASK: u32 = 0x000F_0000;

const COMMAND_TRNG_GET_RANDOM: u32 = 0x0700_0000;
const DATATRANSFER_STOP: u32 = 0x0000_0001;
const DATATRANSFER_REALIGN: u32 = 0x2000_0000;

/// Bytes per mailbox command (a multiple of four; the stack asks for at
/// most 18 at a time).
pub const CHUNK: usize = 64;
/// Bounded polls of the mailbox status (the SE answers in microseconds;
/// this is about 50 ms at 39 MHz).
const TIMEOUT_POLLS: u32 = 2_000_000;

static BUSY: AtomicBool = AtomicBool::new(false);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SeError {
    /// Another caller holds the mailbox.
    Busy,
    /// The mailbox did not become free or the SE did not answer.
    Timeout,
    /// The SE answered with this non-OK result code (`RX_HEADER` bits
    /// 19:16, shifted down).
    Response(u8),
}

/// `SE_DataTransfer_t`.
#[repr(C)]
struct DataTransfer {
    data: u32,
    next: u32,
    length: u32,
}

/// The message words of one TRNG request, in FIFO order after the header.
const fn trng_message(descriptor: u32, length: u32) -> (u32, [u32; 4]) {
    (
        4 * (4 + 1),
        [COMMAND_TRNG_GET_RANDOM, 0, descriptor, length],
    )
}

/// Fill `output` with Secure Engine TRNG bytes.
pub fn fill_random(output: &mut [u8]) -> Result<(), SeError> {
    if BUSY
        .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
        .is_err()
    {
        return Err(SeError::Busy);
    }
    crate::clock::enable_bus_clock(BusClock::SEMAILBOXHOST);
    let mut result = Ok(());
    for chunk in output.chunks_mut(CHUNK) {
        let mut words = [0u32; CHUNK / 4];
        let length = chunk.len().next_multiple_of(4);
        result = request(&mut words, length as u32);
        if result.is_err() {
            break;
        }
        for (index, byte) in chunk.iter_mut().enumerate() {
            *byte = words[index / 4].to_le_bytes()[index % 4];
        }
    }
    if result.is_err() {
        output.fill(0);
    }
    BUSY.store(false, Ordering::Release);
    result
}

fn request(words: &mut [u32; CHUNK / 4], length: u32) -> Result<(), SeError> {
    let descriptor = DataTransfer {
        data: words.as_mut_ptr() as u32,
        next: DATATRANSFER_STOP,
        length: length | DATATRANSFER_REALIGN,
    };
    let (header, message) = trng_message(&descriptor as *const DataTransfer as u32, length);
    unsafe {
        wait(TX_STATUS, TX_STATUS_TXINT)?;
        write(TX_HEADER, header);
        for word in message {
            write(FIFO, word);
        }
        wait(RX_STATUS, RX_STATUS_RXINT)?;
        let response = (read(RX_HEADER) & RESPONSE_MASK) >> 16;
        // The SE wrote `words` by DMA; keep the compiler from assuming the
        // buffer and descriptor are unchanged since the FIFO writes.
        core::sync::atomic::compiler_fence(Ordering::SeqCst);
        let _ = core::ptr::read_volatile(&descriptor.length);
        if response != 0 {
            return Err(SeError::Response(response as u8));
        }
    }
    Ok(())
}

unsafe fn wait(address: u32, flag: u32) -> Result<(), SeError> {
    for _ in 0..TIMEOUT_POLLS {
        if unsafe { read(address) } & flag != 0 {
            return Ok(());
        }
        core::hint::spin_loop();
    }
    Err(SeError::Timeout)
}

#[inline]
unsafe fn read(address: u32) -> u32 {
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    unsafe {
        core::ptr::read_volatile(address as *const u32)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    {
        // Host: a mailbox that is always free and answers OK at once.
        let _ = address;
        TX_STATUS_TXINT | RX_STATUS_RXINT
    }
}

#[inline]
unsafe fn write(address: u32, value: u32) {
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    unsafe {
        core::ptr::write_volatile(address as *mut u32, value)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    let _ = (address, value);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn message_matches_em_se_layout() {
        // TX_HEADER counts the header word, command, two descriptors and
        // one parameter: 4 * (4 + 1) bytes.
        let (header, message) = trng_message(0x2000_1000, 16);
        assert_eq!(header, 20);
        assert_eq!(message, [0x0700_0000, 0, 0x2000_1000, 16]);
        assert_eq!(core::mem::size_of::<DataTransfer>(), 12);
        assert_eq!((TX_STATUS, RX_HEADER), (0x4C00_0040, 0x4C00_0054));
    }

    #[test]
    fn host_fill_releases_the_mailbox() {
        let mut out = [0xAAu8; 70];
        assert!(fill_random(&mut out).is_ok());
        assert!(fill_random(&mut out[..3]).is_ok());
    }
}
