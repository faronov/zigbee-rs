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
//!
//! DMA memory and mailbox ownership: the SE reads the output descriptor and
//! writes the random bytes by DMA, possibly *after* our bounded wait gave
//! up. Both therefore live in a `static` area ([`TrngDma`]) that only the
//! holder of the mailbox lock touches, never on the caller's stack. A
//! command that was submitted but not answered in time (or rejected with
//! `TX_STATUS.TXERROR`) poisons the mailbox: no client sends another
//! command and nobody reads or reuses the DMA area until reset, so a late
//! completion only writes memory nobody uses, and its response cannot be
//! taken for the answer to a later command. Nothing is published from a
//! failed request: the caller's buffer is zeroed.
//!
//! The AES engine (`crate::crypto::se`) is the other client of this
//! mailbox and follows the same discipline: it takes the lock through
//! `lock_mailbox`, keeps its descriptors and data in its own owned static
//! area (`SE_DMA`) and poisons the mailbox for both clients through
//! `poison_mailbox` when one of its commands is not answered.

use core::sync::atomic::{AtomicBool, Ordering};

/// `SEMAILBOX_HOST` register offsets (`efr32mg24_semailbox.h`).
mod reg {
    pub const FIFO: u32 = 0x00;
    pub const TX_STATUS: u32 = 0x40;
    pub const RX_STATUS: u32 = 0x44;
    pub const TX_HEADER: u32 = 0x50;
    pub const RX_HEADER: u32 = 0x54;
}

/// `SEMAILBOX_S_HOST_BASE` (efr32mg24a410f1536im48.h).
#[cfg(all(target_arch = "arm", target_os = "none"))]
const SEMAILBOX_HOST_BASE: u32 = 0x4C00_0000;
/// `TX_STATUS.TXINT`: the mailbox can take a new command.
const TX_STATUS_TXINT: u32 = 1 << 20;
/// `TX_STATUS.TXERROR`: the command was not accepted.
const TX_STATUS_TXERROR: u32 = 1 << 23;
/// `RX_STATUS.RXINT`: a response is available.
const RX_STATUS_RXINT: u32 = 1 << 20;
/// `RX_STATUS.RXERROR`.
const RX_STATUS_RXERROR: u32 = 1 << 23;
const RESPONSE_MASK: u32 = 0x000F_0000;

const COMMAND_TRNG_GET_RANDOM: u32 = 0x0700_0000;
const DATATRANSFER_STOP: u32 = 0x0000_0001;
const DATATRANSFER_REALIGN: u32 = 0x2000_0000;

/// Bytes per mailbox command (a multiple of four; the stack asks for at
/// most 18 at a time).
pub const CHUNK: usize = 64;
/// Bounded polls of the mailbox status (the SE answers in microseconds;
/// this is about 50 ms at 39 MHz).
#[cfg(all(target_arch = "arm", target_os = "none"))]
const TIMEOUT_POLLS: u32 = 2_000_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SeError {
    /// Another caller holds the mailbox.
    Busy,
    /// The mailbox did not become free or the SE did not answer.
    Timeout,
    /// An earlier command was not answered (or not accepted); the mailbox
    /// is unusable until reset.
    Poisoned,
    /// The mailbox reported `TX_STATUS.TXERROR` / `RX_STATUS.RXERROR`.
    MailboxError,
    /// The SE answered with this non-OK result code (`RX_HEADER` bits
    /// 19:16, shifted down).
    Response(u8),
    /// Host build: there is no Secure Engine.
    Unavailable,
}

/// Ownership of the SE mailbox and of every DMA area a client hands to the
/// SE: one request at a time; a submitted command that was not answered
/// poisons it for good.
pub(crate) struct MailboxLock {
    busy: AtomicBool,
    poisoned: AtomicBool,
}

impl MailboxLock {
    pub(crate) const fn new() -> Self {
        Self {
            busy: AtomicBool::new(false),
            poisoned: AtomicBool::new(false),
        }
    }

    /// Take the mailbox for one request.
    pub(crate) fn lock(&self) -> Result<(), SeError> {
        if self.poisoned.load(Ordering::Acquire) {
            return Err(SeError::Poisoned);
        }
        self.busy
            .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
            .map(|_| ())
            .map_err(|_| SeError::Busy)
    }

    /// Release the mailbox after [`Self::lock`].
    pub(crate) fn unlock(&self) {
        self.busy.store(false, Ordering::Release);
    }

    /// Refuse every later request of every client (until reset).
    pub(crate) fn poison(&self) {
        self.poisoned.store(true, Ordering::Release);
    }

    #[cfg(test)]
    fn is_poisoned(&self) -> bool {
        self.poisoned.load(Ordering::Acquire)
    }
}

/// The image's only SE mailbox. Thread mode only.
#[cfg(all(target_arch = "arm", target_os = "none"))]
static MAILBOX: MailboxLock = MailboxLock::new();

/// Take the mailbox for one request of another SE client (the AES engine,
/// `crate::crypto::se`). The client must keep every DMA area it hands to
/// the SE in owned static memory, like `TRNG_DMA`.
#[cfg(all(target_arch = "arm", target_os = "none"))]
pub(crate) fn lock_mailbox() -> Result<(), SeError> {
    MAILBOX.lock()
}

/// Release the mailbox after [`lock_mailbox`].
#[cfg(all(target_arch = "arm", target_os = "none"))]
pub(crate) fn unlock_mailbox() {
    MAILBOX.unlock();
}

/// A client's command was not answered (or not accepted): refuse every
/// later request of every client until reset.
#[cfg(all(target_arch = "arm", target_os = "none"))]
pub(crate) fn poison_mailbox() {
    MAILBOX.poison();
}

/// `SE_DataTransfer_t`.
#[repr(C)]
#[derive(Clone, Copy)]
struct DataTransfer {
    data: u32,
    next: u32,
    length: u32,
}

/// RAM shared with the SE DMA for one TRNG command: the output descriptor
/// and the buffer it points to.
#[repr(C, align(4))]
pub(crate) struct TrngDma {
    descriptor: DataTransfer,
    words: [u32; CHUNK / 4],
}

impl TrngDma {
    pub(crate) const ZERO: Self = Self {
        descriptor: DataTransfer {
            data: 0,
            next: 0,
            length: 0,
        },
        words: [0; CHUNK / 4],
    };
}

/// DMA area of the TRNG. Accessed only by [`fill_random`] while it holds
/// [`MAILBOX`]; after a poisoning timeout never again, so a late SE write
/// cannot reach live data.
#[cfg(all(target_arch = "arm", target_os = "none"))]
static mut TRNG_DMA: TrngDma = TrngDma::ZERO;

/// Mailbox register access: MMIO on the target, a model in host tests.
pub(crate) trait SeBus {
    fn read(&mut self, offset: u32) -> u32;
    fn write(&mut self, offset: u32, value: u32);
    /// Bus address of `ptr` as the SE DMA sees it.
    fn bus_address(&self, ptr: *const u8) -> u32;
    /// Open the mailbox bus clock (`CMU_CLKEN1.SEMAILBOXHOST`).
    fn enable_clock(&mut self) {}
}

#[cfg(all(target_arch = "arm", target_os = "none"))]
struct Mmio;

#[cfg(all(target_arch = "arm", target_os = "none"))]
impl SeBus for Mmio {
    fn read(&mut self, offset: u32) -> u32 {
        // SAFETY: SEMAILBOX_HOST register; the caller holds the mailbox.
        unsafe { core::ptr::read_volatile((SEMAILBOX_HOST_BASE + offset) as *const u32) }
    }
    fn write(&mut self, offset: u32, value: u32) {
        // SAFETY: SEMAILBOX_HOST register; the caller holds the mailbox.
        unsafe { core::ptr::write_volatile((SEMAILBOX_HOST_BASE + offset) as *mut u32, value) }
    }
    fn bus_address(&self, ptr: *const u8) -> u32 {
        ptr as u32
    }
    fn enable_clock(&mut self) {
        crate::clock::enable_bus_clock(crate::chip::BusClock::SEMAILBOXHOST);
    }
}

/// The message words of one TRNG request, in FIFO order after the header.
const fn trng_message(descriptor: u32, length: u32) -> (u32, [u32; 4]) {
    (
        4 * (4 + 1),
        [COMMAND_TRNG_GET_RANDOM, 0, descriptor, length],
    )
}

/// Order the CPU's DMA-area stores before the MMIO write that starts the
/// SE, and the SE's DMA writes before the CPU reads them.
#[inline(always)]
fn dma_fence() {
    core::sync::atomic::compiler_fence(Ordering::SeqCst);
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    cortex_m::asm::dsb();
}

fn poll(iterations: u32, mut done: impl FnMut() -> bool) -> Result<(), SeError> {
    for _ in 0..iterations {
        if done() {
            return Ok(());
        }
        core::hint::spin_loop();
    }
    Err(SeError::Timeout)
}

/// Fill `output` with Secure Engine TRNG bytes. On error `output` is
/// zeroed.
#[cfg(all(target_arch = "arm", target_os = "none"))]
pub fn fill_random(output: &mut [u8]) -> Result<(), SeError> {
    // SAFETY: `TRNG_DMA` is a static used only here, under `MAILBOX`.
    unsafe {
        fill_random_with(
            &mut Mmio,
            &MAILBOX,
            &raw mut TRNG_DMA,
            TIMEOUT_POLLS,
            output,
        )
    }
}

/// Host build: no Secure Engine; fails explicitly, never fakes entropy.
#[cfg(not(all(target_arch = "arm", target_os = "none")))]
pub fn fill_random(output: &mut [u8]) -> Result<(), SeError> {
    output.fill(0);
    Err(SeError::Unavailable)
}

/// # Safety
///
/// `dma` must stay valid for the rest of the program (a timed-out command
/// may still be completed by the SE) and be accessed only by holders of
/// `lock`.
unsafe fn fill_random_with<B: SeBus>(
    bus: &mut B,
    lock: &MailboxLock,
    dma: *mut TrngDma,
    timeout: u32,
    output: &mut [u8],
) -> Result<(), SeError> {
    if let Err(error) = lock.lock() {
        output.fill(0);
        return Err(error);
    }
    bus.enable_clock();
    let mut result = Ok(());
    for chunk in output.chunks_mut(CHUNK) {
        let length = chunk.len().next_multiple_of(4) as u32;
        // SAFETY: we hold `lock`; see the function contract.
        match unsafe { request(bus, dma, timeout, length) } {
            Ok(()) => {
                // SAFETY: the command completed; we hold `lock`.
                let words = unsafe { &raw mut (*dma).words };
                for (index, byte) in chunk.iter_mut().enumerate() {
                    // SAFETY: in bounds (`index < CHUNK`), written by the SE.
                    let word = unsafe { core::ptr::read_volatile(&raw const (*words)[index / 4]) };
                    *byte = word.to_le_bytes()[index % 4];
                }
                for index in 0..CHUNK / 4 {
                    // SAFETY: as above; wipe the copy of the random bytes.
                    unsafe { core::ptr::write_volatile(&raw mut (*words)[index], 0) };
                }
            }
            Err(fault) => {
                if fault.in_flight {
                    // The SE may still answer and write the DMA area: no
                    // client may use the mailbox or the area again.
                    lock.poison();
                }
                result = Err(fault.error);
                break;
            }
        }
    }
    if result.is_err() {
        output.fill(0);
    }
    lock.unlock();
    result
}

struct Fault {
    error: SeError,
    /// The command was submitted and its completion is unknown.
    in_flight: bool,
}

impl Fault {
    const fn before_submit(error: SeError) -> Self {
        Self {
            error,
            in_flight: false,
        }
    }
    const fn in_flight(error: SeError) -> Self {
        Self {
            error,
            in_flight: true,
        }
    }
    const fn completed(error: SeError) -> Self {
        Self {
            error,
            in_flight: false,
        }
    }
}

/// One TRNG command of `length` bytes into `(*dma).words`.
///
/// # Safety
///
/// As [`fill_random_with`]; the caller holds the mailbox lock.
unsafe fn request<B: SeBus>(
    bus: &mut B,
    dma: *mut TrngDma,
    timeout: u32,
    length: u32,
) -> Result<(), Fault> {
    // Nothing is submitted yet: a busy mailbox is a plain timeout.
    poll(timeout, || bus.read(reg::TX_STATUS) & TX_STATUS_TXINT != 0)
        .map_err(Fault::before_submit)?;
    // A response nobody read would be taken for ours: drain it. A foreign
    // command in flight cannot exist: clients are serialized by the lock and
    // an unanswered command poisons it.
    if bus.read(reg::RX_STATUS) & RX_STATUS_RXINT != 0 {
        let _ = bus.read(reg::RX_HEADER);
    }
    // SAFETY: no command is in flight and we hold the lock, so the area is
    // ours alone until the SE gets the descriptor address below.
    let descriptor = unsafe {
        let words = &raw mut (*dma).words;
        for index in 0..CHUNK / 4 {
            core::ptr::write_volatile(&raw mut (*words)[index], 0);
        }
        let descriptor = &raw mut (*dma).descriptor;
        core::ptr::write_volatile(
            descriptor,
            DataTransfer {
                data: bus.bus_address(words.cast()),
                next: DATATRANSFER_STOP,
                length: length | DATATRANSFER_REALIGN,
            },
        );
        bus.bus_address(descriptor.cast())
    };
    let (header, message) = trng_message(descriptor, length);
    dma_fence();
    bus.write(reg::TX_HEADER, header);
    for word in message {
        bus.write(reg::FIFO, word);
    }
    if bus.read(reg::TX_STATUS) & TX_STATUS_TXERROR != 0 {
        // Whether the SE will still run the command is unknown.
        return Err(Fault::in_flight(SeError::MailboxError));
    }
    poll(timeout, || bus.read(reg::RX_STATUS) & RX_STATUS_RXINT != 0).map_err(Fault::in_flight)?;
    let rx_status = bus.read(reg::RX_STATUS);
    let response = (bus.read(reg::RX_HEADER) & RESPONSE_MASK) >> 16;
    dma_fence();
    if rx_status & RX_STATUS_RXERROR != 0 {
        return Err(Fault::completed(SeError::MailboxError));
    }
    if response != 0 {
        return Err(Fault::completed(SeError::Response(response as u8)));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::boxed::Box;
    use std::vec::Vec;

    /// Fake 32-bit bus addresses for the host DMA area: the model only sees
    /// `u32` words, as the SE does, and panics on any address outside it.
    struct AddressMap {
        base: usize,
        len: usize,
    }

    const FAKE_RAM: u32 = 0x2000_1000;

    impl AddressMap {
        fn to_bus(&self, ptr: *const u8) -> u32 {
            let offset = (ptr as usize)
                .checked_sub(self.base)
                .filter(|o| *o < self.len)
                .expect("DMA pointer outside the TRNG DMA area");
            FAKE_RAM + offset as u32
        }
        fn to_host(&self, address: u32, len: usize) -> *mut u8 {
            let offset = address
                .checked_sub(FAKE_RAM)
                .map(|o| o as usize)
                .filter(|o| o + len <= self.len)
                .expect("SE DMA access outside the TRNG DMA area");
            (self.base + offset) as *mut u8
        }
    }

    #[derive(Clone, Copy, PartialEq)]
    enum Answer {
        Now,
        Never,
        TxError,
        Response(u8),
        RxError,
    }

    /// Behavioural SE mailbox: collects one command from the FIFO, follows
    /// the output descriptor through the fake bus addresses and writes a
    /// byte pattern by "DMA".
    struct Model {
        map: AddressMap,
        answer: Answer,
        tx_busy: bool,
        header: Option<u32>,
        fifo: Vec<u32>,
        pending: Option<(u32, u32)>,
        rx_ready: bool,
        rx_header: u32,
        tx_error: bool,
        commands: u32,
        next_byte: u8,
        clock_enables: u32,
    }

    impl Model {
        fn new(dma: *const TrngDma) -> Self {
            Self {
                map: AddressMap {
                    base: dma as usize,
                    len: core::mem::size_of::<TrngDma>(),
                },
                answer: Answer::Now,
                tx_busy: false,
                header: None,
                fifo: Vec::new(),
                pending: None,
                rx_ready: false,
                rx_header: 0,
                tx_error: false,
                commands: 0,
                next_byte: 1,
                clock_enables: 0,
            }
        }

        /// Run the pending command's DMA: read the descriptor, write the
        /// pattern, post the response (also used for a late completion).
        fn complete(&mut self, response: u8) {
            let (descriptor, length) = self.pending.take().expect("no command in flight");
            // SAFETY: `to_host` checked the range against the DMA area.
            let d =
                unsafe { core::ptr::read(self.map.to_host(descriptor, 12) as *const DataTransfer) };
            assert_eq!(d.next, DATATRANSFER_STOP);
            assert_eq!(d.length, length | DATATRANSFER_REALIGN);
            let out = self.map.to_host(d.data, length as usize);
            for i in 0..length as usize {
                // SAFETY: range checked by `to_host`.
                unsafe { out.add(i).write(self.next_byte) };
                self.next_byte = self.next_byte.wrapping_add(1);
            }
            self.rx_ready = true;
            self.rx_header = u32::from(response) << 16;
        }
    }

    impl SeBus for Model {
        fn read(&mut self, offset: u32) -> u32 {
            match offset {
                reg::TX_STATUS => {
                    let mut v = if self.tx_busy { 0 } else { TX_STATUS_TXINT };
                    if self.tx_error {
                        v |= TX_STATUS_TXERROR;
                    }
                    v
                }
                reg::RX_STATUS => {
                    let mut v = if self.rx_ready { RX_STATUS_RXINT } else { 0 };
                    if self.rx_ready && self.answer == Answer::RxError {
                        v |= RX_STATUS_RXERROR;
                    }
                    v
                }
                reg::RX_HEADER => {
                    self.rx_ready = false;
                    self.rx_header
                }
                _ => panic!("unexpected read {offset:#x}"),
            }
        }

        fn write(&mut self, offset: u32, value: u32) {
            match offset {
                reg::TX_HEADER => {
                    assert!(self.pending.is_none(), "command while one is in flight");
                    self.header = Some(value);
                    self.fifo.clear();
                }
                reg::FIFO => {
                    self.fifo.push(value);
                    let words = self.header.expect("FIFO before TX_HEADER") / 4 - 1;
                    if self.fifo.len() as u32 == words {
                        assert_eq!(self.fifo[0], COMMAND_TRNG_GET_RANDOM);
                        assert_eq!(self.fifo[1], 0);
                        self.commands += 1;
                        self.pending = Some((self.fifo[2], self.fifo[3]));
                        match self.answer {
                            Answer::Now | Answer::RxError => self.complete(0),
                            Answer::Response(code) => self.complete(code),
                            Answer::TxError => self.tx_error = true,
                            Answer::Never => {}
                        }
                    }
                }
                _ => panic!("unexpected write {offset:#x}"),
            }
        }

        fn bus_address(&self, ptr: *const u8) -> u32 {
            self.map.to_bus(ptr)
        }

        fn enable_clock(&mut self) {
            self.clock_enables += 1;
        }
    }

    /// A DMA area that, like the static one, outlives every command.
    fn leaked_dma() -> *mut TrngDma {
        Box::leak(Box::new(TrngDma::ZERO))
    }

    fn fill(
        model: &mut Model,
        lock: &MailboxLock,
        dma: *mut TrngDma,
        out: &mut [u8],
    ) -> Result<(), SeError> {
        // SAFETY: `dma` is leaked (lives forever) and only used under `lock`.
        unsafe { fill_random_with(model, lock, dma, 50, out) }
    }

    fn dma_words(dma: *mut TrngDma) -> [u32; CHUNK / 4] {
        // SAFETY: test-owned area, no command in flight in the model.
        unsafe { (*dma).words }
    }

    #[test]
    fn message_matches_em_se_layout() {
        // TX_HEADER counts the header word, command, two descriptors and
        // one parameter: 4 * (4 + 1) bytes.
        let (header, message) = trng_message(0x2000_1000, 16);
        assert_eq!(header, 20);
        assert_eq!(message, [0x0700_0000, 0, 0x2000_1000, 16]);
        assert_eq!(core::mem::size_of::<DataTransfer>(), 12);
        assert_eq!(core::mem::size_of::<TrngDma>(), 12 + CHUNK);
        assert_eq!((reg::TX_STATUS, reg::RX_HEADER), (0x40, 0x54));
    }

    #[test]
    fn output_comes_from_the_owned_dma_area_in_chunks() {
        let dma = leaked_dma();
        let lock = MailboxLock::new();
        let mut model = Model::new(dma);
        let mut out = [0u8; 70];
        assert_eq!(fill(&mut model, &lock, dma, &mut out), Ok(()));
        // 64 bytes, then 6 rounded up to 8.
        assert_eq!(model.commands, 2);
        let expected: Vec<u8> = (1..=64u8).chain(65..=70).collect();
        assert_eq!(out.as_slice(), expected.as_slice());
        // The DMA copy of the random bytes is wiped; the mailbox is free.
        assert_eq!(dma_words(dma), [0; CHUNK / 4]);
        assert_eq!(model.clock_enables, 1);
        assert_eq!(lock.lock(), Ok(()));
        assert!(!lock.is_poisoned());
    }

    #[test]
    fn late_completion_after_timeout_lands_only_in_the_owned_buffer() {
        let dma = leaked_dma();
        let lock = MailboxLock::new();
        let mut model = Model::new(dma);
        model.answer = Answer::Never;
        let mut out = [0xAAu8; 16];
        assert_eq!(
            fill(&mut model, &lock, dma, &mut out),
            Err(SeError::Timeout)
        );
        // Nothing published, the mailbox is poisoned and released.
        assert_eq!(out, [0; 16]);
        assert!(lock.is_poisoned());
        assert_eq!(model.commands, 1);

        // The SE completes the abandoned command now. `Model::complete`
        // resolves the descriptor and output addresses through the DMA
        // area's address map, so the write provably stays inside it.
        model.complete(0);
        let words = dma_words(dma);
        assert_eq!(
            words[..4],
            [0x0403_0201, 0x0807_0605, 0x0c0b_0a09, 0x100f_0e0d]
        );
        assert_eq!(out, [0; 16]);

        // Every later request is refused before it touches the mailbox or
        // the DMA area, and the stale response is never consumed.
        model.answer = Answer::Now;
        let mut again = [0x55u8; 8];
        assert_eq!(
            fill(&mut model, &lock, dma, &mut again),
            Err(SeError::Poisoned)
        );
        assert_eq!(again, [0; 8]);
        assert_eq!(model.commands, 1);
        assert!(model.rx_ready);
        assert_eq!(dma_words(dma), words);
        assert_eq!(lock.lock(), Err(SeError::Poisoned));
    }

    #[test]
    fn rejected_command_poisons_the_mailbox() {
        let dma = leaked_dma();
        let lock = MailboxLock::new();
        let mut model = Model::new(dma);
        model.answer = Answer::TxError;
        let mut out = [0xAAu8; 4];
        assert_eq!(
            fill(&mut model, &lock, dma, &mut out),
            Err(SeError::MailboxError)
        );
        assert_eq!(out, [0; 4]);
        assert!(lock.is_poisoned());
    }

    #[test]
    fn busy_mailbox_before_submit_is_a_plain_timeout() {
        let dma = leaked_dma();
        let lock = MailboxLock::new();
        let mut model = Model::new(dma);
        model.tx_busy = true;
        let mut out = [0xAAu8; 4];
        assert_eq!(
            fill(&mut model, &lock, dma, &mut out),
            Err(SeError::Timeout)
        );
        assert_eq!(out, [0; 4]);
        assert_eq!(model.commands, 0);
        // Nothing was submitted, so nothing can complete late.
        assert!(!lock.is_poisoned());
        model.tx_busy = false;
        assert_eq!(fill(&mut model, &lock, dma, &mut out), Ok(()));
    }

    #[test]
    fn se_errors_publish_nothing_and_keep_the_mailbox_usable() {
        for (answer, error) in [
            (Answer::Response(5), SeError::Response(5)),
            (Answer::RxError, SeError::MailboxError),
        ] {
            let dma = leaked_dma();
            let lock = MailboxLock::new();
            let mut model = Model::new(dma);
            model.answer = answer;
            let mut out = [0xAAu8; 70];
            assert_eq!(fill(&mut model, &lock, dma, &mut out), Err(error));
            assert_eq!(out, [0; 70]);
            // The first chunk failed; no second command was sent.
            assert_eq!(model.commands, 1);
            assert!(!lock.is_poisoned());
            model.answer = Answer::Now;
            assert_eq!(fill(&mut model, &lock, dma, &mut out[..4]), Ok(()));
        }
    }

    #[test]
    fn stale_response_is_drained_before_a_command() {
        let dma = leaked_dma();
        let lock = MailboxLock::new();
        let mut model = Model::new(dma);
        model.rx_ready = true;
        model.rx_header = 7 << 16;
        let mut out = [0u8; 4];
        assert_eq!(fill(&mut model, &lock, dma, &mut out), Ok(()));
        assert_eq!(out, [1, 2, 3, 4]);
    }

    #[test]
    fn held_mailbox_is_busy() {
        let dma = leaked_dma();
        let lock = MailboxLock::new();
        let mut model = Model::new(dma);
        assert_eq!(lock.lock(), Ok(()));
        let mut out = [0xAAu8; 4];
        assert_eq!(fill(&mut model, &lock, dma, &mut out), Err(SeError::Busy));
        assert_eq!(out, [0; 4]);
        assert_eq!(model.commands, 0);
        lock.unlock();
        assert_eq!(fill(&mut model, &lock, dma, &mut out), Ok(()));
    }

    #[test]
    fn host_build_has_no_entropy() {
        let mut out = [0xAAu8; 8];
        assert_eq!(fill_random(&mut out), Err(SeError::Unavailable));
        assert_eq!(out, [0; 8]);
    }
}
