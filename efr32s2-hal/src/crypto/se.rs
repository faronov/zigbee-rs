//! AES-128 ECB through the HSE Secure Engine mailbox (`SEMAILBOX_HOST`).
//!
//! xG21 has no host-side AES core: symmetric crypto runs inside the Secure
//! Engine. One request is the GSDK 4.5 `sl_se_aes_crypt_ecb` command with a
//! plaintext key, as `em_se.c SE_executeCommand` posts it:
//!
//! ```text
//! TX_HEADER = 4 * (4 + 2)                (bytes incl. header, 2 parameters)
//! FIFO      = 0x0400_0100                (AES_ENCRYPT | MODE_ECB | CONTEXT_WHOLE)
//! FIFO      = &auth_desc                 (input descriptor list)
//! FIFO      = &out_desc                  (output descriptor list)
//! FIFO      = 0x0000_0010                (keyspec: symmetric, unprotected, 16 B)
//! FIFO      = 16                         (message length)
//! input  list: auth (0 B, REALIGN) -> key (16 B, REALIGN) -> block (16 B)
//! output list: block (16 B)
//! ```
//!
//! then waits for `RX_STATUS.RXINT` and reads the response code from
//! `RX_HEADER[19:16]` (`SE_RESPONSE_OK` = 0). The keyspec follows
//! `sli_se_key_to_keyspec` for `SL_SE_KEY_TYPE_AES_128` with
//! `SL_SE_KEY_STORAGE_EXTERNAL_PLAINTEXT`; the auth descriptor is the empty
//! one `sli_se_get_auth_buffer` adds for plaintext keys, and the key length
//! needs no padding (16 is a multiple of 4, `SLI_SE_KEY_PADDING_REQUIRED`).
//!
//! The register layout (FIFO at 0x00, TX/RX status at 0x40/0x44, headers at
//! 0x50/0x54) and the command encoding are the same on xG24, whose
//! `SEMAILBOX_HOST` bus clock must additionally be opened in `CMU_CLKEN1`
//! (`SeBus::enable_clock`); xG21 has no bus clock gates.
//!
//! Every wait is bounded. A command the SE does not answer in time poisons
//! the engine: its late response would otherwise be read as the answer to
//! the next command. See the parent module for the shared contract and the
//! ownership rules (Gecko Bootloader application interface).

use super::{AesError, dma_fence, poll, run_self_test, wipe};

/// `SEMAILBOX_HOST` register offsets (`efr32mg21_semailbox.h`).
pub(crate) mod reg {
    /// `FIFO[0].DATA` (xG21) / `FIFO` (xG24).
    pub const FIFO: u32 = 0x00;
    pub const TX_STATUS: u32 = 0x40;
    pub const RX_STATUS: u32 = 0x44;
    pub const TX_HEADER: u32 = 0x50;
    pub const RX_HEADER: u32 = 0x54;
}

/// `SEMAILBOX_TX_STATUS_TXINT`: the mailbox can take a new command.
pub(crate) const TX_STATUS_TXINT: u32 = 1 << 20;
/// `SEMAILBOX_TX_STATUS_TXERROR`.
pub(crate) const TX_STATUS_TXERROR: u32 = 1 << 23;
/// `SEMAILBOX_RX_STATUS_RXINT`: a response is available.
pub(crate) const RX_STATUS_RXINT: u32 = 1 << 20;
/// `SEMAILBOX_RX_STATUS_RXERROR`.
pub(crate) const RX_STATUS_RXERROR: u32 = 1 << 23;
/// `SE_RESPONSE_MASK`.
pub(crate) const RESPONSE_MASK: u32 = 0x000F_0000;

/// `SLI_SE_COMMAND_AES_ENCRYPT | SLI_SE_COMMAND_OPTION_MODE_ECB |
/// SLI_SE_COMMAND_OPTION_CONTEXT_WHOLE`.
pub(crate) const COMMAND_AES_ENCRYPT_ECB: u32 = 0x0400_0000 | 0x0000_0100;
/// Keyspec of an unprotected (plaintext) `SL_SE_KEY_TYPE_AES_128` key:
/// type symmetric [31:28] = 0, mode unprotected [27:26] = 0, no
/// restriction, slot 0, attributes [14:0] = key size 16.
pub(crate) const KEYSPEC_AES128_PLAINTEXT: u32 = 0x0000_0010;
/// Command parameters: keyspec and message length.
const PARAMETERS: u32 = 2;
/// `TX_HEADER`: command size in bytes, `sizeof(uint32_t) * (4 + params)`.
pub(crate) const TX_HEADER_BYTES: u32 = 4 * (4 + PARAMETERS);
/// `SE_DATATRANSFER_STOP`: end of a descriptor list.
pub(crate) const DATATRANSFER_STOP: u32 = 0x0000_0001;
/// `SE_DATATRANSFER_REALIGN`.
pub(crate) const DATATRANSFER_REALIGN: u32 = 0x2000_0000;
const BLOCK: u32 = 16;

/// `SE_DataTransfer_t`: `{ data, next, length }`, 32-bit bus addresses.
#[repr(C)]
#[derive(Clone, Copy)]
pub(crate) struct Descriptor {
    pub data: u32,
    pub next: u32,
    pub length: u32,
}

impl Descriptor {
    const ZERO: Self = Self {
        data: 0,
        next: 0,
        length: 0,
    };
}

/// RAM shared with the SE DMA for one request.
#[repr(C, align(4))]
pub(crate) struct SeDma {
    pub auth: Descriptor,
    pub key_desc: Descriptor,
    pub data_in: Descriptor,
    pub data_out: Descriptor,
    pub key: [u8; 16],
    pub input: [u8; 16],
    pub output: [u8; 16],
}

const _: () = assert!(core::mem::size_of::<SeDma>() == 4 * 12 + 3 * 16);

impl SeDma {
    pub(crate) const ZERO: Self = Self {
        auth: Descriptor::ZERO,
        key_desc: Descriptor::ZERO,
        data_in: Descriptor::ZERO,
        data_out: Descriptor::ZERO,
        key: [0; 16],
        input: [0; 16],
        output: [0; 16],
    };
}

/// Mailbox register access; MMIO on the target, a model in host tests.
pub(crate) trait SeBus {
    fn read(&mut self, offset: u32) -> u32;
    fn write(&mut self, offset: u32, value: u32);
    /// Bus address of `ptr` as the SE DMA sees it.
    fn bus_address(&self, ptr: *const u8) -> u32;
    /// Open the mailbox bus clock (xG24 `CMU_CLKEN1.SEMAILBOXHOST`).
    fn enable_clock(&mut self) {}
    /// Take the mailbox from other SE clients of this image (xG24: the SE
    /// TRNG); [`AesError::MailboxBusy`] if one holds it or it is poisoned.
    fn lock(&mut self) -> Result<(), AesError> {
        Ok(())
    }
    fn unlock(&mut self) {}
    /// Mark the mailbox unusable for every client (a command timed out).
    fn poison_mailbox(&mut self) {}
}

/// Mailbox protocol, independent of the register transport.
pub(crate) struct SeAes<B: SeBus> {
    bus: B,
    /// Exclusively owned (singleton token on the target); never borrowed
    /// across a command, because the SE writes it by DMA.
    dma: *mut SeDma,
    timeout: u32,
    poisoned: bool,
    verify_before_use: bool,
}

impl<B: SeBus> SeAes<B> {
    /// # Safety
    ///
    /// `dma` must be valid for the whole life of the returned value and of
    /// any command it started, and nothing else may access it.
    pub(crate) unsafe fn new(bus: B, dma: *mut SeDma, timeout: u32) -> Self {
        Self {
            bus,
            dma,
            timeout,
            poisoned: false,
            verify_before_use: false,
        }
    }

    #[cfg(test)]
    pub(crate) fn bus_mut(&mut self) -> &mut B {
        &mut self.bus
    }

    /// Encrypt one block; after an earlier fault, re-run the KAT first.
    pub(crate) fn encrypt(
        &mut self,
        key: &[u8; 16],
        input: &[u8; 16],
        output: &mut [u8; 16],
    ) -> Result<(), AesError> {
        if self.verify_before_use {
            self.self_test()?;
        }
        self.run(key, input, output)
    }

    pub(crate) fn self_test(&mut self) -> Result<(), AesError> {
        let result = run_self_test(|k, i, o| self.run(k, i, o));
        self.verify_before_use = result.is_err();
        result
    }

    fn run(
        &mut self,
        key: &[u8; 16],
        input: &[u8; 16],
        output: &mut [u8; 16],
    ) -> Result<(), AesError> {
        if self.poisoned {
            return Err(AesError::Poisoned);
        }
        self.bus.lock()?;
        let result = self.run_locked(key, input, output);
        if self.poisoned {
            // Other mailbox clients must not trust it either.
            self.bus.poison_mailbox();
        }
        self.bus.unlock();
        result
    }

    fn run_locked(
        &mut self,
        key: &[u8; 16],
        input: &[u8; 16],
        output: &mut [u8; 16],
    ) -> Result<(), AesError> {
        let timeout = self.timeout;
        self.bus.enable_clock();
        let bus = &mut self.bus;
        // Nothing is submitted yet: a busy mailbox is a plain timeout.
        poll(timeout, || bus.read(reg::TX_STATUS) & TX_STATUS_TXINT != 0)?;
        // A response nobody read (left by an earlier client) would be taken
        // for ours: drain it. A foreign command still in flight cannot
        // exist: clients are serialized by the mailbox lock, and a command
        // that timed out poisons the mailbox.
        if self.bus.read(reg::RX_STATUS) & RX_STATUS_RXINT != 0 {
            let _ = self.bus.read(reg::RX_HEADER);
        }

        let (auth, data_out) = self.fill(key, input);
        dma_fence();
        self.bus.write(reg::TX_HEADER, TX_HEADER_BYTES);
        for word in [
            COMMAND_AES_ENCRYPT_ECB,
            auth,
            data_out,
            KEYSPEC_AES128_PLAINTEXT,
            BLOCK,
        ] {
            self.bus.write(reg::FIFO, word);
        }
        if self.bus.read(reg::TX_STATUS) & TX_STATUS_TXERROR != 0 {
            // Whether the SE will still answer is unknown.
            self.poisoned = true;
            // SAFETY: exclusively ours; a late SE access only sees zeros.
            let dma = unsafe { &mut *self.dma };
            wipe(&mut dma.key);
            wipe(&mut dma.input);
            return Err(AesError::MailboxError);
        }
        let bus = &mut self.bus;
        if poll(timeout, || bus.read(reg::RX_STATUS) & RX_STATUS_RXINT != 0).is_err() {
            // The SE may still complete the command later; its output lands
            // in the static DMA area and is never published. Wipe the key
            // copy and refuse all further work.
            self.poisoned = true;
            // SAFETY: exclusively ours; a late SE access only sees zeros.
            let dma = unsafe { &mut *self.dma };
            wipe(&mut dma.key);
            wipe(&mut dma.input);
            return Err(AesError::Timeout);
        }
        let rx_status = self.bus.read(reg::RX_STATUS);
        let response = (self.bus.read(reg::RX_HEADER) & RESPONSE_MASK) >> 16;
        dma_fence();

        // SAFETY: the command completed, so the SE no longer accesses the
        // area; `self.dma` is exclusively ours (see `new`).
        let dma = unsafe { &mut *self.dma };
        let result = if rx_status & RX_STATUS_RXERROR != 0 {
            Err(AesError::MailboxError)
        } else if response != 0 {
            Err(AesError::SeResponse(response as u8))
        } else {
            output.copy_from_slice(&dma.output);
            Ok(())
        };
        wipe(&mut dma.key);
        wipe(&mut dma.input);
        wipe(&mut dma.output);
        if result.is_err() {
            self.verify_before_use = true;
        }
        result
    }

    /// Build the descriptor lists; returns the bus addresses of the input
    /// and output list heads.
    fn fill(&mut self, key: &[u8; 16], input: &[u8; 16]) -> (u32, u32) {
        // SAFETY: no command is in flight (the mailbox accepted a new one
        // and the previous one completed or poisoned the engine).
        let dma = unsafe { &mut *self.dma };
        dma.key = *key;
        dma.input = *input;
        dma.output = [0; 16];
        let bus = &self.bus;
        let addr = |p: *const u8| bus.bus_address(p);
        let key_desc = addr((&raw const dma.key_desc).cast());
        let data_in = addr((&raw const dma.data_in).cast());
        dma.auth = Descriptor {
            data: 0,
            next: key_desc,
            length: DATATRANSFER_REALIGN,
        };
        dma.key_desc = Descriptor {
            data: addr(dma.key.as_ptr()),
            next: data_in,
            length: BLOCK | DATATRANSFER_REALIGN,
        };
        dma.data_in = Descriptor {
            data: addr(dma.input.as_ptr()),
            next: DATATRANSFER_STOP,
            length: BLOCK,
        };
        dma.data_out = Descriptor {
            data: addr(dma.output.as_ptr()),
            next: DATATRANSFER_STOP,
            length: BLOCK,
        };
        (
            addr((&raw const dma.auth).cast()),
            addr((&raw const dma.data_out).cast()),
        )
    }
}

#[cfg(all(target_arch = "arm", target_os = "none"))]
mod hw {
    /// `SEMAILBOX_S_HOST_BASE` (efr32mg21a010f1024im32.h; same on xG24).
    const SEMAILBOX_HOST_BASE: u32 = 0x4C00_0000;

    pub(crate) struct Mmio;

    impl super::SeBus for Mmio {
        fn read(&mut self, offset: u32) -> u32 {
            // SAFETY: SEMAILBOX_HOST register, owned through the token.
            unsafe { core::ptr::read_volatile((SEMAILBOX_HOST_BASE + offset) as *const u32) }
        }
        fn write(&mut self, offset: u32, value: u32) {
            // SAFETY: SEMAILBOX_HOST register, owned through the token.
            unsafe { core::ptr::write_volatile((SEMAILBOX_HOST_BASE + offset) as *mut u32, value) }
        }
        fn bus_address(&self, ptr: *const u8) -> u32 {
            ptr as u32
        }
    }
}

/// DMA area of the single engine (see the parent module).
#[cfg(all(target_arch = "arm", target_os = "none"))]
static mut SE_DMA: SeDma = SeDma::ZERO;

/// The Secure Engine AES-128 service, owned through the [`super::Crypto`]
/// token.
#[cfg(all(target_arch = "arm", target_os = "none"))]
pub struct AesEngine {
    inner: SeAes<hw::Mmio>,
    _token: super::Crypto,
}

#[cfg(all(target_arch = "arm", target_os = "none"))]
impl AesEngine {
    /// Conservative bound for one mailbox wait.
    pub const DEFAULT_TIMEOUT_ITERATIONS: u32 = super::DEFAULT_TIMEOUT_ITERATIONS;

    /// Take ownership of the SE AES service. Does not talk to the SE; run
    /// [`Self::self_test`] before the first use.
    pub fn new(token: super::Crypto, timeout_iterations: u32) -> Result<Self, AesError> {
        let timeout = super::validate_timeout_iterations(timeout_iterations)?;
        // SAFETY: the `Crypto` token is a singleton consumed here, so this is
        // the only user of `SE_DMA`, for the rest of the program.
        let inner = unsafe { SeAes::new(hw::Mmio, &raw mut SE_DMA, timeout) };
        Ok(Self {
            inner,
            _token: token,
        })
    }

    /// Encrypt one standard AES-128 block in place. On error `block` is
    /// left unchanged.
    pub fn encrypt_block(&mut self, key: &[u8; 16], block: &mut [u8; 16]) -> Result<(), AesError> {
        let input = *block;
        self.inner.encrypt(key, &input, block)
    }

    /// Run two known-answer vectors with different keys on the SE.
    pub fn self_test(&mut self) -> Result<(), AesError> {
        self.inner.self_test()
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::super::test_support::{AddressMap, Provider, reference_encrypt};
    use super::*;
    use std::boxed::Box;
    use std::vec::Vec;

    /// Behavioural SE mailbox: collects one command from the FIFO, walks the
    /// descriptor lists through the fake bus addresses, encrypts with the
    /// reference AES and posts a response. Faults are injectable.
    struct Model {
        map: AddressMap,
        fifo: Vec<u32>,
        header: Option<u32>,
        rx_ready: bool,
        rx_header: u32,
        commands: u32,
        tx_busy: bool,
        never_answer: bool,
        response: u32,
        rx_error: bool,
        tx_error: bool,
        corrupt: bool,
        clock_enables: u32,
        locked: bool,
        mailbox_poisoned: bool,
    }

    impl Model {
        fn new(dma: *mut SeDma) -> Self {
            Self {
                map: AddressMap::new(dma),
                fifo: Vec::new(),
                header: None,
                rx_ready: false,
                rx_header: 0,
                commands: 0,
                tx_busy: false,
                never_answer: false,
                response: 0,
                rx_error: false,
                tx_error: false,
                corrupt: false,
                clock_enables: 0,
                locked: false,
                mailbox_poisoned: false,
            }
        }

        fn read_list(&self, mut address: u32, out: &mut Vec<u8>) {
            while address != DATATRANSFER_STOP {
                let d = self.map.to_host(address) as *const Descriptor;
                // SAFETY: the address maps into the live DMA area.
                let d = unsafe { *d };
                let len = (d.length & 0x0FFF_FFFF) as usize;
                for i in 0..len {
                    // SAFETY: as above.
                    out.push(unsafe { *self.map.to_host(d.data + i as u32) });
                }
                address = d.next;
            }
        }

        fn execute(&mut self) {
            let words = core::mem::take(&mut self.fifo);
            assert_eq!(self.header.take(), Some(TX_HEADER_BYTES));
            assert_eq!(words.len(), 5, "command, two lists, two parameters");
            self.commands += 1;
            if self.never_answer {
                return;
            }
            assert_eq!(words[0], COMMAND_AES_ENCRYPT_ECB);
            assert_eq!(words[3], KEYSPEC_AES128_PLAINTEXT);
            assert_eq!(words[4], 16);
            let mut input = Vec::new();
            self.read_list(words[1], &mut input);
            assert_eq!(input.len(), 32, "empty auth, 16 B key, 16 B block");
            let key: [u8; 16] = input[..16].try_into().unwrap();
            let block: [u8; 16] = input[16..].try_into().unwrap();
            let mut result = reference_encrypt(&key, &block);
            if self.corrupt {
                result[0] ^= 1;
            }
            if self.response == 0 {
                let out = self.map.to_host(words[2]) as *const Descriptor;
                // SAFETY: maps into the DMA area.
                let out = unsafe { *out };
                assert_eq!(out.next, DATATRANSFER_STOP);
                assert_eq!(out.length, 16);
                for (i, b) in result.iter().enumerate() {
                    // SAFETY: maps into the DMA area.
                    unsafe { *self.map.to_host(out.data + i as u32) = *b };
                }
            }
            self.rx_header = self.response << 16;
            self.rx_ready = true;
        }
    }

    impl SeBus for Model {
        fn read(&mut self, offset: u32) -> u32 {
            match offset {
                reg::TX_STATUS => {
                    let mut v = 0;
                    if !self.tx_busy {
                        v |= TX_STATUS_TXINT;
                    }
                    if self.tx_error {
                        v |= TX_STATUS_TXERROR;
                    }
                    v
                }
                reg::RX_STATUS => {
                    let mut v = 0;
                    if self.rx_ready {
                        v |= RX_STATUS_RXINT;
                    }
                    if self.rx_error {
                        v |= RX_STATUS_RXERROR;
                    }
                    v
                }
                reg::RX_HEADER => {
                    self.rx_ready = false;
                    self.rx_header
                }
                _ => 0,
            }
        }
        fn write(&mut self, offset: u32, value: u32) {
            match offset {
                reg::TX_HEADER => {
                    assert!(self.header.is_none(), "header written twice");
                    self.header = Some(value);
                }
                reg::FIFO => {
                    self.fifo.push(value);
                    if self.fifo.len() as u32 * 4 + 4 == TX_HEADER_BYTES {
                        self.execute();
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
        fn lock(&mut self) -> Result<(), AesError> {
            if self.locked || self.mailbox_poisoned {
                return Err(AesError::MailboxBusy);
            }
            self.locked = true;
            Ok(())
        }
        fn unlock(&mut self) {
            self.locked = false;
        }
        fn poison_mailbox(&mut self) {
            self.mailbox_poisoned = true;
        }
    }

    fn engine(timeout: u32) -> SeAes<Model> {
        let dma = Box::into_raw(Box::new(SeDma::ZERO));
        // SAFETY: leaked, exclusively used by this engine.
        unsafe { SeAes::new(Model::new(dma), dma, timeout) }
    }

    fn dma_is_wiped(e: &SeAes<Model>) -> bool {
        // SAFETY: no command in flight.
        let dma = unsafe { &*e.dma };
        dma.key == [0; 16] && dma.input == [0; 16] && dma.output == [0; 16]
    }

    #[test]
    fn self_test_and_command_layout() {
        let mut e = engine(8);
        e.self_test().unwrap();
        assert_eq!(e.bus.commands, 2);
        assert!(e.bus.clock_enables >= 2);
        assert!(dma_is_wiped(&e));
    }

    #[test]
    fn passes_zigbee_crypto_kat_suite() {
        let mut e = engine(8);
        let mut provider = Provider(|key: &[u8; 16], block: &mut [u8; 16]| {
            let input = *block;
            e.encrypt(key, &input, block)
        });
        zigbee_crypto::kat::run_all(&mut provider).unwrap();
    }

    #[test]
    fn rejects_zero_timeout() {
        assert_eq!(
            super::super::validate_timeout_iterations(0),
            Err(AesError::ZeroIterations)
        );
    }

    #[test]
    fn busy_mailbox_times_out_without_submitting() {
        let mut e = engine(4);
        e.bus_mut().tx_busy = true;
        let mut out = [0xA5; 16];
        assert_eq!(
            e.encrypt(&[0; 16], &[0; 16], &mut out),
            Err(AesError::Timeout)
        );
        assert_eq!(out, [0xA5; 16]);
        assert_eq!(e.bus.commands, 0);
        // Not poisoned: nothing was submitted.
        e.bus_mut().tx_busy = false;
        e.encrypt(&[0; 16], &[0; 16], &mut out).unwrap();
    }

    #[test]
    fn unanswered_command_poisons_the_engine() {
        let mut e = engine(4);
        e.bus_mut().never_answer = true;
        let mut out = [0x5A; 16];
        assert_eq!(
            e.encrypt(&[1; 16], &[2; 16], &mut out),
            Err(AesError::Timeout)
        );
        assert_eq!(out, [0x5A; 16]);
        e.bus_mut().never_answer = false;
        assert_eq!(
            e.encrypt(&[1; 16], &[2; 16], &mut out),
            Err(AesError::Poisoned)
        );
        assert_eq!(e.bus.commands, 1);
        assert_eq!(out, [0x5A; 16]);
    }

    #[test]
    fn se_error_response_is_reported_then_kat_rerun() {
        let mut e = engine(4);
        e.bus_mut().response = 7;
        let mut out = [0x11; 16];
        assert_eq!(
            e.encrypt(&[1; 16], &[2; 16], &mut out),
            Err(AesError::SeResponse(7))
        );
        assert_eq!(out, [0x11; 16]);
        assert!(dma_is_wiped(&e));
        e.bus_mut().response = 0;
        let before = e.bus.commands;
        e.encrypt(&[1; 16], &[2; 16], &mut out).unwrap();
        // Two KAT commands precede the request after a fault.
        assert_eq!(e.bus.commands, before + 3);
        assert_eq!(out, reference_encrypt(&[1; 16], &[2; 16]));
    }

    #[test]
    fn stale_response_is_drained_before_submitting() {
        let mut e = engine(4);
        // An unread response with an error code from an earlier client.
        e.bus_mut().rx_ready = true;
        e.bus_mut().rx_header = 7 << 16;
        let mut out = [0; 16];
        e.encrypt(&[1; 16], &[2; 16], &mut out).unwrap();
        assert_eq!(out, reference_encrypt(&[1; 16], &[2; 16]));
    }

    #[test]
    fn busy_or_poisoned_mailbox_is_refused_and_poison_is_shared() {
        let mut e = engine(4);
        e.bus_mut().locked = true;
        let mut out = [0x66; 16];
        assert_eq!(
            e.encrypt(&[1; 16], &[2; 16], &mut out),
            Err(AesError::MailboxBusy)
        );
        assert_eq!(e.bus.commands, 0);
        e.bus_mut().locked = false;
        e.bus_mut().never_answer = true;
        assert_eq!(
            e.encrypt(&[1; 16], &[2; 16], &mut out),
            Err(AesError::Timeout)
        );
        assert!(
            e.bus.mailbox_poisoned,
            "a timeout poisons the shared mailbox"
        );
        assert!(!e.bus.locked, "the lock is released after the failure");
        assert_eq!(out, [0x66; 16]);
    }

    #[test]
    fn rx_error_flag_fails_closed() {
        let mut e = engine(4);
        e.bus_mut().rx_error = true;
        let mut out = [0x22; 16];
        assert_eq!(
            e.encrypt(&[1; 16], &[2; 16], &mut out),
            Err(AesError::MailboxError)
        );
        assert_eq!(out, [0x22; 16]);
    }

    #[test]
    fn tx_error_flag_poisons() {
        let mut e = engine(4);
        e.bus_mut().tx_error = true;
        let mut out = [0x33; 16];
        assert_eq!(
            e.encrypt(&[1; 16], &[2; 16], &mut out),
            Err(AesError::MailboxError)
        );
        e.bus_mut().tx_error = false;
        assert_eq!(
            e.encrypt(&[1; 16], &[2; 16], &mut out),
            Err(AesError::Poisoned)
        );
        assert_eq!(out, [0x33; 16]);
    }

    #[test]
    fn wrong_engine_output_fails_the_kat() {
        let mut e = engine(4);
        e.bus_mut().corrupt = true;
        assert_eq!(e.self_test(), Err(AesError::KnownAnswerMismatch));
        // A later request must pass the KAT first.
        let mut out = [0x44; 16];
        assert_eq!(
            e.encrypt(&[1; 16], &[2; 16], &mut out),
            Err(AesError::KnownAnswerMismatch)
        );
        assert_eq!(out, [0x44; 16]);
    }
}
