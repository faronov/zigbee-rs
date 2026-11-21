//! AES-128 ECB on the xG22 CRYPTOACC BA411E core through the CRYPTOMASTER
//! scatter/gather DMA.
//!
//! The sequence follows GSDK 4.5 `sl_cryptoacc_library`
//! (`sx_blk_cipher_ecb` -> `sx_blk_cipher_build_descr` -> `cryptodma_run_sg`)
//! for one 16-byte block with a 128-bit key:
//!
//! ```text
//! fetch list (to the engine):
//!   config  4 B  REALIGN  tag BA411E|CONFIG|offset 0    (ENCRYPT|ECB|128 = 0x100)
//!   key    16 B  REALIGN  tag BA411E|CONFIG|offset 8
//!   block  16 B  REALIGN  tag BA411E|DATA|LAST         (next = STOP)
//! push list (from the engine):
//!   block  16 B  REALIGN  tag LAST                     (next = STOP)
//! FETCH_ADDR, PUSH_ADDR, CONFIG = FETCHER_INDIRECT | PUSHER_INDIRECT
//! INT_STAT_CLR = all, INT_EN = PUSHER_STOPPED, START = FETCH | PUSH
//! wait: PUSHER_STOPPED raised and PUSHER_BUSY clear, or a bus error
//! check: no fetcher/pusher bus error, both FIFOs empty
//! ```
//!
//! GSDK traps a DMA error with a forced hard fault; here it is an
//! [`AesError`], followed by a CRYPTOMASTER soft reset (`cryptodma_reset`).
//! The AES countermeasure mask (`sx_aes_load_mask`) is a VSE_V2 (xG27+)
//! feature; GSDK does not load it on xG22 (`SLI_MBEDTLS_DEVICE_VSE_V1`).
//!
//! Clocks: like `cryptoacc_management_acquire/release`, each request opens
//! the CRYPTOACC bus clock and the AES/PK clocks and gates them again
//! afterwards, through the reference count shared with the TRNG
//! (`crate::clock::cryptoacc_acquire`), so a request never gates the
//! clocks of a running TRNG and a TRNG release never gates an AES request.

use super::{AesError, dma_fence, poll, run_self_test, wipe};

/// CRYPTOMASTER and BA411E register offsets from `CRYPTOACC_S_BASE`
/// (`sx_dma.h`, `cryptolib_def.h`).
pub(crate) mod reg {
    pub const FETCH_ADDR: u32 = 0x00;
    pub const PUSH_ADDR: u32 = 0x10;
    pub const INT_EN: u32 = 0x1C;
    pub const INT_STAT_RAW: u32 = 0x28;
    pub const INT_STAT_CLR: u32 = 0x30;
    pub const CONFIG: u32 = 0x34;
    pub const START: u32 = 0x38;
    pub const STATUS: u32 = 0x3C;
    pub const BA411E_HW_CFG_1: u32 = 0x404;
}

pub(crate) const CONFIG_FETCHER_INDIRECT: u32 = 0x1;
pub(crate) const CONFIG_PUSHER_INDIRECT: u32 = 0x2;
pub(crate) const CONFIG_SOFTRESET: u32 = 0x10;
pub(crate) const START_FETCH_PUSH: u32 = 0x3;
pub(crate) const STATUS_FETCHER_BUSY: u32 = 0x1;
pub(crate) const STATUS_PUSHER_BUSY: u32 = 0x2;
pub(crate) const STATUS_FIFOIN_NOT_EMPTY: u32 = 0x10;
pub(crate) const STATUS_SOFT_RESET: u32 = 0x40;
pub(crate) const STATUS_FIFOOUT_NDATA: u32 = 0xFFFF_0000;
pub(crate) const RAW_FETCHER_ERROR: u32 = 0x04;
pub(crate) const RAW_PUSHER_STOPPED: u32 = 0x10;
pub(crate) const RAW_PUSHER_ERROR: u32 = 0x20;
pub(crate) const INT_ALL: u32 = 0x3F;

/// `AES_HW_CFG_ECB_SUPPORTED_MASK | AES_HW_CFG_KEY_SIZE_128_SUPPORTED_MASK`.
pub(crate) const HW_CFG_ECB_AES128: u32 = (1 << 0) | (1 << 24);

/// `DMA_AXI_DESCR_REALIGN`.
pub(crate) const DESCR_REALIGN: u32 = 0x2000_0000;
/// `DMA_AXI_DESCR_NEXT_STOP`.
pub(crate) const DESCR_NEXT_STOP: u32 = 0x1;
/// `DMA_SG_ENGINESELECT_BA411E`.
pub(crate) const TAG_BA411E: u32 = 0x01;
/// `DMA_SG_TAG_ISCONFIG`.
pub(crate) const TAG_CONFIG: u32 = 0x10;
/// `DMA_SG_TAG_ISLAST`.
pub(crate) const TAG_LAST: u32 = 0x20;
/// `DMA_SG_TAG_SETCFGOFFSET(SX_BLK_CIPHER_OFFSET_KEY)`.
pub(crate) const TAG_OFFSET_KEY: u32 = 8 << 8;
/// `SX_BLK_CIPHER_MODEID_ENCRYPT | SX_BLK_CIPHER_MODEID_ECB |
/// SX_BLK_CIPHER_MODEID_128`.
pub(crate) const MODE_ENCRYPT_ECB_128: u32 = 0x0000_0100;

/// `struct dma_sg_descr_s` with 32-bit bus addresses.
#[repr(C)]
#[derive(Clone, Copy)]
pub(crate) struct Descriptor {
    pub addr: u32,
    pub next: u32,
    pub length_irq: u32,
    pub tag: u32,
}

impl Descriptor {
    const ZERO: Self = Self {
        addr: 0,
        next: 0,
        length_irq: 0,
        tag: 0,
    };
}

/// RAM read and written by the CRYPTOMASTER DMA for one request.
#[repr(C, align(4))]
pub(crate) struct CryptoaccDma {
    pub fetch: [Descriptor; 3],
    pub push: Descriptor,
    pub config: u32,
    pub key: [u8; 16],
    pub input: [u8; 16],
    pub output: [u8; 16],
}

const _: () = assert!(core::mem::size_of::<CryptoaccDma>() == 4 * 16 + 4 + 3 * 16);

impl CryptoaccDma {
    pub(crate) const ZERO: Self = Self {
        fetch: [Descriptor::ZERO; 3],
        push: Descriptor::ZERO,
        config: 0,
        key: [0; 16],
        input: [0; 16],
        output: [0; 16],
    };
}

/// Register and clock access; MMIO on the target, a model in host tests.
pub(crate) trait CryptoaccBus {
    fn read(&mut self, offset: u32) -> u32;
    fn write(&mut self, offset: u32, value: u32);
    /// Bus address of `ptr` as the CRYPTOMASTER DMA sees it.
    fn bus_address(&self, ptr: *const u8) -> u32;
    /// Take the AES user's reference on the shared CRYPTOACC clocks.
    fn acquire_clocks(&mut self);
    /// Drop it; the clocks stop only if the TRNG does not hold them.
    fn release_clocks(&mut self);
}

/// BA411E/CRYPTOMASTER protocol, independent of the register transport.
pub(crate) struct Ba411e<B: CryptoaccBus> {
    bus: B,
    /// Exclusively owned (singleton token on the target); never borrowed
    /// across a transfer, because the DMA writes it.
    dma: *mut CryptoaccDma,
    timeout: u32,
    poisoned: bool,
    verify_before_use: bool,
}

impl<B: CryptoaccBus> Ba411e<B> {
    /// # Safety
    ///
    /// `dma` must be valid for the whole life of the returned value and of
    /// any transfer it started, and nothing else may access it.
    pub(crate) unsafe fn new(bus: B, dma: *mut CryptoaccDma, timeout: u32) -> Self {
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

    /// Check that the BA411E synthesis supports AES-128 ECB
    /// (`AES_HW_CFG_ECB_SUPPORTED`, `AES_HW_CFG_KEY_SIZE_128_SUPPORTED`).
    pub(crate) fn probe(&mut self) -> Result<(), AesError> {
        self.bus.acquire_clocks();
        let cfg = self.bus.read(reg::BA411E_HW_CFG_1);
        self.bus.release_clocks();
        if cfg & HW_CFG_ECB_AES128 == HW_CFG_ECB_AES128 {
            Ok(())
        } else {
            Err(AesError::Unsupported)
        }
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
        self.bus.acquire_clocks();
        let result = self.run_clocked(key, input, output);
        self.bus.release_clocks();
        if result.is_err() {
            self.verify_before_use = true;
        }
        result
    }

    fn run_clocked(
        &mut self,
        key: &[u8; 16],
        input: &[u8; 16],
        output: &mut [u8; 16],
    ) -> Result<(), AesError> {
        // Only this engine starts CRYPTOMASTER transfers (the TRNG reads its
        // FIFO directly); a busy DMA is left over from a fault.
        if self.bus.read(reg::STATUS) & (STATUS_FETCHER_BUSY | STATUS_PUSHER_BUSY) != 0 {
            self.soft_reset()?;
        }
        let (fetch, push) = self.fill(key, input);
        dma_fence();
        self.bus.write(reg::FETCH_ADDR, fetch);
        self.bus.write(reg::PUSH_ADDR, push);
        self.bus.write(
            reg::CONFIG,
            CONFIG_FETCHER_INDIRECT | CONFIG_PUSHER_INDIRECT,
        );
        self.bus.write(reg::INT_STAT_CLR, INT_ALL);
        self.bus.write(reg::INT_EN, RAW_PUSHER_STOPPED);
        self.bus.write(reg::START, START_FETCH_PUSH);

        let mut raw = 0;
        let bus = &mut self.bus;
        let waited = poll(self.timeout, || {
            raw = bus.read(reg::INT_STAT_RAW);
            raw & (RAW_FETCHER_ERROR | RAW_PUSHER_ERROR) != 0
                || (raw & RAW_PUSHER_STOPPED != 0
                    && bus.read(reg::STATUS) & STATUS_PUSHER_BUSY == 0)
        });
        let status = self.bus.read(reg::STATUS);
        // Drop the (unused, NVIC-disabled) interrupt line again.
        self.bus.write(reg::INT_STAT_CLR, INT_ALL);
        self.bus.write(reg::INT_EN, 0);
        dma_fence();

        let result = if waited.is_err() {
            Err(AesError::Timeout)
        } else if raw & (RAW_FETCHER_ERROR | RAW_PUSHER_ERROR) != 0 {
            Err(AesError::DmaError)
        } else if status & (STATUS_FIFOIN_NOT_EMPTY | STATUS_FIFOOUT_NDATA) != 0 {
            Err(AesError::FifoError)
        } else {
            Ok(())
        };
        if result.is_err() {
            // Stop the DMA before the area is reused; poisons on failure.
            self.soft_reset()?;
        }
        // SAFETY: the transfer finished or the DMA was reset; the area is
        // exclusively ours (see `new`).
        let dma = unsafe { &mut *self.dma };
        if result.is_ok() {
            output.copy_from_slice(&dma.output);
        }
        wipe(&mut dma.key);
        wipe(&mut dma.input);
        wipe(&mut dma.output);
        result
    }

    /// `cryptodma_reset`, bounded. A DMA that does not leave reset poisons
    /// the engine.
    fn soft_reset(&mut self) -> Result<(), AesError> {
        self.bus.write(reg::CONFIG, CONFIG_SOFTRESET);
        self.bus.write(reg::CONFIG, 0);
        let bus = &mut self.bus;
        poll(self.timeout, || {
            bus.read(reg::STATUS) & STATUS_SOFT_RESET == 0
        })
        .map_err(|_| {
            self.poisoned = true;
            AesError::Poisoned
        })
    }

    /// Build the descriptor lists; returns the bus addresses of the fetch
    /// and push list heads.
    fn fill(&mut self, key: &[u8; 16], input: &[u8; 16]) -> (u32, u32) {
        // SAFETY: no transfer is in flight (completed or reset).
        let dma = unsafe { &mut *self.dma };
        dma.config = MODE_ENCRYPT_ECB_128;
        dma.key = *key;
        dma.input = *input;
        dma.output = [0; 16];
        let bus = &self.bus;
        let addr = |p: *const u8| bus.bus_address(p);
        let fetch1 = addr((&raw const dma.fetch[1]).cast());
        let fetch2 = addr((&raw const dma.fetch[2]).cast());
        dma.fetch[0] = Descriptor {
            addr: addr((&raw const dma.config).cast()),
            next: fetch1,
            length_irq: 4 | DESCR_REALIGN,
            tag: TAG_BA411E | TAG_CONFIG,
        };
        dma.fetch[1] = Descriptor {
            addr: addr(dma.key.as_ptr()),
            next: fetch2,
            length_irq: 16 | DESCR_REALIGN,
            tag: TAG_BA411E | TAG_CONFIG | TAG_OFFSET_KEY,
        };
        dma.fetch[2] = Descriptor {
            addr: addr(dma.input.as_ptr()),
            next: DESCR_NEXT_STOP,
            length_irq: 16 | DESCR_REALIGN,
            tag: TAG_BA411E | TAG_LAST,
        };
        dma.push = Descriptor {
            addr: addr(dma.output.as_ptr()),
            next: DESCR_NEXT_STOP,
            length_irq: 16 | DESCR_REALIGN,
            tag: TAG_LAST,
        };
        (
            addr((&raw const dma.fetch[0]).cast()),
            addr((&raw const dma.push).cast()),
        )
    }
}

#[cfg(all(target_arch = "arm", target_os = "none"))]
mod hw {
    use crate::clock::{CryptoaccUser, cryptoacc_acquire, cryptoacc_release};

    /// `CRYPTOACC_S_BASE` (efr32mg22c224f512im40.h); the TRNG uses the same
    /// secure alias (`CRYPTOACC_S_RNGCTRL_BASE`).
    const CRYPTOACC_BASE: u32 = 0x4C02_0000;

    pub(crate) struct Mmio;

    impl super::CryptoaccBus for Mmio {
        fn read(&mut self, offset: u32) -> u32 {
            // SAFETY: CRYPTOMASTER/BA411E register, owned through the token.
            unsafe { core::ptr::read_volatile((CRYPTOACC_BASE + offset) as *const u32) }
        }
        fn write(&mut self, offset: u32, value: u32) {
            // SAFETY: CRYPTOMASTER register, owned through the token.
            unsafe { core::ptr::write_volatile((CRYPTOACC_BASE + offset) as *mut u32, value) }
        }
        fn bus_address(&self, ptr: *const u8) -> u32 {
            ptr as u32
        }
        fn acquire_clocks(&mut self) {
            cryptoacc_acquire(CryptoaccUser::Aes);
        }
        fn release_clocks(&mut self) {
            cryptoacc_release(CryptoaccUser::Aes);
        }
    }
}

/// DMA area of the single engine (see the parent module).
#[cfg(all(target_arch = "arm", target_os = "none"))]
static mut CRYPTOACC_DMA: CryptoaccDma = CryptoaccDma::ZERO;

/// The CRYPTOACC AES-128 engine, owned through the [`super::Crypto`] token.
#[cfg(all(target_arch = "arm", target_os = "none"))]
pub struct AesEngine {
    inner: Ba411e<hw::Mmio>,
    _token: super::Crypto,
}

#[cfg(all(target_arch = "arm", target_os = "none"))]
impl AesEngine {
    /// Conservative bound for one DMA wait.
    pub const DEFAULT_TIMEOUT_ITERATIONS: u32 = super::DEFAULT_TIMEOUT_ITERATIONS;

    /// Take ownership of the CRYPTOACC AES core and check that it supports
    /// AES-128 ECB. Run [`Self::self_test`] before the first use.
    pub fn new(token: super::Crypto, timeout_iterations: u32) -> Result<Self, AesError> {
        let timeout = super::validate_timeout_iterations(timeout_iterations)?;
        // SAFETY: the `Crypto` token is a singleton consumed here, so this is
        // the only user of `CRYPTOACC_DMA`, for the rest of the program.
        let mut inner = unsafe { Ba411e::new(hw::Mmio, &raw mut CRYPTOACC_DMA, timeout) };
        inner.probe()?;
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

    /// Run two known-answer vectors with different keys on the BA411E.
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

    #[derive(Clone, Copy, PartialEq, Eq, Debug)]
    enum Fault {
        None,
        /// The transfer never finishes.
        Hang,
        /// The transfer never finishes and the soft reset never completes.
        Dead,
        FetchError,
        FifoLeftover,
        Corrupt,
    }

    /// Behavioural CRYPTOMASTER + BA411E: on START it walks both descriptor
    /// lists through the fake bus addresses, checks the GSDK tags, encrypts
    /// with the reference AES and raises PUSHER_STOPPED.
    struct Model {
        map: AddressMap,
        fetch: u32,
        push: u32,
        config: u32,
        int_en: u32,
        raw: u32,
        status: u32,
        hw_cfg: u32,
        transfers: u32,
        clocks: i32,
        max_clocks: i32,
        fault: Fault,
    }

    impl Model {
        fn new(dma: *mut CryptoaccDma) -> Self {
            Self {
                map: AddressMap::new(dma),
                fetch: 0,
                push: 0,
                config: 0,
                int_en: 0,
                raw: 0,
                status: 0,
                hw_cfg: 0x0700_01FF,
                transfers: 0,
                clocks: 0,
                max_clocks: 0,
                fault: Fault::None,
            }
        }

        fn desc(&self, address: u32) -> Descriptor {
            // SAFETY: the address maps into the live DMA area.
            unsafe { *(self.map.to_host(address) as *const Descriptor) }
        }

        fn bytes<const N: usize>(&self, address: u32) -> [u8; N] {
            let mut out = [0u8; N];
            for (i, b) in out.iter_mut().enumerate() {
                // SAFETY: maps into the DMA area.
                *b = unsafe { *self.map.to_host(address + i as u32) };
            }
            out
        }

        fn start(&mut self) {
            assert!(self.clocks > 0, "transfer started with gated clocks");
            assert_eq!(
                self.config,
                CONFIG_FETCHER_INDIRECT | CONFIG_PUSHER_INDIRECT
            );
            assert_eq!(self.int_en, RAW_PUSHER_STOPPED);
            self.transfers += 1;
            match self.fault {
                Fault::Hang | Fault::Dead => {
                    self.status |= STATUS_FETCHER_BUSY | STATUS_PUSHER_BUSY;
                    return;
                }
                Fault::FetchError => {
                    self.raw |= RAW_FETCHER_ERROR;
                    return;
                }
                _ => {}
            }
            let cfg = self.desc(self.fetch);
            assert_eq!(cfg.tag, TAG_BA411E | TAG_CONFIG);
            assert_eq!(cfg.length_irq, 4 | DESCR_REALIGN);
            assert_eq!(
                u32::from_le_bytes(self.bytes::<4>(cfg.addr)),
                MODE_ENCRYPT_ECB_128
            );
            let key = self.desc(cfg.next);
            assert_eq!(key.tag, TAG_BA411E | TAG_CONFIG | TAG_OFFSET_KEY);
            assert_eq!(key.length_irq, 16 | DESCR_REALIGN);
            let data = self.desc(key.next);
            assert_eq!(data.tag, TAG_BA411E | TAG_LAST);
            assert_eq!(data.next, DESCR_NEXT_STOP);
            let out = self.desc(self.push);
            assert_eq!(out.tag, TAG_LAST);
            assert_eq!(out.next, DESCR_NEXT_STOP);
            assert_eq!(out.length_irq, 16 | DESCR_REALIGN);

            let mut result = reference_encrypt(&self.bytes(key.addr), &self.bytes(data.addr));
            if self.fault == Fault::Corrupt {
                result[15] ^= 0x80;
            }
            for (i, b) in result.iter().enumerate() {
                // SAFETY: maps into the DMA area.
                unsafe { *self.map.to_host(out.addr + i as u32) = *b };
            }
            if self.fault == Fault::FifoLeftover {
                self.status |= STATUS_FIFOIN_NOT_EMPTY;
            }
            self.raw |= RAW_PUSHER_STOPPED;
        }
    }

    impl CryptoaccBus for Model {
        fn read(&mut self, offset: u32) -> u32 {
            assert!(self.clocks > 0, "register read with gated clocks");
            match offset {
                reg::STATUS => self.status,
                reg::INT_STAT_RAW => self.raw,
                reg::BA411E_HW_CFG_1 => self.hw_cfg,
                _ => 0,
            }
        }
        fn write(&mut self, offset: u32, value: u32) {
            assert!(self.clocks > 0, "register write with gated clocks");
            match offset {
                reg::FETCH_ADDR => self.fetch = value,
                reg::PUSH_ADDR => self.push = value,
                reg::INT_EN => self.int_en = value,
                reg::INT_STAT_CLR => self.raw &= !value,
                reg::CONFIG => {
                    if value & CONFIG_SOFTRESET != 0 {
                        // A dead DMA never leaves soft reset.
                        self.status = if self.fault == Fault::Dead {
                            self.status | STATUS_SOFT_RESET
                        } else {
                            0
                        };
                    }
                    self.config = value;
                }
                reg::START => {
                    assert_eq!(value, START_FETCH_PUSH);
                    self.start();
                }
                _ => panic!("unexpected write {offset:#x}"),
            }
        }
        fn bus_address(&self, ptr: *const u8) -> u32 {
            self.map.to_bus(ptr)
        }
        fn acquire_clocks(&mut self) {
            self.clocks += 1;
            self.max_clocks = self.max_clocks.max(self.clocks);
        }
        fn release_clocks(&mut self) {
            self.clocks -= 1;
            assert!(self.clocks >= 0);
        }
    }

    fn engine(timeout: u32) -> Ba411e<Model> {
        let dma = Box::into_raw(Box::new(CryptoaccDma::ZERO));
        // SAFETY: leaked, exclusively used by this engine.
        unsafe { Ba411e::new(Model::new(dma), dma, timeout) }
    }

    fn dma_is_wiped(e: &Ba411e<Model>) -> bool {
        // SAFETY: no transfer in flight.
        let dma = unsafe { &*e.dma };
        dma.key == [0; 16] && dma.input == [0; 16] && dma.output == [0; 16]
    }

    #[test]
    fn probe_and_self_test_balance_the_clock_reference() {
        let mut e = engine(8);
        e.probe().unwrap();
        e.self_test().unwrap();
        assert_eq!(e.bus.transfers, 2);
        assert_eq!(
            e.bus.clocks, 0,
            "every request releases its clock reference"
        );
        assert_eq!(e.bus.max_clocks, 1);
        assert!(dma_is_wiped(&e));
    }

    #[test]
    fn probe_rejects_a_core_without_aes128_ecb() {
        let mut e = engine(8);
        e.bus_mut().hw_cfg = 0x0700_01FE; // no ECB
        assert_eq!(e.probe(), Err(AesError::Unsupported));
        e.bus_mut().hw_cfg = 0x0600_01FF; // no 128-bit keys
        assert_eq!(e.probe(), Err(AesError::Unsupported));
        assert_eq!(e.bus.clocks, 0);
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
    fn hung_transfer_times_out_resets_and_rechecks() {
        let mut e = engine(4);
        e.bus_mut().fault = Fault::Hang;
        let mut out = [0xA5; 16];
        assert_eq!(
            e.encrypt(&[1; 16], &[2; 16], &mut out),
            Err(AesError::Timeout)
        );
        assert_eq!(out, [0xA5; 16]);
        assert_eq!(e.bus.status, 0, "soft reset cleared the DMA");
        assert_eq!(e.bus.clocks, 0);
        assert!(dma_is_wiped(&e));
        e.bus_mut().fault = Fault::None;
        let before = e.bus.transfers;
        e.encrypt(&[1; 16], &[2; 16], &mut out).unwrap();
        assert_eq!(e.bus.transfers, before + 3, "KAT re-run before publishing");
        assert_eq!(out, reference_encrypt(&[1; 16], &[2; 16]));
    }

    #[test]
    fn dma_that_never_resets_poisons_the_engine() {
        let mut e = engine(4);
        e.bus_mut().fault = Fault::Dead;
        let mut out = [0x5A; 16];
        assert_eq!(
            e.encrypt(&[1; 16], &[2; 16], &mut out),
            Err(AesError::Poisoned)
        );
        e.bus_mut().fault = Fault::None;
        assert_eq!(
            e.encrypt(&[1; 16], &[2; 16], &mut out),
            Err(AesError::Poisoned)
        );
        assert_eq!(out, [0x5A; 16]);
        assert_eq!(e.bus.clocks, 0);
    }

    #[test]
    fn bus_error_fails_closed() {
        let mut e = engine(4);
        e.bus_mut().fault = Fault::FetchError;
        let mut out = [0x11; 16];
        assert_eq!(
            e.encrypt(&[1; 16], &[2; 16], &mut out),
            Err(AesError::DmaError)
        );
        assert_eq!(out, [0x11; 16]);
        assert!(dma_is_wiped(&e));
    }

    #[test]
    fn fifo_leftover_fails_closed() {
        let mut e = engine(4);
        e.bus_mut().fault = Fault::FifoLeftover;
        let mut out = [0x22; 16];
        assert_eq!(
            e.encrypt(&[1; 16], &[2; 16], &mut out),
            Err(AesError::FifoError)
        );
        assert_eq!(out, [0x22; 16]);
    }

    #[test]
    fn zero_poll_budget_is_rejected() {
        assert_eq!(
            super::super::validate_timeout_iterations(0),
            Err(AesError::ZeroIterations)
        );
    }

    #[test]
    fn wrong_engine_output_fails_the_kat() {
        let mut e = engine(4);
        e.bus_mut().fault = Fault::Corrupt;
        assert_eq!(e.self_test(), Err(AesError::KnownAnswerMismatch));
        let mut out = [0x44; 16];
        assert_eq!(
            e.encrypt(&[1; 16], &[2; 16], &mut out),
            Err(AesError::KnownAnswerMismatch)
        );
        assert_eq!(out, [0x44; 16]);
    }
}
