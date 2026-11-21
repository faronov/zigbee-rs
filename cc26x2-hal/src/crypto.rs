//! AES-128 ECB single-block encryption on the CRYPTO engine
//! (`inc/hw_crypto.h`, driverlib `aes.c`, TI `AESECBCC26XX.c`).
//!
//! The key goes through the key store (area 6, as TI's driver): written by
//! the engine's DMA channel 0 with `ALGSEL = KEY_STORE`, then loaded into the
//! AES core with `KEYREADAREA`. A block is encrypted by DMA channel 0
//! (input) and channel 1 (output) with `AESCTL.DIR` (encrypt, ECB) and
//! `DATALEN = 16`, polled on `IRQSTAT.RESULT_AVAIL`; the CRYPTO interrupt
//! stays masked in the NVIC. The last key stays in the key store, so CCM*
//! (one key, many blocks) writes it once.
//!
//! Every wait is bounded. A timeout or bus/key-store error resets the
//! engine (`SWRESET`, DMA `SWRES`), forgets the stored key and is
//! returned; if the DMA does not become idle after the reset the engine is
//! poisoned and refuses further work.
//!
//! The register sequence runs against [`CryptoBus`], so host tests drive it
//! through a behavioural model of the engine.

use core::sync::atomic::{Ordering, compiler_fence};

// Register offsets from CRYPTO_BASE.
const DMACH0CTL: u32 = 0x000;
const DMACH0EXTADDR: u32 = 0x004;
const DMACH0LEN: u32 = 0x00C;
const DMASTAT: u32 = 0x018;
const DMASWRESET: u32 = 0x01C;
const DMACH1CTL: u32 = 0x020;
const DMACH1EXTADDR: u32 = 0x024;
const DMACH1LEN: u32 = 0x02C;
const KEYWRITEAREA: u32 = 0x400;
const KEYWRITTENAREA: u32 = 0x404;
const KEYSIZE: u32 = 0x408;
const KEYREADAREA: u32 = 0x40C;
const AESCTL: u32 = 0x550;
const AESDATALEN0: u32 = 0x554;
const AESDATALEN1: u32 = 0x558;
const ALGSEL: u32 = 0x700;
const SWRESET: u32 = 0x740;
const IRQTYPE: u32 = 0x780;
const IRQEN: u32 = 0x784;
const IRQCLR: u32 = 0x788;
const IRQSTAT: u32 = 0x790;

const DMA_EN: u32 = 1;
const DMASTAT_ACTIVE: u32 = 0x3;
const KEYSIZE_M: u32 = 0x3;
const KEYSIZE_128: u32 = 1;
const KEYREADAREA_BUSY: u32 = 1 << 31;
const AESCTL_DIR: u32 = 1 << 2;
const ALGSEL_KEY_STORE: u32 = 1;
const ALGSEL_AES: u32 = 2;
const IRQTYPE_LEVEL: u32 = 1;
const IRQ_RESULT_AVAIL: u32 = 1 << 0;
const IRQ_DMA_IN_DONE: u32 = 1 << 1;
const IRQ_KEY_ST_RD_ERR: u32 = 1 << 29;
const IRQ_KEY_ST_WR_ERR: u32 = 1 << 30;
const IRQ_DMA_BUS_ERR: u32 = 1 << 31;

/// Key-store area used for the plaintext key (TI's convention).
const KEY_AREA: u32 = 6;
/// Polls of a hardware condition before it counts as a timeout (a key
/// write and a block each take well under 1 000 CPU cycles).
const WAIT_LIMIT: u32 = 100_000;

/// Register and DMA access of the engine.
pub trait CryptoBus {
    fn read(&mut self, offset: u32) -> u32;
    fn write(&mut self, offset: u32, value: u32);
    /// The 32-bit bus address of `ptr` (the engine's DMA addresses SRAM).
    fn bus_address(&self, ptr: *const u8) -> u32;
}

/// Failure of the fail-closed engine.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AesError {
    /// The PERIPH power domain did not come up.
    Power,
    /// A bounded wait expired (the engine was reset).
    Timeout,
    /// DMA bus error (the engine was reset).
    Bus,
    /// Key-store write or read error (the engine was reset).
    KeyStore,
    /// A reset did not bring the DMA back to idle: the engine is unusable.
    Poisoned,
}

/// Ownership token of the CRYPTO engine.
pub struct CryptoToken(pub(crate) ());

/// DMA buffers; word-aligned SRAM owned by the engine.
#[repr(C, align(4))]
struct Buffers {
    key: [u8; 16],
    input: [u8; 16],
    output: [u8; 16],
}

/// The AES-128 engine.
pub struct AesEngine<B: CryptoBus> {
    bus: B,
    buf: Buffers,
    stored_key: Option<[u8; 16]>,
    poisoned: bool,
}

/// Memory-mapped CRYPTO registers.
pub struct Mmio(());

impl CryptoBus for Mmio {
    fn read(&mut self, offset: u32) -> u32 {
        crate::mmio::read(crate::mmio::CRYPTO_BASE + offset)
    }

    fn write(&mut self, offset: u32, value: u32) {
        crate::mmio::write(crate::mmio::CRYPTO_BASE + offset, value)
    }

    fn bus_address(&self, ptr: *const u8) -> u32 {
        ptr as usize as u32
    }
}

impl AesEngine<Mmio> {
    /// Power and clock the engine (PERIPH domain, run and sleep gates).
    pub fn new(_token: CryptoToken) -> Result<Self, AesError> {
        use crate::prcm::{self, Domain, Gate};
        if !prcm::is_on(Domain::Periph) && !prcm::power_on(Domain::Periph) {
            return Err(AesError::Power);
        }
        prcm::enable(Gate::Crypto);
        prcm::load();
        let mut engine = Self::with_bus(Mmio(()));
        engine.reset()?;
        Ok(engine)
    }
}

impl<B: CryptoBus> AesEngine<B> {
    /// An engine on `bus` (host tests: a model).
    pub const fn with_bus(bus: B) -> Self {
        Self {
            bus,
            buf: Buffers {
                key: [0; 16],
                input: [0; 16],
                output: [0; 16],
            },
            stored_key: None,
            poisoned: false,
        }
    }

    pub fn bus(&self) -> &B {
        &self.bus
    }

    /// Encrypt `block` in place with `key` (AES-128, ECB, one block).
    pub fn encrypt_block(&mut self, key: &[u8; 16], block: &mut [u8; 16]) -> Result<(), AesError> {
        if self.poisoned {
            return Err(AesError::Poisoned);
        }
        let result = self.run(key, block);
        if let Err(error) = result {
            self.stored_key = None;
            self.reset()?;
            return Err(error);
        }
        result
    }

    fn run(&mut self, key: &[u8; 16], block: &mut [u8; 16]) -> Result<(), AesError> {
        if self.stored_key.as_ref() != Some(key) {
            self.store_key(key)?;
            self.stored_key = Some(*key);
        }
        // AESReadFromKeyStore: key store area -> AES core.
        self.bus.write(IRQEN, IRQ_RESULT_AVAIL);
        self.bus.write(ALGSEL, ALGSEL_AES);
        self.bus.write(KEYREADAREA, KEY_AREA);
        self.wait(|bus| bus.read(KEYREADAREA) & KEYREADAREA_BUSY == 0)?;
        if self.bus.read(IRQSTAT) & IRQ_KEY_ST_RD_ERR != 0 {
            return Err(AesError::KeyStore);
        }
        // One ECB block, encrypt.
        self.bus.write(AESCTL, AESCTL_DIR);
        self.bus.write(AESDATALEN0, 16);
        self.bus.write(AESDATALEN1, 0);
        self.buf.input = *block;
        let input = self.bus.bus_address(self.buf.input.as_ptr());
        let output = self.bus.bus_address(self.buf.output.as_ptr());
        self.start_dma(input, Some(output));
        let irq = self.wait_irq(IRQ_RESULT_AVAIL | IRQ_DMA_BUS_ERR)?;
        self.bus.write(ALGSEL, 0);
        if irq & IRQ_DMA_BUS_ERR != 0 {
            return Err(AesError::Bus);
        }
        compiler_fence(Ordering::SeqCst);
        *block = self.buf.output;
        Ok(())
    }

    /// AESWriteToKeyStore(key, 16, area 6).
    fn store_key(&mut self, key: &[u8; 16]) -> Result<(), AesError> {
        self.bus.write(KEYWRITTENAREA, 1 << KEY_AREA);
        self.bus.write(IRQTYPE, IRQTYPE_LEVEL);
        self.bus.write(IRQEN, IRQ_DMA_IN_DONE | IRQ_RESULT_AVAIL);
        self.bus.write(ALGSEL, ALGSEL_KEY_STORE);
        self.bus.write(IRQCLR, IRQ_DMA_IN_DONE | IRQ_RESULT_AVAIL);
        if self.bus.read(KEYSIZE) & KEYSIZE_M != KEYSIZE_128 {
            // Changing the size invalidates every stored key.
            self.bus.write(KEYSIZE, KEYSIZE_128);
        }
        self.bus.write(KEYWRITEAREA, 1 << KEY_AREA);
        self.buf.key = *key;
        let source = self.bus.bus_address(self.buf.key.as_ptr());
        self.start_dma(source, None);
        let irq = self
            .wait_irq(IRQ_DMA_IN_DONE | IRQ_RESULT_AVAIL | IRQ_DMA_BUS_ERR | IRQ_KEY_ST_WR_ERR)?;
        // The key bytes leave the DMA buffer once stored.
        self.buf.key = [0; 16];
        if irq & IRQ_DMA_BUS_ERR != 0 {
            return Err(AesError::Bus);
        }
        if irq & IRQ_KEY_ST_WR_ERR != 0 || self.bus.read(KEYWRITTENAREA) & (1 << KEY_AREA) == 0 {
            return Err(AesError::KeyStore);
        }
        Ok(())
    }

    /// AESStartDMAOperation(source, 16, destination, 16).
    fn start_dma(&mut self, source: u32, destination: Option<u32>) {
        compiler_fence(Ordering::SeqCst);
        self.bus.write(IRQCLR, IRQ_RESULT_AVAIL | IRQ_DMA_IN_DONE);
        self.bus.write(DMACH0CTL, DMA_EN);
        self.bus.write(DMACH0EXTADDR, source);
        if let Some(destination) = destination {
            self.bus.write(DMACH1CTL, DMA_EN);
            self.bus.write(DMACH1EXTADDR, destination);
            self.bus.write(DMACH1LEN, 16);
        }
        // Writing the channel 0 length starts the transfer.
        self.bus.write(DMACH0LEN, 16);
    }

    /// Wait for any of `flags` in `IRQSTAT`, then clear and return them.
    fn wait_irq(&mut self, flags: u32) -> Result<u32, AesError> {
        self.wait(|bus| bus.read(IRQSTAT) & flags != 0)?;
        let irq = self.bus.read(IRQSTAT) & flags;
        self.bus.write(IRQCLR, irq);
        Ok(irq)
    }

    fn wait(&mut self, mut done: impl FnMut(&mut B) -> bool) -> Result<(), AesError> {
        for _ in 0..WAIT_LIMIT {
            if done(&mut self.bus) {
                return Ok(());
            }
            core::hint::spin_loop();
        }
        if done(&mut self.bus) {
            Ok(())
        } else {
            Err(AesError::Timeout)
        }
    }

    /// AESReset + AESDMAReset; poison the engine if the DMA stays busy.
    fn reset(&mut self) -> Result<(), AesError> {
        self.bus.write(SWRESET, 1);
        self.bus.write(AESCTL, 0);
        self.bus.write(AESDATALEN0, 0);
        self.bus.write(AESDATALEN1, 0);
        self.bus.write(DMASWRESET, 1);
        if self
            .wait(|bus| bus.read(DMASTAT) & DMASTAT_ACTIVE == 0)
            .is_err()
        {
            self.poisoned = true;
            return Err(AesError::Poisoned);
        }
        self.bus.write(KEYWRITTENAREA, 1 << KEY_AREA);
        self.bus.write(ALGSEL, 0);
        self.stored_key = None;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use aes::Aes128;
    use aes::cipher::{BlockEncrypt, KeyInit, generic_array::GenericArray};
    use std::collections::HashMap;
    use std::vec::Vec;

    /// Behavioural model of the CRYPTO engine: the registers the driver
    /// uses, a key store, and DMA that completes instantly.
    #[derive(Default)]
    struct Model {
        regs: HashMap<u32, u32>,
        key_store: [Option<[u8; 16]>; 8],
        aes_key: Option<[u8; 16]>,
        key_writes: usize,
        blocks: usize,
        /// Fault injection.
        stall_result: bool,
        bus_error: bool,
        dma_stuck: bool,
    }

    impl Model {
        fn reg(&self, offset: u32) -> u32 {
            *self.regs.get(&offset).unwrap_or(&0)
        }

        fn set_irq(&mut self, bits: u32) {
            let stat = self.reg(IRQSTAT) | bits;
            self.regs.insert(IRQSTAT, stat);
        }

        fn written_area(&self) -> u32 {
            (0..8)
                .filter(|&i| self.key_store[i].is_some())
                .fold(0, |m, i| m | (1 << i))
        }

        fn dma(&mut self) {
            let src = self.reg(DMACH0EXTADDR) as usize;
            // Host tests run on 64-bit: addresses are indices into a
            // registry of the driver's buffers (see `bus_address`).
            let data = unsafe { *(BUFFERS.with(|b| b.borrow()[src]) as *const [u8; 16]) };
            if self.bus_error {
                self.set_irq(IRQ_DMA_BUS_ERR);
                return;
            }
            match self.reg(ALGSEL) {
                ALGSEL_KEY_STORE => {
                    let area = self.reg(KEYWRITEAREA).trailing_zeros() as usize;
                    if self.reg(KEYSIZE) & KEYSIZE_M == KEYSIZE_128 && area < 8 {
                        self.key_store[area] = Some(data);
                        self.key_writes += 1;
                        self.set_irq(IRQ_DMA_IN_DONE | IRQ_RESULT_AVAIL);
                    } else {
                        self.set_irq(IRQ_KEY_ST_WR_ERR);
                    }
                }
                ALGSEL_AES => {
                    if self.stall_result {
                        return;
                    }
                    let key = self.aes_key.expect("AES core has a key");
                    assert_eq!(self.reg(AESCTL), AESCTL_DIR, "ECB encrypt");
                    assert_eq!(self.reg(AESDATALEN0), 16);
                    let mut block = GenericArray::clone_from_slice(&data);
                    Aes128::new(GenericArray::from_slice(&key)).encrypt_block(&mut block);
                    let dst = self.reg(DMACH1EXTADDR) as usize;
                    let out = BUFFERS.with(|b| b.borrow()[dst]) as *mut [u8; 16];
                    unsafe { (*out).copy_from_slice(&block) };
                    self.blocks += 1;
                    self.set_irq(IRQ_DMA_IN_DONE | IRQ_RESULT_AVAIL);
                }
                other => panic!("DMA with ALGSEL {other}"),
            }
        }
    }

    std::thread_local! {
        static BUFFERS: core::cell::RefCell<Vec<usize>> = const { core::cell::RefCell::new(Vec::new()) };
    }

    impl CryptoBus for Model {
        fn read(&mut self, offset: u32) -> u32 {
            match offset {
                KEYWRITTENAREA => self.written_area(),
                DMASTAT => {
                    if self.dma_stuck {
                        1
                    } else {
                        0
                    }
                }
                _ => self.reg(offset),
            }
        }

        fn write(&mut self, offset: u32, value: u32) {
            match offset {
                KEYWRITTENAREA => {
                    for i in 0..8 {
                        if value & (1 << i) != 0 {
                            self.key_store[i] = None;
                        }
                    }
                }
                KEYSIZE => {
                    self.key_store = [None; 8];
                    self.regs.insert(KEYSIZE, value);
                }
                KEYREADAREA => {
                    match self.key_store[value as usize] {
                        Some(key) => self.aes_key = Some(key),
                        None => self.set_irq(IRQ_KEY_ST_RD_ERR),
                    }
                    self.regs.insert(KEYREADAREA, value);
                }
                IRQCLR => {
                    let stat = self.reg(IRQSTAT) & !value;
                    self.regs.insert(IRQSTAT, stat);
                }
                DMACH0LEN => {
                    self.regs.insert(offset, value);
                    self.dma();
                }
                SWRESET => self.aes_key = None,
                _ => {
                    self.regs.insert(offset, value);
                }
            }
        }

        fn bus_address(&self, ptr: *const u8) -> u32 {
            BUFFERS.with(|b| {
                let mut b = b.borrow_mut();
                b.push(ptr as usize);
                (b.len() - 1) as u32
            })
        }
    }

    fn software(key: &[u8; 16], block: &[u8; 16]) -> [u8; 16] {
        let mut b = GenericArray::clone_from_slice(block);
        Aes128::new(GenericArray::from_slice(key)).encrypt_block(&mut b);
        b.into()
    }

    #[test]
    fn fips197_vector_and_key_reuse() {
        let mut engine = AesEngine::with_bus(Model::default());
        let key = *b"\x00\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0a\x0b\x0c\x0d\x0e\x0f";
        let mut block = *b"\x00\x11\x22\x33\x44\x55\x66\x77\x88\x99\xaa\xbb\xcc\xdd\xee\xff";
        engine.encrypt_block(&key, &mut block).unwrap();
        assert_eq!(
            block,
            *b"\x69\xc4\xe0\xd8\x6a\x7b\x04\x30\xd8\xcd\xb7\x80\x70\xb4\xc5\x5a"
        );
        // Same key: no second key-store write.
        let mut next = [0x5A; 16];
        engine.encrypt_block(&key, &mut next).unwrap();
        assert_eq!(next, software(&key, &[0x5A; 16]));
        assert_eq!((engine.bus().key_writes, engine.bus().blocks), (1, 2));
        // A new key is stored once.
        let other = [0xA5; 16];
        let mut b = [1; 16];
        engine.encrypt_block(&other, &mut b).unwrap();
        assert_eq!(b, software(&other, &[1; 16]));
        assert_eq!(engine.bus().key_writes, 2);
        // The plaintext key does not stay in the DMA buffer.
        assert_eq!(engine.buf.key, [0; 16]);
    }

    #[test]
    fn faults_are_reported_and_the_engine_recovers() {
        let mut engine = AesEngine::with_bus(Model::default());
        let key = [7; 16];
        engine.bus.stall_result = true;
        let mut block = [0; 16];
        assert_eq!(
            engine.encrypt_block(&key, &mut block),
            Err(AesError::Timeout)
        );
        engine.bus.stall_result = false;
        engine.encrypt_block(&key, &mut block).unwrap();
        assert_eq!(block, software(&key, &[0; 16]));

        engine.bus.bus_error = true;
        assert_eq!(
            engine.encrypt_block(&[8; 16], &mut block),
            Err(AesError::Bus)
        );
        engine.bus.bus_error = false;
        let mut block = [3; 16];
        engine.encrypt_block(&[8; 16], &mut block).unwrap();
        assert_eq!(block, software(&[8; 16], &[3; 16]));
    }

    #[test]
    fn a_dma_that_never_idles_poisons_the_engine() {
        let mut engine = AesEngine::with_bus(Model::default());
        engine.bus.stall_result = true;
        engine.bus.dma_stuck = true;
        let mut block = [0; 16];
        assert_eq!(
            engine.encrypt_block(&[1; 16], &mut block),
            Err(AesError::Poisoned)
        );
        engine.bus.stall_result = false;
        engine.bus.dma_stuck = false;
        assert_eq!(
            engine.encrypt_block(&[1; 16], &mut block),
            Err(AesError::Poisoned)
        );
    }

    /// The engine as a CCM* block cipher.
    struct EngineCipher<'a> {
        engine: &'a mut AesEngine<Model>,
        key: [u8; 16],
    }

    impl zigbee_crypto::Aes128Forward for EngineCipher<'_> {
        type Error = AesError;
        fn encrypt_block(&mut self, block: &mut [u8; 16]) -> Result<(), AesError> {
            self.engine.encrypt_block(&self.key, block)
        }
    }

    /// Zigbee CCM* (security level 5, 4-byte MIC) golden vectors of
    /// `zigbee-crypto` (an NWK frame and a captured APS frame, both checked
    /// against an independent CCM implementation) through the engine.
    #[test]
    fn zigbee_ccm_star_vectors_through_the_engine() {
        let mut engine = AesEngine::with_bus(Model::default());
        let nwk_key = [
            0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x0F, 0x00, 0x02, 0x04, 0x06, 0x08, 0x0A,
            0x0C, 0x0D,
        ];
        let nwk_nonce = [
            0xA1, 0xB2, 0xC3, 0xD4, 0xE5, 0xF6, 0x07, 0x08, 0x01, 0x00, 0x00, 0x00, 0x2D,
        ];
        let mut cipher = EngineCipher {
            engine: &mut engine,
            key: nwk_key,
        };
        let sealed = zigbee_crypto::ccm_star_encrypt_with(
            &mut cipher,
            &nwk_nonce,
            b"NWK-HDR+AUX",
            b"hello-nwk-frame",
        )
        .unwrap()
        .unwrap();
        assert_eq!(
            sealed.as_slice(),
            [
                0xAC, 0x17, 0x74, 0xEC, 0x17, 0x76, 0xC2, 0x7C, 0x41, 0xEE, 0x31, 0x0A, 0xE0, 0x0B,
                0x5B, 0x5A, 0xA0, 0x05, 0xC9,
            ]
        );
        let opened =
            zigbee_crypto::ccm_star_decrypt_with(&mut cipher, &nwk_nonce, b"NWK-HDR+AUX", &sealed)
                .unwrap()
                .unwrap();
        assert_eq!(opened.as_slice(), b"hello-nwk-frame");

        let aps_key = [
            0x4B, 0xAB, 0x0F, 0x17, 0x3E, 0x14, 0x34, 0xA2, 0xD5, 0x72, 0xE1, 0xC1, 0xEF, 0x47,
            0x87, 0x82,
        ];
        let aps_nonce = [
            0xF2, 0xA6, 0xC9, 0xFE, 0xFF, 0x27, 0x71, 0x84, 0x53, 0x50, 0x0B, 0x00, 0x35,
        ];
        let aad = [
            0x21, 0x95, 0x35, 0x53, 0x50, 0x0B, 0x00, 0xF2, 0xA6, 0xC9, 0xFE, 0xFF, 0x27, 0x71,
            0x84,
        ];
        let plaintext = [
            0x05, 0x01, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55,
            0x66, 0x77, 0x88, 0x99, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
            0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10,
        ];
        let mut cipher = EngineCipher {
            engine: &mut engine,
            key: aps_key,
        };
        let sealed =
            zigbee_crypto::ccm_star_encrypt_with(&mut cipher, &aps_nonce, &aad, &plaintext)
                .unwrap()
                .unwrap();
        assert_eq!(
            sealed.as_slice(),
            [
                0xB6, 0x78, 0x43, 0xD6, 0x39, 0xC1, 0x70, 0xF2, 0x0B, 0x53, 0x6F, 0xDA, 0x11, 0xB4,
                0x67, 0xCA, 0xAF, 0xEC, 0xD6, 0xC2, 0x4C, 0xD8, 0x2D, 0xFB, 0xD1, 0xD8, 0x33, 0x8A,
                0x1D, 0xBD, 0x15, 0xEE, 0x18, 0x92, 0xAE, 0x51, 0xAB, 0x41, 0xEB,
            ]
        );
        // Tampering is rejected through the engine as well.
        let mut tampered = sealed.clone();
        tampered[0] ^= 1;
        assert_eq!(
            zigbee_crypto::ccm_star_decrypt_with(&mut cipher, &aps_nonce, &aad, &tampered),
            Ok(None)
        );
        // Two keys, one key-store write each per key change.
        assert_eq!(engine.bus().key_writes, 2);
    }
}
