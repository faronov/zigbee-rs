//! CC2340R5 `LAES` hardware AES-128 behind [`zigbee_crypto::ForwardAesProvider`].
//!
//! The composition root moves the `cc2340r5_hal::aes::Aes` token into
//! [`super::Cc2340RadioPhy::install_aes_engine`], which builds the HAL
//! [`AesEngine`] and runs its known-answer self-test before accepting it.
//! From then on every NWK/APS CCM* block, AES-MMO compression, install-code
//! derivation and Trust Center key-verification hash of this image runs on
//! the `LAES` core. There is no software fall-back: an image without an
//! installed engine fails every cipher operation with
//! [`Cc2340AesError::NotInstalled`], and an engine error surfaces as
//! [`Cc2340AesError::Engine`] (the stack drops the frame).
//!
//! Ownership: the radio (LRFD/PBE, driven by this crate's `driver`) never
//! uses `LAES`; nothing else in the image can reach it because the HAL token
//! is consumed here. The entropy source (noise conditioning, CTR_DRBG and its
//! known-answer test) borrows the same engine from the PHY through
//! [`LaesProvider`] for the duration of one request; it never holds it.

#[cfg(target_os = "none")]
pub use cc2340r5_hal::aes::AesEngine;
pub use cc2340r5_hal::aes::{Aes as AesPeripheral, AesError};
use zigbee_crypto::{Aes128Forward, AesKey, ForwardAesProvider};

/// Host builds have no `LAES` core, so no engine can exist there.
#[cfg(not(target_os = "none"))]
pub enum AesEngine {}

#[cfg(not(target_os = "none"))]
impl AesEngine {
    fn encrypt_block(&mut self, _key: &AesKey, _block: &mut [u8; 16]) -> Result<(), AesError> {
        match *self {}
    }
}

/// Failure of the CC2340 forward AES backend.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Cc2340AesError {
    /// The composition root has not installed a validated `LAES` engine.
    NotInstalled,
    /// The `LAES` engine reported an error (no output was published).
    Engine(AesError),
}

impl From<AesError> for Cc2340AesError {
    fn from(error: AesError) -> Self {
        Self::Engine(error)
    }
}

/// Keyed forward AES-128 permutation on the `LAES` core.
pub struct Cc2340HardwareAes128<'engine> {
    engine: Option<&'engine mut AesEngine>,
    key: AesKey,
}

impl<'engine> Cc2340HardwareAes128<'engine> {
    pub(super) fn new(engine: Option<&'engine mut AesEngine>, key: &AesKey) -> Self {
        Self { engine, key: *key }
    }
}

impl Aes128Forward for Cc2340HardwareAes128<'_> {
    type Error = Cc2340AesError;

    #[inline]
    fn encrypt_block(&mut self, block: &mut [u8; 16]) -> Result<(), Self::Error> {
        match self.engine.as_mut() {
            Some(engine) => engine.encrypt_block(&self.key, block).map_err(Into::into),
            None => Err(Cc2340AesError::NotInstalled),
        }
    }
}

/// [`ForwardAesProvider`] over a borrowed engine, so the PHY's entropy
/// state can use `LAES` while the PHY keeps sole ownership of it.
pub(super) struct LaesProvider<'engine> {
    engine: Option<&'engine mut AesEngine>,
}

impl<'engine> LaesProvider<'engine> {
    pub(super) fn new(engine: Option<&'engine mut AesEngine>) -> Self {
        Self { engine }
    }
}

impl ForwardAesProvider for LaesProvider<'_> {
    fn forward_cipher(&mut self, key: &AesKey) -> impl Aes128Forward + '_ {
        Cc2340HardwareAes128::new(self.engine.as_deref_mut(), key)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn missing_engine_fails_closed_without_output() {
        let mut cipher = Cc2340HardwareAes128::new(None, &[0x11; 16]);
        let mut block = [0x5A; 16];
        assert_eq!(
            cipher.encrypt_block(&mut block),
            Err(Cc2340AesError::NotInstalled)
        );
        assert_eq!(block, [0x5A; 16]);
    }

    #[test]
    fn engine_errors_are_preserved() {
        assert_eq!(
            Cc2340AesError::from(AesError::Timeout),
            Cc2340AesError::Engine(AesError::Timeout)
        );
    }
}
