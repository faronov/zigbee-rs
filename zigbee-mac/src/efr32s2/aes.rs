//! Hardware AES-128 for the EFR32 Series 2 PHY (NWK/APS CCM*, AES-MMO,
//! Green Power).
//!
//! The engine is `efr32s2_hal::crypto::AesEngine`: the Secure Engine mailbox
//! on xG21 and the CRYPTOACC BA411E on xG22. The composition root hands the
//! chip's `Crypto` token to [`Efr32s2RadioPhy::install_aes_engine`], which
//! runs the on-silicon known-answer test before it accepts the engine. There
//! is no software AES on this platform: until an engine is installed (and
//! on host builds, which have no engine) every cipher request fails with
//! [`Efr32s2AesError::NotInstalled`], which the stack turns into a dropped
//! frame.

use zigbee_crypto::{Aes128Forward, AesKey, ForwardAesProvider};

use super::Efr32s2RadioPhy;

/// Failure of the EFR32 Series 2 AES backend.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Efr32s2AesError {
    /// No engine was installed with [`Efr32s2RadioPhy::install_aes_engine`].
    NotInstalled,
    /// The hardware engine reported an error.
    Engine(efr32s2_hal::crypto::AesError),
}

/// Keyed AES-128 forward permutation on the installed hardware engine.
pub struct Efr32s2Aes128<'a> {
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    engine: Option<&'a mut efr32s2_hal::crypto::AesEngine>,
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    engine: Option<&'a mut core::convert::Infallible>,
    key: AesKey,
}

impl Aes128Forward for Efr32s2Aes128<'_> {
    type Error = Efr32s2AesError;

    #[inline]
    #[cfg_attr(
        not(all(target_arch = "arm", target_os = "none")),
        allow(unused_variables)
    )]
    fn encrypt_block(&mut self, block: &mut [u8; 16]) -> Result<(), Self::Error> {
        match self.engine.as_deref_mut() {
            #[cfg(all(target_arch = "arm", target_os = "none"))]
            Some(engine) => engine
                .encrypt_block(&self.key, block)
                .map_err(Efr32s2AesError::Engine),
            #[cfg(not(all(target_arch = "arm", target_os = "none")))]
            Some(never) => match *never {},
            None => Err(Efr32s2AesError::NotInstalled),
        }
    }
}

impl Drop for Efr32s2Aes128<'_> {
    fn drop(&mut self) {
        zigbee_crypto::zeroize_key(&mut self.key);
    }
}

impl<const D: i8> ForwardAesProvider for Efr32s2RadioPhy<D> {
    // Concrete return type, so wrappers (e.g. a lab PHY) can delegate.
    #[allow(refining_impl_trait)]
    fn forward_cipher(&mut self, key: &AesKey) -> Efr32s2Aes128<'_> {
        Efr32s2Aes128 {
            #[cfg(all(target_arch = "arm", target_os = "none"))]
            engine: self.aes.as_mut(),
            #[cfg(not(all(target_arch = "arm", target_os = "none")))]
            engine: None,
            key: *key,
        }
    }
}

#[cfg(all(target_arch = "arm", target_os = "none"))]
impl<const D: i8> Efr32s2RadioPhy<D> {
    /// Consume the chip's `Crypto` token, run the engine's known-answer test
    /// on silicon and install it as this PHY's AES provider. On error the
    /// engine is not installed and every cipher request keeps failing; the
    /// composition root must not start Zigbee.
    pub fn install_aes_engine(
        &mut self,
        token: efr32s2_hal::crypto::Crypto,
    ) -> Result<(), efr32s2_hal::crypto::AesError> {
        let mut engine = efr32s2_hal::crypto::AesEngine::new(
            token,
            efr32s2_hal::crypto::AesEngine::DEFAULT_TIMEOUT_ITERATIONS,
        )?;
        engine.self_test()?;
        self.aes = Some(engine);
        Ok(())
    }

    /// `true` once [`Self::install_aes_engine`] succeeded.
    pub fn aes_engine_installed(&self) -> bool {
        self.aes.is_some()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn missing_engine_fails_closed() {
        let mut cipher = Efr32s2Aes128 {
            engine: None,
            key: [7; 16],
        };
        let mut block = [0x5A; 16];
        assert_eq!(
            cipher.encrypt_block(&mut block),
            Err(Efr32s2AesError::NotInstalled)
        );
        assert_eq!(block, [0x5A; 16]);
    }
}
