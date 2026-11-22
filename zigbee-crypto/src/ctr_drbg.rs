//! NIST SP 800-90A Rev. 1 `CTR_DRBG` with AES-128, no derivation function.
//!
//! `seedlen = keylen + blocklen = 256` bits and `ctr_len = blocklen`. The
//! entropy input must therefore be a 256-bit *full-entropy* string
//! (SP 800-90A 10.2.1.3.1): the caller conditions its noise source first
//! (e.g. with the SP 800-90B CBC-MAC vetted conditioning component). There is
//! no nonce in the no-df construction; an optional personalization string of
//! up to 256 bits is XORed into the seed material.
//!
//! The block cipher comes from a [`ForwardAesProvider`], so a platform AES
//! engine serves the DRBG through the same owner as CCM*: the DRBG keeps only
//! its working state (`Key`, `V`, `reseed_counter`) and borrows the provider
//! for each operation. A cipher error is never hidden: the operation fails,
//! the working state is wiped (the DRBG becomes uninstantiated) and any
//! partially generated output is zeroised before the error is returned.
//!
//! Prediction resistance is not provided by this type: the caller decides
//! when to [`reseed`](CtrDrbg::reseed) with fresh entropy. Backtracking
//! resistance is inherent: every generate request ends with the `Update`
//! step.

use crate::{Aes128Forward, AesKey, ForwardAesProvider, zeroize};

/// AES block length in bytes.
pub const BLOCK_LEN: usize = 16;
/// `seedlen` in bytes (entropy input and personalization string length).
pub const SEED_LEN: usize = 32;
/// SP 800-90A Table 3: at most 2^19 bits per generate request.
pub const MAX_REQUEST_BYTES: usize = 1 << 16;
/// SP 800-90A Table 3 bound on `reseed_interval` (2^48 requests).
pub const MAX_RESEED_INTERVAL: u64 = 1 << 48;

/// Failure of a DRBG operation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DrbgError {
    /// The block cipher failed; the DRBG was uninstantiated.
    Cipher,
    /// `reseed_counter` exceeded `reseed_interval`; reseed before generating.
    ReseedRequired,
    /// More than [`MAX_REQUEST_BYTES`] requested in one generate call.
    RequestTooLarge,
    /// A `reseed_interval` of zero or above [`MAX_RESEED_INTERVAL`].
    InvalidReseedInterval,
}

/// Working state of an instantiated AES-128 `CTR_DRBG` (no df).
///
/// Dropping it wipes `Key` and `V`.
pub struct CtrDrbg {
    key: AesKey,
    v: [u8; BLOCK_LEN],
    reseed_counter: u64,
    reseed_interval: u64,
}

impl CtrDrbg {
    /// `CTR_DRBG_Instantiate_algorithm` (10.2.1.3.1): `Key = 0`, `V = 0`,
    /// then `Update(entropy_input XOR personalization_string)`.
    ///
    /// `entropy_input` must carry full entropy; it is not retained.
    pub fn instantiate<P: ForwardAesProvider>(
        aes: &mut P,
        entropy_input: &[u8; SEED_LEN],
        personalization: Option<&[u8; SEED_LEN]>,
        reseed_interval: u64,
    ) -> Result<Self, DrbgError> {
        if reseed_interval == 0 || reseed_interval > MAX_RESEED_INTERVAL {
            return Err(DrbgError::InvalidReseedInterval);
        }
        let mut drbg = Self {
            key: [0; 16],
            v: [0; BLOCK_LEN],
            reseed_counter: 1,
            reseed_interval,
        };
        // From `Key = 0, V = 0` the instantiate steps are exactly the reseed
        // steps, with the personalization string as `additional_input`.
        drbg.reseed(aes, entropy_input, personalization)?;
        Ok(drbg)
    }

    /// `CTR_DRBG_Reseed_algorithm` (10.2.1.4.1):
    /// `Update(entropy_input XOR additional_input)`, `reseed_counter = 1`.
    ///
    /// On a cipher error the state is wiped and the DRBG must be
    /// re-instantiated.
    pub fn reseed<P: ForwardAesProvider>(
        &mut self,
        aes: &mut P,
        entropy_input: &[u8; SEED_LEN],
        additional_input: Option<&[u8; SEED_LEN]>,
    ) -> Result<(), DrbgError> {
        let mut seed_material = *entropy_input;
        if let Some(additional_input) = additional_input {
            xor_into(&mut seed_material, additional_input);
        }
        let result = self.update(aes, Some(&seed_material));
        zeroize(&mut seed_material);
        result?;
        self.reseed_counter = 1;
        Ok(())
    }

    /// Whether the next [`Self::generate`] call needs a reseed first.
    pub const fn reseed_required(&self) -> bool {
        self.reseed_counter > self.reseed_interval
    }

    /// Generate requests served since the last (re)seed.
    pub const fn requests_since_seed(&self) -> u64 {
        self.reseed_counter - 1
    }

    /// `CTR_DRBG_Generate_algorithm` (10.2.1.5.1) for `output.len()` bytes.
    ///
    /// On any error `output` is zeroised; on a cipher error the DRBG state is
    /// also wiped.
    pub fn generate<P: ForwardAesProvider>(
        &mut self,
        aes: &mut P,
        output: &mut [u8],
        additional_input: Option<&[u8; SEED_LEN]>,
    ) -> Result<(), DrbgError> {
        let result = self.generate_inner(aes, output, additional_input);
        if result.is_err() {
            zeroize(output);
        }
        result
    }

    fn generate_inner<P: ForwardAesProvider>(
        &mut self,
        aes: &mut P,
        output: &mut [u8],
        additional_input: Option<&[u8; SEED_LEN]>,
    ) -> Result<(), DrbgError> {
        if self.reseed_required() {
            return Err(DrbgError::ReseedRequired);
        }
        if output.len() > MAX_REQUEST_BYTES {
            return Err(DrbgError::RequestTooLarge);
        }
        if additional_input.is_some() {
            self.update(aes, additional_input)?;
        }

        self.ctr_blocks(aes, output)?;

        // Step 6: Update(additional_input), with a zero string when absent.
        self.update(aes, additional_input)?;
        self.reseed_counter += 1;
        Ok(())
    }

    /// `CTR_DRBG_Update` (10.2.1.2). `None` means `provided_data = 0^seedlen`.
    fn update<P: ForwardAesProvider>(
        &mut self,
        aes: &mut P,
        provided_data: Option<&[u8; SEED_LEN]>,
    ) -> Result<(), DrbgError> {
        let mut temp = [0u8; SEED_LEN];
        self.ctr_blocks(aes, &mut temp)?;
        if let Some(provided_data) = provided_data {
            xor_into(&mut temp, provided_data);
        }
        self.key.copy_from_slice(&temp[..16]);
        self.v.copy_from_slice(&temp[16..]);
        zeroize(&mut temp);
        Ok(())
    }

    /// The counter-mode loop shared by `Update` (10.2.1.2 step 2) and
    /// `Generate` (10.2.1.5.1 step 4): `V = (V + 1) mod 2^128`, then the next
    /// output block is `Encrypt(Key, V)`, until `output` is full (the last
    /// block truncated). A cipher error zeroises `output`, wipes the state
    /// and returns [`DrbgError::Cipher`].
    fn ctr_blocks<P: ForwardAesProvider>(
        &mut self,
        aes: &mut P,
        output: &mut [u8],
    ) -> Result<(), DrbgError> {
        let mut block = [0u8; BLOCK_LEN];
        let mut failed = false;
        {
            let mut cipher = aes.forward_cipher(&self.key);
            for chunk in output.chunks_mut(BLOCK_LEN) {
                increment(&mut self.v);
                block = self.v;
                if cipher.encrypt_block(&mut block).is_err() {
                    failed = true;
                    break;
                }
                chunk.copy_from_slice(&block[..chunk.len()]);
            }
        }
        zeroize(&mut block);
        if failed {
            zeroize(output);
            self.wipe();
            return Err(DrbgError::Cipher);
        }
        Ok(())
    }

    /// Wipe the working state; the DRBG can no longer generate.
    fn wipe(&mut self) {
        zeroize(&mut self.key);
        zeroize(&mut self.v);
        self.reseed_counter = u64::MAX;
        self.reseed_interval = 0;
    }
}

impl Drop for CtrDrbg {
    fn drop(&mut self) {
        self.wipe();
    }
}

fn xor_into(target: &mut [u8; SEED_LEN], source: &[u8; SEED_LEN]) {
    for (target, source) in target.iter_mut().zip(source) {
        *target ^= source;
    }
}

/// `V = (V + 1) mod 2^128`, big-endian.
fn increment(v: &mut [u8; BLOCK_LEN]) {
    for byte in v.iter_mut().rev() {
        let (next, carry) = byte.overflowing_add(1);
        *byte = next;
        if !carry {
            break;
        }
    }
}

/// One known-answer vector for an on-target self-test: NIST CAVP
/// `drbgvectors_no_reseed/CTR_DRBG.rsp` (CAVS 14.3), `[AES-128 no df]`,
/// `[PredictionResistance = False]`, no personalization or additional
/// input, `COUNT = 0`: instantiate, generate 512 bits (discarded), generate
/// 512 bits.
pub mod self_test {
    use super::*;

    /// `EntropyInput`.
    pub const ENTROPY_INPUT: [u8; SEED_LEN] = [
        0xce, 0x50, 0xf3, 0x3d, 0xa5, 0xd4, 0xc1, 0xd3, 0xd4, 0x00, 0x4e, 0xb3, 0x52, 0x44, 0xb7,
        0xf2, 0xcd, 0x7f, 0x2e, 0x50, 0x76, 0xfb, 0xf6, 0x78, 0x0a, 0x7f, 0xf6, 0x34, 0xb2, 0x49,
        0xa5, 0xfc,
    ];
    /// `ReturnedBits` (the second 512-bit generate).
    pub const RETURNED_BITS: [u8; 64] = [
        0x65, 0x45, 0xc0, 0x52, 0x9d, 0x37, 0x24, 0x43, 0xb3, 0x92, 0xce, 0xb3, 0xae, 0x3a, 0x99,
        0xa3, 0x0f, 0x96, 0x3e, 0xaf, 0x31, 0x32, 0x80, 0xf1, 0xd1, 0xa1, 0xe8, 0x7f, 0x9d, 0xb3,
        0x73, 0xd3, 0x61, 0xe7, 0x5d, 0x18, 0x01, 0x82, 0x66, 0x49, 0x9c, 0xcc, 0xd6, 0x4d, 0x9b,
        0xbb, 0x8d, 0xe0, 0x18, 0x5f, 0x21, 0x33, 0x83, 0x08, 0x0f, 0xad, 0xde, 0xc4, 0x6b, 0xae,
        0x1f, 0x78, 0x4e, 0x5a,
    ];

    /// Failure of [`run`].
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum SelfTestError {
        Drbg(DrbgError),
        Mismatch,
    }

    /// Run the vector through `aes`. Nothing from the test leaks into a
    /// live DRBG: the test instance is wiped on drop.
    pub fn run<P: ForwardAesProvider>(aes: &mut P) -> Result<(), SelfTestError> {
        let mut drbg =
            CtrDrbg::instantiate(aes, &ENTROPY_INPUT, None, 2).map_err(SelfTestError::Drbg)?;
        let mut output = [0u8; 64];
        drbg.generate(aes, &mut output, None)
            .map_err(SelfTestError::Drbg)?;
        drbg.generate(aes, &mut output, None)
            .map_err(SelfTestError::Drbg)?;
        let matches = output == RETURNED_BITS;
        zeroize(&mut output);
        if matches {
            Ok(())
        } else {
            Err(SelfTestError::Mismatch)
        }
    }
}

#[cfg(test)]
mod tests {
    //! Known answers from NIST CAVP `drbgtestvectors.zip` (CAVS 14.3, SP
    //! 800-90A), file `CTR_DRBG.rsp`, sections `[AES-128 no df]`
    //! `[PredictionResistance = False]` `[EntropyInputLen = 256]`
    //! `[NonceLen = 0]` `[ReturnedBitsLen = 512]`:
    //!
    //! * `drbgvectors_no_reseed`: Instantiate, Generate, Generate (the
    //!   second output is `ReturnedBits`);
    //! * `drbgvectors_pr_false`: Instantiate, Reseed(`EntropyInputReseed`,
    //!   `AdditionalInputReseed`), Generate, Generate.
    //!
    //! <https://csrc.nist.gov/projects/cryptographic-algorithm-validation-program/random-number-generators>
    use super::*;
    use crate::SoftwareAesProvider;

    fn hex<const N: usize>(text: &str) -> [u8; N] {
        assert_eq!(text.len(), 2 * N, "{text}");
        let mut out = [0u8; N];
        for (index, byte) in out.iter_mut().enumerate() {
            *byte = u8::from_str_radix(&text[2 * index..2 * index + 2], 16).unwrap();
        }
        out
    }

    fn opt(text: &str) -> Option<[u8; SEED_LEN]> {
        (!text.is_empty()).then(|| hex(text))
    }

    struct Vector {
        entropy: &'static str,
        personalization: &'static str,
        reseed: Option<(&'static str, &'static str)>,
        additional: [&'static str; 2],
        returned: &'static str,
    }

    fn check(vector: &Vector) {
        let mut aes = SoftwareAesProvider::new();
        let personalization = opt(vector.personalization);
        let mut drbg = CtrDrbg::instantiate(
            &mut aes,
            &hex(vector.entropy),
            personalization.as_ref(),
            1 << 20,
        )
        .unwrap();
        if let Some((entropy, additional)) = vector.reseed {
            drbg.reseed(&mut aes, &hex(entropy), opt(additional).as_ref())
                .unwrap();
        }
        let mut output = [0u8; 64];
        for additional in vector.additional {
            drbg.generate(&mut aes, &mut output, opt(additional).as_ref())
                .unwrap();
        }
        assert_eq!(output, hex::<64>(vector.returned));
    }

    #[test]
    fn cavp_no_reseed_vectors() {
        // [PersonalizationStringLen = 0] [AdditionalInputLen = 0] COUNT = 0
        check(&Vector {
            entropy: "ce50f33da5d4c1d3d4004eb35244b7f2cd7f2e5076fbf6780a7ff634b249a5fc",
            personalization: "",
            reseed: None,
            additional: ["", ""],
            returned: "6545c0529d372443b392ceb3ae3a99a30f963eaf313280f1d1a1e87f9db373d3\
                       61e75d18018266499cccd64d9bbb8de0185f213383080faddec46bae1f784e5a",
        });
        // COUNT = 1
        check(&Vector {
            entropy: "a385f70a4d450321dfd18d8379ef8e7736fee5fbf0a0aea53b76696094e8aa93",
            personalization: "",
            reseed: None,
            additional: ["", ""],
            returned: "1a062553ab60457ed1f1c52f5aca5a3be564a27545358c112ed92c6eae2cb759\
                       7cfcc2e0a5dd81c5bfecc941da5e8152a9010d4845170734676c8c1b6b3073a5",
        });
        // [PersonalizationStringLen = 0] [AdditionalInputLen = 256] COUNT = 0
        check(&Vector {
            entropy: "6bd4f2ae649fc99350951ff0c5d460c1a9214154e7384975ee54b34b7cae0704",
            personalization: "",
            reseed: None,
            additional: [
                "ecd4893b979ac92db1894ae3724518a2f78cf2dbe2f6bbc6fda596df87c7a4ae",
                "b23e9188687c88768b26738862c4791fa52f92502e1f94bf66af017c4228a0dc",
            ],
            returned: "5b2bf7a5c60d8ab6591110cbd61cd387b02de19784f496d1a109123d8b3562a5\
                       de2dd6d5d1aef957a6c4f371cecd93c15799d82e34d6a0dba7e915a27d8e65f3",
        });
        // [PersonalizationStringLen = 256] [AdditionalInputLen = 0] COUNT = 0
        check(&Vector {
            entropy: "cee23de86a69c7ef57f6e1e12bd16e35e51624226fa19597bf93ec476a44b0f2",
            personalization: "a2ef16f226ea324f23abd59d5e3c660561c25e73638fe21c87566e86a9e04c3e",
            reseed: None,
            additional: ["", ""],
            returned: "2a76d71b329f449c98dc08fff1d205a2fbd9e4ade120c7611c225c984eac8531\
                       288dd3049f3dc3bb3671501ab8fbf9ad49c86cce307653bd8caf29cb0cf07764",
        });
        // [PersonalizationStringLen = 256] [AdditionalInputLen = 256] COUNT = 0
        check(&Vector {
            entropy: "50b96542a1f2b8b05074051fe8fb0e45adbbd5560e3594e12d485fe1bfcb741f",
            personalization: "820c3030f97b3ead81a93b88b871937278fd3d711d2085d9280cba394673b17e",
            reseed: None,
            additional: [
                "1f1632058806d6d8e231288f3b15a3c324e90ccef4891bd595f09c3e80e27469",
                "5cadc8bfd86d2a5d44f921f64c7d153001b9bdd7caa6618639b948ebfad5cb8a",
            ],
            returned: "02b76a66f103e98d450e25e09c35337747d987471d2b3d81e03be24c7e985417\
                       a32acd72bc0a6eddd9871410dacb921c659249b4e2b368c4ac8580fb5db559bc",
        });
    }

    #[test]
    fn cavp_reseed_vectors() {
        // [PersonalizationStringLen = 0] [AdditionalInputLen = 0] COUNT = 0
        check(&Vector {
            entropy: "ed1e7f21ef66ea5d8e2a85b9337245445b71d6393a4eecb0e63c193d0f72f9a9",
            personalization: "",
            reseed: Some((
                "303fb519f0a4e17d6df0b6426aa0ecb2a36079bd48be47ad2a8dbfe48da3efad",
                "",
            )),
            additional: ["", ""],
            returned: "f80111d08e874672f32f42997133a5210f7a9375e22cea70587f9cfafebe0f6a\
                       6aa2eb68e7dd9164536d53fa020fcab20f54caddfab7d6d91e5ffec1dfd8deaa",
        });
        // [PersonalizationStringLen = 0] [AdditionalInputLen = 256] COUNT = 0
        check(&Vector {
            entropy: "6bc709aa4c975b0eccb922ce2110fa9b572403f9013dfd10f06a88d54d380002",
            personalization: "",
            reseed: Some((
                "cf1af84eddd5bef666ea42bea6067a23e52742e24661f944ba2514fe052abf31",
                "a46988bad49b78c613c94e06a53b080bf6d20b7385bf4c782ad7cd145ddc9053",
            )),
            additional: [
                "139d6f72bf1d0ec5bfdd245e013f5cdac85e3eca716196018b92133c00a07436",
                "240f1a5af7fc2e4d32ace635acba5947f3564ecbfd7516c479c0adda20747f26",
            ],
            returned: "e727268a546c0c891cf53a70a92820ee9bbf728ad52f30625b2e28f0f6c906f6\
                       0ffd02f7d81623295950c04b63a48634eb41a5b4d649bdabff335ac3200690b0",
        });
        // [PersonalizationStringLen = 256] [AdditionalInputLen = 0] COUNT = 0
        check(&Vector {
            entropy: "34cbc2b217f3d907fa2ad6a0d7a813b0fda1e17fbeed94b0e0a0abfbec947146",
            personalization: "e8fa4c5de825791e68180f2ba107e829c48299cb01be939cd0be76da120a91f2",
            reseed: Some((
                "8326f8e9cfbd02eb076bbb9819d96a02386f80bf913c8e4a80361d82cafad52e",
                "",
            )),
            additional: ["", ""],
            returned: "52f5e718bf48d99e498775c00378e545799bb2059aef0b74be573d8283f02b52\
                       93917913bc8f26fc23760a1c86c3f5c844857419868eafeb17c9248227d026b8",
        });
        // [PersonalizationStringLen = 256] [AdditionalInputLen = 256] COUNT = 0
        check(&Vector {
            entropy: "289e5c8283cbd7dbe707255cb3cf2907d8a5ce5b347314966f9b2bebb1a1e200",
            personalization: "7f7b59f23510b976fe155d047525c94e2dacb30d77ac8b09281544dd815d5293",
            reseed: Some((
                "98c522028f36fc6b85a8f3c003efd4b130dd90180ec81cf7c67d4c53d10f0022",
                "f7a0378328d939f0f8521e39409d7175d87319c7597a9050414f7adc392a328d",
            )),
            additional: [
                "19c286f5b36194d1cc62c0188140bc9d61d2a9c5d88bb5aebc224bfb04dfca83",
                "820650c3201d347f5b20d3d25d1c8c7bef4d9f66a5a04c7dd9d669e95182a0c4",
            ],
            returned: "79a79d44edada58e3fc12a4e36ae900eeace290265f01262f40f2958a70dcbd4\
                       d4185f708c088ede7ff8c8375f44f4012f2512d38328a5df171a17029d90f185",
        });
    }

    #[test]
    fn embedded_self_test_vector_passes() {
        assert_eq!(self_test::run(&mut SoftwareAesProvider::new()), Ok(()));
    }

    #[test]
    fn partial_blocks_are_a_prefix_of_full_blocks() {
        let mut aes = SoftwareAesProvider::new();
        let entropy = [7u8; SEED_LEN];
        let mut a = CtrDrbg::instantiate(&mut aes, &entropy, None, 8).unwrap();
        let mut b = CtrDrbg::instantiate(&mut aes, &entropy, None, 8).unwrap();
        let mut full = [0u8; 32];
        let mut partial = [0u8; 21];
        a.generate(&mut aes, &mut full, None).unwrap();
        b.generate(&mut aes, &mut partial, None).unwrap();
        assert_eq!(&full[..21], &partial[..]);
    }

    #[test]
    fn reseed_interval_is_enforced() {
        let mut aes = SoftwareAesProvider::new();
        let mut drbg = CtrDrbg::instantiate(&mut aes, &[1; SEED_LEN], None, 2).unwrap();
        let mut out = [0u8; 16];
        drbg.generate(&mut aes, &mut out, None).unwrap();
        drbg.generate(&mut aes, &mut out, None).unwrap();
        assert!(drbg.reseed_required());
        out = [0xA5; 16];
        assert_eq!(
            drbg.generate(&mut aes, &mut out, None),
            Err(DrbgError::ReseedRequired)
        );
        assert_eq!(out, [0; 16], "a refused request must not leave stale bytes");
        drbg.reseed(&mut aes, &[2; SEED_LEN], None).unwrap();
        assert_eq!(drbg.requests_since_seed(), 0);
        drbg.generate(&mut aes, &mut out, None).unwrap();
    }

    #[test]
    fn invalid_parameters_are_refused() {
        let mut aes = SoftwareAesProvider::new();
        assert!(matches!(
            CtrDrbg::instantiate(&mut aes, &[0; SEED_LEN], None, 0),
            Err(DrbgError::InvalidReseedInterval)
        ));
        assert!(matches!(
            CtrDrbg::instantiate(&mut aes, &[0; SEED_LEN], None, MAX_RESEED_INTERVAL + 1),
            Err(DrbgError::InvalidReseedInterval)
        ));
    }

    /// A cipher that fails after `budget` blocks.
    struct FailingProvider {
        budget: usize,
    }

    struct FailingCipher<'a> {
        inner: crate::SoftwareAes128,
        budget: &'a mut usize,
    }

    impl Aes128Forward for FailingCipher<'_> {
        type Error = ();
        fn encrypt_block(&mut self, block: &mut [u8; 16]) -> Result<(), ()> {
            if *self.budget == 0 {
                return Err(());
            }
            *self.budget -= 1;
            let _ = self.inner.encrypt_block(block);
            Ok(())
        }
    }

    impl ForwardAesProvider for FailingProvider {
        fn forward_cipher(&mut self, key: &AesKey) -> impl Aes128Forward + '_ {
            FailingCipher {
                inner: crate::SoftwareAes128::new(key),
                budget: &mut self.budget,
            }
        }
    }

    #[test]
    fn cipher_failure_wipes_state_and_output() {
        // Instantiate needs 2 blocks; then fail in the middle of generate.
        let mut aes = FailingProvider { budget: 3 };
        let mut drbg = CtrDrbg::instantiate(&mut aes, &[3; SEED_LEN], None, 16).unwrap();
        let mut out = [0xA5u8; 48];
        assert_eq!(
            drbg.generate(&mut aes, &mut out, None),
            Err(DrbgError::Cipher)
        );
        assert_eq!(out, [0; 48]);
        assert_eq!(drbg.key, [0; 16]);
        assert_eq!(drbg.v, [0; 16]);
        // The wiped DRBG refuses to generate even with a working cipher.
        let mut aes = SoftwareAesProvider::new();
        out = [0xA5; 48];
        assert_eq!(
            drbg.generate(&mut aes, &mut out, None),
            Err(DrbgError::ReseedRequired)
        );
        assert_eq!(out, [0; 48]);
    }

    #[test]
    fn cipher_failure_during_instantiate_is_reported() {
        let mut aes = FailingProvider { budget: 1 };
        assert!(matches!(
            CtrDrbg::instantiate(&mut aes, &[3; SEED_LEN], None, 16),
            Err(DrbgError::Cipher)
        ));
    }

    #[test]
    fn counter_increment_wraps_big_endian() {
        let mut v = [0xFF; 16];
        increment(&mut v);
        assert_eq!(v, [0; 16]);
        let mut v = [0; 16];
        v[15] = 0xFF;
        increment(&mut v);
        assert_eq!(v[14..], [1, 0]);
    }
}
