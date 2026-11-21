//! GPD key provisioning on the sink (feature `key-provisioning`).
//!
//! Follows gecko_sdk `sendCommissioningReply`,
//! `emberAfGreenPowerServerDeriveSharedKeyFromSinkAttribute` and
//! `sli_zigbee_af_gp_key_tc_lk_derivation`:
//!
//! * **Shared key attributes.** gpSharedSecurityKeyType (0x0020) and
//!   gpSharedSecurityKey (0x0021) select the key the sink hands to a GPD that
//!   asks for one (GP Security Key Request in a bidirectional Commissioning
//!   GPDF): the network key (type 1), the NWK-key derived group key
//!   (type 3, HMAC-AES-MMO of the network key over "ZGP", A.1.5.7.1), the
//!   GPD group key itself (type 2) or a derived individual key (type 7,
//!   HMAC-AES-MMO of the group key over the SrcID or the GPD IEEE address,
//!   A.1.5.7.2 / A.1.5.12.1). The key goes in the Commissioning Reply and
//!   becomes the Sink Table entry's key; it is protected with gpLinkKey
//!   (0x0022, default ZigBeeAlliance09, A.1.5.8.3) when the GPD signalled
//!   key encryption in its Commissioning GPDF.
//! * **Out-of-band keys.** A GPD whose individual key is known out of band
//!   (a label, a QR code) may commission without sending it: the product
//!   or host provisions the key ([`Sink::provision_oob_key`]) before the
//!   GPD commissions. A GPD re-commissioning without a key also keeps its
//!   Sink Table entry's key.
//!
//! Differences from gecko_sdk: the network key (type 1) is always sent
//! TC-LK protected, also to a GPD that did not signal key encryption
//! (gecko_sdk would send it in clear); gpSharedSecurityKey and gpLinkKey are
//! write-only (a read returns UNSUPPORTED_ATTRIBUTE instead of the key to
//! any node holding the network key); gpLinkKey is honoured for key
//! protection in both directions (gecko_sdk always uses ZigBeeAlliance09);
//! the out-of-band store is an addition (gecko_sdk drops a protected
//! GPD that sends no key unidirectionally).

use zigbee_crypto::{AesKey, ForwardAesProvider};

use super::{Sink, zcl_status};
use crate::cluster::attr;
use crate::codec::Writer;
use crate::security::{self, DEFAULT_TC_LINK_KEY, Direction};
use crate::types::{GpdId, KeyType};
use crate::{Error, Result};

/// Out-of-band GPD keys held until the GPD commissions.
pub const MAX_OOB_KEYS: usize = 4;

/// ZCL "security key 128" data type.
const SECURITY_KEY: u8 = 0xF1;
/// ZCL bitmap8.
const BITMAP8: u8 = 0x18;

/// Key material of the sink.
#[derive(Debug, Clone)]
pub(super) struct KeyStore {
    /// gpSharedSecurityKeyType.
    shared_key_type: KeyType,
    /// gpSharedSecurityKey.
    shared_key: AesKey,
    /// gpLinkKey.
    link_key: AesKey,
    /// The network's active key (types 1 and 3), from the runtime.
    network_key: Option<AesKey>,
    oob: heapless::Vec<(GpdId, AesKey), MAX_OOB_KEYS>,
}

impl KeyStore {
    pub(super) const fn new() -> Self {
        Self {
            shared_key_type: KeyType::NoKey,
            shared_key: [0; 16],
            link_key: DEFAULT_TC_LINK_KEY,
            network_key: None,
            oob: heapless::Vec::new(),
        }
    }
}

/// HMAC-AES-MMO (Zigbee B.1.4) keyed with `key` over `message` (at most 16
/// bytes here).
fn hmac_mmo<P: ForwardAesProvider>(
    provider: &mut P,
    key: &AesKey,
    message: &[u8],
) -> Result<AesKey> {
    const IPAD: u8 = 0x36;
    const OPAD: u8 = 0x5C;
    let mut inner = [0u8; 32];
    for (o, k) in inner[..16].iter_mut().zip(key) {
        *o = k ^ IPAD;
    }
    let n = message.len().min(16);
    inner[16..16 + n].copy_from_slice(&message[..n]);
    let inner_hash = zigbee_crypto::aes_mmo_hash_with(provider, &inner[..16 + n])
        .map_err(|_| Error::Unsupported)?;
    let mut outer = [0u8; 32];
    for (o, k) in outer[..16].iter_mut().zip(key) {
        *o = k ^ OPAD;
    }
    outer[16..].copy_from_slice(&inner_hash);
    zigbee_crypto::aes_mmo_hash_with(provider, &outer).map_err(|_| Error::Unsupported)
}

/// The key of `key_type` gecko_sdk derives from the sink attributes for
/// `gpd`; `None` without the needed network key or for "no key".
pub fn derive_shared_key<P: ForwardAesProvider>(
    provider: &mut P,
    key_type: KeyType,
    shared_key: &AesKey,
    network_key: Option<&AesKey>,
    gpd: &GpdId,
) -> Option<AesKey> {
    match key_type {
        KeyType::NoKey => None,
        KeyType::NwkKey => network_key.copied(),
        KeyType::NwkKeyDerivedGroupKey => hmac_mmo(provider, network_key?, b"ZGP").ok(),
        KeyType::GpdGroupKey | KeyType::IndividualKey => Some(*shared_key),
        KeyType::DerivedIndividualKey => match gpd {
            GpdId::SrcId(id) => hmac_mmo(provider, shared_key, &id.to_le_bytes()).ok(),
            GpdId::Ieee { ieee, .. } => hmac_mmo(provider, shared_key, ieee).ok(),
        },
    }
}

/// The key a sink sends in a Commissioning Reply.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct ProvidedKey {
    pub key: AesKey,
    pub key_type: KeyType,
}

impl<const N: usize, const Q: usize> Sink<N, Q> {
    /// Set gpSharedSecurityKeyType and gpSharedSecurityKey (as a write of
    /// both attributes). [`KeyType::NoKey`] stops key provisioning.
    pub fn set_shared_key(&mut self, key_type: KeyType, key: AesKey) {
        self.keys.shared_key_type = key_type;
        self.keys.shared_key = key;
    }

    /// gpSharedSecurityKeyType.
    pub fn shared_key_type(&self) -> KeyType {
        self.keys.shared_key_type
    }

    /// Set gpLinkKey, the key that protects GPD keys at commissioning.
    pub fn set_gp_link_key(&mut self, key: AesKey) {
        self.keys.link_key = key;
    }

    /// The network's active key, for the shared key types 1 and 3 (the
    /// runtime keeps it current).
    pub fn set_network_key(&mut self, key: Option<AesKey>) {
        self.keys.network_key = key;
    }

    /// Hold the out-of-band key of `gpd` until it commissions (replaces an
    /// earlier one). Fails with [`Error::BufferFull`] when
    /// [`MAX_OOB_KEYS`] other GPDs wait.
    pub fn provision_oob_key(&mut self, gpd: GpdId, key: AesKey) -> Result<()> {
        if gpd.is_reserved() {
            return Err(Error::Unsupported);
        }
        if let Some(slot) = self.keys.oob.iter_mut().find(|(g, _)| g.matches(&gpd)) {
            slot.1 = key;
            return Ok(());
        }
        self.keys
            .oob
            .push((gpd, key))
            .map_err(|_| Error::BufferFull)
    }

    /// Forget the out-of-band key of `gpd`; returns whether one was held.
    pub fn forget_oob_key(&mut self, gpd: &GpdId) -> bool {
        let before = self.keys.oob.len();
        self.keys.oob.retain(|(g, _)| !g.matches(gpd));
        self.keys.oob.len() != before
    }

    /// The link key protecting GPD keys (gpLinkKey).
    pub(super) fn gp_link_key(&self) -> &AesKey {
        &self.keys.link_key
    }

    /// A key for a protected GPD that sent none: its out-of-band key (used
    /// once), else its Sink Table entry's key of the same kind.
    pub(super) fn stored_key(&mut self, gpd: &GpdId, key_type: KeyType) -> Option<AesKey> {
        if let Some(i) = self.keys.oob.iter().position(|(g, _)| g.matches(gpd)) {
            return Some(self.keys.oob.swap_remove(i).1);
        }
        self.table
            .iter()
            .find(|e| e.gpd_id.matches(gpd) && e.key_type == key_type)
            .and_then(|e| e.key)
    }

    /// The key gecko_sdk `sendCommissioningReply` hands to a GPD that
    /// asked for one: the shared key when its type is configured and
    /// differs from the GPD's own key type.
    pub(super) fn key_to_provide<P: ForwardAesProvider>(
        &self,
        aes: &mut P,
        gpd: &GpdId,
        gpd_key_type: KeyType,
        requested: bool,
    ) -> Option<ProvidedKey> {
        let k = &self.keys;
        if !requested || k.shared_key_type == KeyType::NoKey || k.shared_key_type == gpd_key_type {
            return None;
        }
        let key = derive_shared_key(
            aes,
            k.shared_key_type,
            &k.shared_key,
            k.network_key.as_ref(),
            gpd,
        )?;
        Some(ProvidedKey {
            key,
            key_type: k.shared_key_type,
        })
    }

    /// The Commissioning Reply's key field: TC-LK protected with gpLinkKey
    /// and frame counter `gpd_counter + 1` (A.1.5.8.3) when `encrypt`.
    pub(super) fn reply_key<P: ForwardAesProvider>(
        &self,
        aes: &mut P,
        gpd: &GpdId,
        key: &AesKey,
        encrypt: bool,
        gpd_counter: u32,
    ) -> Option<(AesKey, Option<(u32, u32)>)> {
        if !encrypt {
            return Some((*key, None));
        }
        let fc = gpd_counter.wrapping_add(1);
        let w =
            security::wrap_key(aes, &self.keys.link_key, gpd, Direction::ToGpd, fc, key).ok()?;
        Some((w.key, Some((u32::from_le_bytes(w.mic), fc))))
    }

    /// Read gpSharedSecurityKeyType; the keys are write-only.
    pub(super) fn read_key_attribute(&self, id: u16, w: &mut Writer<'_>) -> Result<Option<u8>> {
        match id {
            attr::GP_SHARED_SECURITY_KEY_TYPE => {
                w.u8(self.keys.shared_key_type as u8)?;
                Ok(Some(BITMAP8))
            }
            _ => Ok(None),
        }
    }

    /// Write one of the key attributes; `None` for another attribute.
    pub(super) fn write_key_attribute(
        &mut self,
        id: u16,
        data_type: u8,
        value: &[u8],
    ) -> Option<u8> {
        match id {
            attr::GP_SHARED_SECURITY_KEY_TYPE => {
                Some(if data_type != BITMAP8 || value.len() != 1 {
                    zcl_status::INVALID_DATA_TYPE
                } else {
                    match KeyType::from_bits(value[0]) {
                        Ok(t) if value[0] & !0x07 == 0 => {
                            self.keys.shared_key_type = t;
                            zcl_status::SUCCESS
                        }
                        _ => zcl_status::INVALID_VALUE,
                    }
                })
            }
            attr::GP_SHARED_SECURITY_KEY | attr::GP_LINK_KEY => {
                Some(if data_type != SECURITY_KEY || value.len() != 16 {
                    zcl_status::INVALID_DATA_TYPE
                } else {
                    let mut key = [0u8; 16];
                    key.copy_from_slice(value);
                    if id == attr::GP_LINK_KEY {
                        self.keys.link_key = key;
                    } else {
                        self.keys.shared_key = key;
                    }
                    zcl_status::SUCCESS
                })
            }
            _ => None,
        }
    }
}

/// Whether a Commissioning GPDF payload signals GPD key encryption (the
/// extended options' GPDkeyEncryption bit, which a GPD also sets when it
/// asks for a key it wants protected).
pub(super) fn gpd_key_encryption(payload: &[u8]) -> bool {
    match payload {
        [_, options, ext, ..] if options & 0x80 != 0 => ext & 0x40 != 0,
        _ => false,
    }
}

#[cfg(test)]
mod tests;
