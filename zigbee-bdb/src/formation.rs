//! Network Formation commissioning (BDB v3.0.1 spec §8.4).
//!
//! Network Formation is only available on coordinator-capable devices.
//! It creates a new Zigbee PAN using `NLME-NETWORK-FORMATION`.
//!
//! ## Procedure
//! 1. Verify device is coordinator-capable
//! 2. Form network on primary channels (`NLME-NETWORK-FORMATION`)
//! 3. If primary fails, retry on secondary channels
//! 4. Set up Trust Center policies
//! 5. Install network key
//! 6. Open permit joining so other devices can join
//!
//! ## Security modes
//! - **Centralized**: coordinator acts as Trust Center, distributes NWK key
//! - **Distributed**: routers form their own trust domain (no TC)

use zigbee_mac::MacDriver;
use zigbee_nwk::DeviceType;
use zigbee_types::ShortAddress;

use crate::attributes::BDB_MIN_COMMISSIONING_TIME;
use crate::{BdbLayer, BdbStatus};

/// Scan duration exponent for ED scan during formation.
const SCAN_DURATION: u8 = 3;

impl<M: MacDriver> BdbLayer<M> {
    /// Execute the Network Formation procedure (BDB spec §8.4).
    ///
    /// Only coordinator-capable devices may form a network.
    /// On success, the device becomes the PAN coordinator and Trust Center.
    pub async fn network_formation(&mut self) -> Result<(), BdbStatus> {
        // Step 1: Verify coordinator capability
        if self.zdo.nwk().device_type() != DeviceType::Coordinator {
            log::warn!("[BDB:Formation] Not a coordinator — skipping");
            return Err(BdbStatus::NotPermitted);
        }

        if self.attributes.node_is_on_a_network {
            log::info!("[BDB:Formation] Already on a network");
            return Ok(());
        }

        log::info!("[BDB:Formation] Forming new network…");

        // Step 2: Attempt formation on primary channels
        let channel_sets = [
            self.attributes.primary_channel_set,
            self.attributes.secondary_channel_set,
        ];

        for (idx, &channel_mask) in channel_sets.iter().enumerate() {
            if channel_mask.0 == 0 {
                continue;
            }

            let set_name = if idx == 0 { "primary" } else { "secondary" };
            log::debug!(
                "[BDB:Formation] Trying {} channels: 0x{:08X}",
                set_name,
                channel_mask.0,
            );

            match self
                .zdo
                .nlme_network_formation(channel_mask, SCAN_DURATION)
                .await
            {
                Ok(()) => {
                    log::info!("[BDB:Formation] Network formed on {} channels", set_name);
                    return self.post_formation_setup().await;
                }
                Err(e) => {
                    log::debug!(
                        "[BDB:Formation] Formation failed on {} channels: {:?}",
                        set_name,
                        e,
                    );
                }
            }
        }

        self.attributes.commissioning_status =
            crate::attributes::BdbCommissioningStatus::SteeringFormationFailure;
        Err(BdbStatus::FormationFailure)
    }

    /// Post-formation setup: TC policies, NWK key, permit joining.
    async fn post_formation_setup(&mut self) -> Result<(), BdbStatus> {
        // Step 3: Set Trust Center policies
        // The coordinator IS the Trust Center in centralized mode.
        log::debug!("[BDB:Formation] Configuring Trust Center policies");

        // Store our own address as TC address in AIB
        let ieee = self.zdo.nwk().nib().ieee_address;
        self.zdo.aps_mut().aib_mut().aps_trust_center_address = ieee;
        self.zdo.aps_mut().aib_mut().aps_designated_coordinator = true;

        // Step 4: Generate and install NWK key
        // The coordinator must install a network key so that all secured
        // NWK frames can be encrypted. Without this, NWK security is broken.
        let nwk_key = generate_nwk_key();
        let key_seq: u8 = 0;
        self.zdo
            .nwk_mut()
            .security_mut()
            .set_network_key(nwk_key, key_seq);
        self.zdo.nwk_mut().nib_mut().active_key_seq_number = key_seq;
        log::info!("[BDB:Formation] NWK key installed (seq={})", key_seq);

        // Step 5: Open permit joining so other devices can join
        let duration = core::cmp::min(BDB_MIN_COMMISSIONING_TIME, 254) as u8;
        if let Err(e) = self.zdo.nlme_permit_joining(duration).await {
            log::warn!("[BDB:Formation] Failed to open permit joining: {:?}", e);
            // Non-fatal — the network is formed, just not open for joining
        }

        // Broadcast Mgmt_Permit_Joining_req so all routers open
        let _ = self
            .zdo
            .mgmt_permit_joining_req(ShortAddress::BROADCAST, duration, true)
            .await;

        self.attributes.node_is_on_a_network = true;
        self.attributes.commissioning_status = crate::attributes::BdbCommissioningStatus::Success;

        log::info!(
            "[BDB:Formation] Network ready, permit joining open for {}s",
            duration,
        );
        Ok(())
    }
}

/// Generate a 128-bit NWK key using a simple PRNG.
///
/// **Production note**: this should use a hardware RNG (TRNG) for
/// cryptographic strength. The xorshift here is a placeholder suitable
/// for bring-up and testing only.
fn generate_nwk_key() -> [u8; 16] {
    static mut SEED: u32 = 0xCAFE_BABE;
    let mut key = [0u8; 16];
    for chunk in key.chunks_exact_mut(4) {
        unsafe {
            SEED ^= SEED << 13;
            SEED ^= SEED >> 17;
            SEED ^= SEED << 5;
            chunk.copy_from_slice(&SEED.to_le_bytes());
        }
    }
    key
}
