//! Network Steering commissioning (BDB v3.0.1 spec §8.3).
//!
//! Network Steering has two operating modes depending on whether the
//! device is already on a network:
//!
//! ## Not on a network
//! 1. Scan primary channels for open networks (`NLME-NETWORK-DISCOVERY`)
//! 2. Filter by extended PAN ID if `bdbUseExtendedPanId` is configured
//! 3. Attempt to join the best-LQI network (`NLME-JOIN`)
//! 4. On join success: broadcast `Device_annce`
//! 5. Request Trust Center link key (APSME-REQUEST-KEY)
//! 6. If primary channels fail, retry on secondary channels
//!
//! ## Already on a network
//! 1. Open local permit joining for `bdbcMinCommissioningTime`
//! 2. Broadcast `Mgmt_Permit_Joining_req` to the network

use zigbee_mac::MacDriver;
use zigbee_nwk::DeviceType;
use zigbee_types::ShortAddress;

use crate::attributes::BDB_MIN_COMMISSIONING_TIME;
use crate::{BdbLayer, BdbStatus};

/// Default scan duration exponent for active scan (2^n + 1 superframes).
/// Exponent 3 ≈ 138 ms per channel — good balance of speed vs. reliability.
const SCAN_DURATION: u8 = 3;

impl<M: MacDriver> BdbLayer<M> {
    /// Execute the Network Steering procedure (BDB spec §8.3).
    ///
    /// Behaviour depends on `bdbNodeIsOnANetwork`:
    /// - **Not on network**: scan → join → announce → TC key exchange
    /// - **On network**: open permit joining → broadcast Mgmt_Permit_Joining_req
    pub async fn network_steering(&mut self) -> Result<(), BdbStatus> {
        if self.attributes.node_is_on_a_network {
            self.steer_on_network().await
        } else {
            self.steer_off_network().await
        }
    }

    /// Steering when the device is NOT on a network — join an existing PAN.
    async fn steer_off_network(&mut self) -> Result<(), BdbStatus> {
        log::info!("[BDB:Steering] Scanning for open networks…");

        // Try primary channels first, then secondary
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
                "[BDB:Steering] Scanning {} channels: 0x{:08X}",
                set_name,
                channel_mask.0
            );

            // Step 1: Network discovery
            let networks = match self
                .zdo
                .nlme_network_discovery(channel_mask, SCAN_DURATION)
                .await
            {
                Ok(n) => n,
                Err(_) => {
                    log::debug!("[BDB:Steering] No networks on {} channels", set_name);
                    continue;
                }
            };

            log::debug!("[BDB:Steering] Found {} network(s)", networks.len());

            // Step 2: Filter by extended PAN ID if configured
            let use_epid = self.zdo.aps().aib().aps_use_extended_pan_id;
            let has_epid_filter = use_epid != [0u8; 8];

            // Step 3: Try joining each network (already sorted by LQI)
            for network in &networks {
                // Apply extended PAN ID filter
                if has_epid_filter && network.extended_pan_id != use_epid {
                    log::debug!(
                        "[BDB:Steering] Skipping PAN 0x{:04X} — EPID mismatch",
                        network.pan_id.0,
                    );
                    continue;
                }

                // Must have permit joining enabled
                if !network.permit_joining {
                    continue;
                }

                log::info!(
                    "[BDB:Steering] Joining PAN 0x{:04X} ch {} LQI {}",
                    network.pan_id.0,
                    network.logical_channel,
                    network.lqi,
                );

                // Step 3: Attempt join
                let nwk_addr = match self.zdo.nlme_join(network).await {
                    Ok(addr) => addr,
                    Err(e) => {
                        log::warn!("[BDB:Steering] Join failed: {:?}", e);
                        continue;
                    }
                };

                // Step 4: Announce our presence
                let ieee = self.zdo.nwk().nib().ieee_address;
                if let Err(e) = self.zdo.device_annce(nwk_addr, ieee).await {
                    log::warn!("[BDB:Steering] Device_annce failed: {:?}", e);
                    // Non-fatal — continue commissioning
                }

                // Step 5: Start router if we are a router
                if self.zdo.nwk().device_type() == DeviceType::Router {
                    let _ = self.zdo.nlme_start_router().await;
                }

                // Step 5b: TC link key exchange
                // Zigbee 3.0: coordinator sends APS Transport-Key command
                // containing the network key after we join. This is handled
                // automatically by process_incoming_aps_frame() which parses
                // Transport-Key commands and installs the NWK key into
                // NwkSecurity. The well-known TC link key (ZigBeeAlliance09)
                // is pre-installed in ApsSecurity.
                //
                // For Install Code devices, APSME-REQUEST-KEY would be needed.
                log::debug!("[BDB:Steering] Awaiting Transport-Key from coordinator");

                // Success!
                self.attributes.node_is_on_a_network = true;
                self.attributes.commissioning_status =
                    crate::attributes::BdbCommissioningStatus::Success;

                log::info!("[BDB:Steering] Joined successfully as 0x{:04X}", nwk_addr.0,);
                return Ok(());
            }
        }

        // All attempts exhausted
        self.attributes.commissioning_status =
            crate::attributes::BdbCommissioningStatus::NoScanResponse;
        Err(BdbStatus::NoScanResponse)
    }

    /// Steering when the device IS already on a network.
    ///
    /// Opens the network for joining and broadcasts Mgmt_Permit_Joining_req
    /// so that routers in the network also open their permit joining.
    async fn steer_on_network(&mut self) -> Result<(), BdbStatus> {
        log::info!("[BDB:Steering] Already on network — opening permit joining");

        // Can only permit joining on coordinator / router
        if self.zdo.nwk().device_type() == DeviceType::EndDevice {
            log::debug!("[BDB:Steering] End device — skipping permit joining");
            // End devices can only trigger steering-on-network by sending
            // Mgmt_Permit_Joining_req to their parent / coordinator.
            let _ = self
                .zdo
                .mgmt_permit_joining_req(
                    ShortAddress::COORDINATOR,
                    BDB_MIN_COMMISSIONING_TIME as u8,
                    true,
                )
                .await;
            return Ok(());
        }

        // Open local permit joining (duration = bdbcMinCommissioningTime)
        // Duration is capped at 254 (0xFE) seconds per Zigbee spec.
        let duration = core::cmp::min(BDB_MIN_COMMISSIONING_TIME, 254) as u8;

        self.zdo
            .nlme_permit_joining(duration)
            .await
            .map_err(|_| BdbStatus::SteeringFailure)?;

        // Broadcast Mgmt_Permit_Joining_req to all routers
        self.zdo
            .mgmt_permit_joining_req(ShortAddress::BROADCAST, duration, true)
            .await
            .map_err(|_| BdbStatus::SteeringFailure)?;

        self.attributes.commissioning_status = crate::attributes::BdbCommissioningStatus::Success;

        log::info!("[BDB:Steering] Permit joining opened for {}s", duration,);
        Ok(())
    }
}
