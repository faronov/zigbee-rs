//! Finding & Binding commissioning — EZ-Mode (BDB v3.0.1 spec §8.5).
//!
//! Finding & Binding (F&B) automatically creates bindings between
//! compatible endpoints on different devices. It uses the Identify
//! cluster to discover targets and ZDP Simple_Desc / Bind_req to
//! match clusters and install bindings.
//!
//! ## Roles
//!
//! ### Initiator (the device that creates bindings)
//! 1. Enter Finding & Binding mode on a local endpoint
//! 2. Broadcast Identify Query to 0xFFFF
//! 3. For each responding (identifying) target:
//!    a. Get `Simple_Desc` for each active endpoint
//!    b. Match client clusters (our output) ↔ server clusters (their input)
//!    c. Create a binding entry for each matching cluster
//! 4. Exit F&B mode after `bdbcMinCommissioningTime` (180 s)
//!
//! ### Target (the device that gets bound TO)
//! 1. Enter Identify mode (LED blink, etc.) on a local endpoint
//! 2. Respond to Identify Query requests
//! 3. Allow initiator to read Simple_Desc and create bindings
//! 4. Exit Identify mode after timeout
//!
//! ## Cluster matching algorithm
//! A binding is created when the initiator's **output** cluster matches
//! the target's **input** cluster (or vice versa), and both endpoints
//! share the same application profile ID.

use heapless::Vec;
use zigbee_aps::apsde::ApsdeDataRequest;
use zigbee_aps::binding::BindingEntry;
use zigbee_aps::{ApsAddress, ApsAddressMode, ApsTxOptions};
use zigbee_mac::MacDriver;
use zigbee_types::ShortAddress;
use zigbee_zcl::ClusterDirection;
use zigbee_zcl::frame::{ZclFrameHeader, ZclFrameType};
use zigbee_zdo::descriptors::SimpleDescriptor;

use crate::attributes::BDB_MIN_COMMISSIONING_TIME;
use crate::{BdbLayer, BdbStatus};

// ── Identify cluster constants ──────────────────────────────

/// ZCL Identify cluster ID
const CLUSTER_IDENTIFY: u16 = 0x0003;

/// Identify Query command ID (cluster-specific, client → server)
const CMD_IDENTIFY_QUERY: u8 = 0x01;

/// Default F&B window (seconds) — spec says minimum 180 s.
const FB_WINDOW_SECONDS: u16 = BDB_MIN_COMMISSIONING_TIME;

/// Identifies a target device that responded to our Identify Query.
#[derive(Debug, Clone)]
struct IdentifyTarget {
    /// NWK short address of the target
    nwk_addr: ShortAddress,
    /// Active endpoints on this target
    endpoints: Vec<u8, 32>,
}

// ── Initiator ───────────────────────────────────────────────

impl<M: MacDriver> BdbLayer<M> {
    /// Run Finding & Binding as **initiator** on the given local endpoint.
    ///
    /// The initiator sends an Identify Query broadcast and starts a response
    /// collection window. Responses arrive asynchronously via the runtime's
    /// ZCL dispatch into `fb_identify_responses`.
    ///
    /// Call [`BdbLayer::tick_finding_binding`] each second from the event loop.
    /// When the window expires, it processes collected responses, reads
    /// simple descriptors, and creates bindings for matching clusters.
    pub async fn finding_binding_initiator(&mut self, local_endpoint: u8) -> Result<(), BdbStatus> {
        if !self.attributes.node_is_on_a_network {
            return Err(BdbStatus::NotOnNetwork);
        }

        // Verify we have a local simple descriptor for this endpoint
        if self.zdo.get_local_descriptor(local_endpoint).is_none() {
            return Err(BdbStatus::NotPermitted);
        }

        let local_desc = self
            .zdo
            .get_local_descriptor(local_endpoint)
            .unwrap()
            .clone();

        log::info!(
            "[BDB:F&B] Initiator start on ep {} (profile=0x{:04X}, out_clusters={})",
            local_endpoint,
            local_desc.profile_id,
            local_desc.output_clusters.len(),
        );

        // Clear stale responses and send Identify Query broadcast
        self.fb_identify_responses.clear();
        self.send_identify_query_broadcast().await?;

        // Start the response collection window
        self.fb_window_remaining = FB_WINDOW_SECONDS;
        self.fb_initiator_endpoint = local_endpoint;

        // The actual binding creation happens when tick_finding_binding()
        // detects the window has expired and calls finalize_finding_binding().
        Ok(())
    }

    /// Tick the F&B response collection window. Call once per second from tick().
    ///
    /// Returns `true` if the F&B procedure just completed (window expired and
    /// responses were processed).
    pub async fn tick_finding_binding(&mut self, elapsed_secs: u16) -> bool {
        if self.fb_window_remaining == 0 {
            return false;
        }

        self.fb_window_remaining = self.fb_window_remaining.saturating_sub(elapsed_secs);

        if self.fb_window_remaining > 0 {
            return false;
        }

        // Window expired — process collected responses
        log::info!(
            "[BDB:F&B] Window expired — {} response(s) collected",
            self.fb_identify_responses.len(),
        );

        if self.fb_identify_responses.is_empty() {
            self.attributes.commissioning_status =
                crate::attributes::BdbCommissioningStatus::NoIdentifyQueryResponse;
            return true;
        }

        // Build target list from collected responses
        let mut targets: Vec<IdentifyTarget, 8> = Vec::new();
        for &(addr, ep) in &self.fb_identify_responses {
            if let Some(t) = targets.iter_mut().find(|t| t.nwk_addr.0 == addr) {
                let _ = t.endpoints.push(ep);
            } else {
                let mut eps = Vec::new();
                let _ = eps.push(ep);
                let _ = targets.push(IdentifyTarget {
                    nwk_addr: ShortAddress(addr),
                    endpoints: eps,
                });
            }
        }

        let local_ep = self.fb_initiator_endpoint;
        let local_desc = match self.zdo.get_local_descriptor(local_ep) {
            Some(d) => d.clone(),
            None => return true,
        };

        let mut any_binding_created = false;

        for target in &targets {
            match self.process_target(target, &local_desc).await {
                Ok(count) if count > 0 => {
                    log::info!(
                        "[BDB:F&B] Created {} binding(s) with 0x{:04X}",
                        count,
                        target.nwk_addr.0,
                    );
                    any_binding_created = true;
                }
                Ok(_) => {
                    log::debug!(
                        "[BDB:F&B] No matching clusters with 0x{:04X}",
                        target.nwk_addr.0,
                    );
                }
                Err(e) => {
                    log::warn!(
                        "[BDB:F&B] Failed to process target 0x{:04X}: {:?}",
                        target.nwk_addr.0,
                        e,
                    );
                }
            }
        }

        self.fb_identify_responses.clear();

        if any_binding_created {
            self.attributes.commissioning_status =
                crate::attributes::BdbCommissioningStatus::Success;
        } else {
            self.attributes.commissioning_status =
                crate::attributes::BdbCommissioningStatus::NoIdentifyQueryResponse;
        }

        true
    }

    /// Broadcast Identify Query to find F&B targets.
    ///
    /// Sends the ZCL broadcast and returns immediately. Responses will be
    /// collected asynchronously into `fb_identify_responses` by the runtime.
    async fn send_identify_query_broadcast(&mut self) -> Result<(), BdbStatus> {
        log::debug!(
            "[BDB:F&B] Broadcasting Identify Query (window={}s)",
            FB_WINDOW_SECONDS,
        );

        // Build ZCL Identify Query frame:
        // Frame control: cluster-specific, client-to-server, disable default response
        let fc = ZclFrameHeader::build_frame_control(
            ZclFrameType::ClusterSpecific,
            false,
            ClusterDirection::ClientToServer,
            true,
        );
        let seq = self.zdo.next_seq();
        let zcl_frame = [fc, seq, CMD_IDENTIFY_QUERY];

        let req = ApsdeDataRequest {
            dst_addr_mode: ApsAddressMode::Short,
            dst_address: ApsAddress::Short(ShortAddress(0xFFFD)),
            dst_endpoint: 0xFF,
            profile_id: 0x0104,
            cluster_id: CLUSTER_IDENTIFY,
            src_endpoint: 0x01,
            payload: &zcl_frame,
            tx_options: ApsTxOptions::default(),
            radius: 0,
            alias_src_addr: None,
            alias_seq: None,
        };

        match self.zdo.aps_mut().apsde_data_request(&req).await {
            Ok(_) => {
                log::debug!("[BDB:F&B] Identify Query broadcast sent");
                Ok(())
            }
            Err(e) => {
                log::warn!("[BDB:F&B] Identify Query broadcast failed: {:?}", e);
                Err(BdbStatus::NotPermitted)
            }
        }
    }

    /// Process a single identifying target: read descriptors, match clusters, bind.
    async fn process_target(
        &mut self,
        target: &IdentifyTarget,
        local_desc: &SimpleDescriptor,
    ) -> Result<usize, BdbStatus> {
        let mut bindings_created = 0;

        for &ep in &target.endpoints {
            // Get the remote simple descriptor
            let remote_desc = match self.zdo.simple_desc_req(target.nwk_addr, ep).await {
                Ok(desc) => desc,
                Err(e) => {
                    log::debug!(
                        "[BDB:F&B] Simple_Desc_req failed for 0x{:04X} ep {}: {:?}",
                        target.nwk_addr.0,
                        ep,
                        e,
                    );
                    continue;
                }
            };

            // Profile must match (or one must be wildcard 0xFFFF)
            if local_desc.profile_id != remote_desc.profile_id
                && local_desc.profile_id != 0xFFFF
                && remote_desc.profile_id != 0xFFFF
            {
                continue;
            }

            // Match clusters and create bindings
            bindings_created += self
                .match_and_bind(local_desc, &remote_desc, target.nwk_addr)
                .await?;
        }

        Ok(bindings_created)
    }

    /// Cluster matching algorithm (BDB spec §8.5).
    ///
    /// Creates bindings where:
    /// - Our **output** cluster matches their **input** cluster
    /// - Our **input** cluster matches their **output** cluster
    async fn match_and_bind(
        &mut self,
        local: &SimpleDescriptor,
        remote: &SimpleDescriptor,
        remote_addr: ShortAddress,
    ) -> Result<usize, BdbStatus> {
        let our_ieee = self.zdo.nwk().nib().ieee_address;
        let remote_ieee = self
            .zdo
            .nwk()
            .find_ieee_by_short(remote_addr)
            .unwrap_or_default();
        let mut count = 0;

        // Our output clusters → their input clusters (client → server binding)
        for &out_cluster in &local.output_clusters {
            if remote.input_clusters.contains(&out_cluster) {
                let entry = BindingEntry::unicast(
                    our_ieee,
                    local.endpoint,
                    out_cluster,
                    remote_ieee,
                    remote.endpoint,
                );
                match self.create_binding(remote_addr, &entry).await {
                    Ok(()) => count += 1,
                    Err(BdbStatus::BindingTableFull) => return Err(BdbStatus::BindingTableFull),
                    Err(_) => {}
                }
            }
        }

        // Our input clusters → their output clusters (server → client binding)
        for &in_cluster in &local.input_clusters {
            if remote.output_clusters.contains(&in_cluster) {
                let entry = BindingEntry::unicast(
                    our_ieee,
                    local.endpoint,
                    in_cluster,
                    remote_ieee,
                    remote.endpoint,
                );
                match self.create_binding(remote_addr, &entry).await {
                    Ok(()) => count += 1,
                    Err(BdbStatus::BindingTableFull) => return Err(BdbStatus::BindingTableFull),
                    Err(_) => {}
                }
            }
        }

        // Group binding (if bdbCommissioningGroupID != 0xFFFF)
        if self.attributes.commissioning_group_id != 0xFFFF {
            for &out_cluster in &local.output_clusters {
                if remote.input_clusters.contains(&out_cluster) {
                    let entry = BindingEntry::group(
                        our_ieee,
                        local.endpoint,
                        out_cluster,
                        self.attributes.commissioning_group_id,
                    );
                    match self.create_binding(remote_addr, &entry).await {
                        Ok(()) => count += 1,
                        Err(BdbStatus::BindingTableFull) => {
                            return Err(BdbStatus::BindingTableFull);
                        }
                        Err(_) => {}
                    }
                }
            }
        }

        Ok(count)
    }

    /// Install a binding in the local APS binding table and send a
    /// ZDP Bind_req to the remote device for bidirectional awareness.
    async fn create_binding(
        &mut self,
        remote_addr: ShortAddress,
        entry: &BindingEntry,
    ) -> Result<(), BdbStatus> {
        // Add to local binding table
        if self
            .zdo
            .aps_mut()
            .binding_table_mut()
            .add(entry.clone())
            .is_err()
            && self.zdo.aps().binding_table().is_full()
        {
            return Err(BdbStatus::BindingTableFull);
        }

        log::debug!(
            "[BDB:F&B] Binding created: ep {} cluster 0x{:04X}",
            entry.src_endpoint,
            entry.cluster_id,
        );

        // Send ZDP Bind_req to remote device (best-effort, don't fail on error)
        if let Err(e) = self.zdo.bind_req(remote_addr, entry).await {
            log::debug!(
                "[BDB:F&B] Remote Bind_req to 0x{:04X} returned {:?} (local binding still valid)",
                remote_addr.0,
                e,
            );
        }

        Ok(())
    }
}

// ── Target ──────────────────────────────────────────────────

impl<M: MacDriver> BdbLayer<M> {
    /// Enter Finding & Binding as **target** on the given local endpoint.
    ///
    /// The target enters Identify mode so that initiators can discover it.
    /// It responds to Identify Query and allows initiators to read its
    /// simple descriptor and create bindings.
    ///
    /// The target stays in Identify mode for [`BDB_MIN_COMMISSIONING_TIME`]
    /// seconds (180 s).
    pub async fn finding_binding_target(&mut self, local_endpoint: u8) -> Result<(), BdbStatus> {
        if !self.attributes.node_is_on_a_network {
            return Err(BdbStatus::NotOnNetwork);
        }

        // Verify we have a local simple descriptor for this endpoint
        if self.zdo.get_local_descriptor(local_endpoint).is_none() {
            return Err(BdbStatus::NotPermitted);
        }

        log::info!(
            "[BDB:F&B] Target mode on ep {} for {}s",
            local_endpoint,
            FB_WINDOW_SECONDS,
        );

        // Request the runtime to set IdentifyTime on the Identify cluster
        // for this endpoint to bdbcMinCommissioningTime (180 s).
        // The runtime reads this and writes the attribute on the next tick.
        self.fb_target_request = Some((local_endpoint, FB_WINDOW_SECONDS));

        // The device's normal APS/ZCL processing handles incoming
        // Simple_Desc_req and Bind_req from the initiator.

        self.attributes.commissioning_status = crate::attributes::BdbCommissioningStatus::Success;
        Ok(())
    }
}
