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
use zigbee_aps::binding::BindingEntry;
use zigbee_mac::MacDriver;
use zigbee_types::ShortAddress;
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
    /// The initiator discovers targets in Identify mode, reads their
    /// simple descriptors, and creates bindings for matching clusters.
    ///
    /// The procedure runs for up to [`BDB_MIN_COMMISSIONING_TIME`] seconds.
    pub async fn finding_binding_initiator(&mut self, local_endpoint: u8) -> Result<(), BdbStatus> {
        if !self.attributes.node_is_on_a_network {
            return Err(BdbStatus::NotOnNetwork);
        }

        // Verify we have a local simple descriptor for this endpoint
        let local_desc = self
            .zdo
            .get_local_descriptor(local_endpoint)
            .ok_or(BdbStatus::NotPermitted)?
            .clone();

        log::info!(
            "[BDB:F&B] Initiator start on ep {} (profile=0x{:04X}, out_clusters={})",
            local_endpoint,
            local_desc.profile_id,
            local_desc.output_clusters.len(),
        );

        // Step 1: Broadcast Identify Query
        let targets = self.send_identify_query().await?;

        if targets.is_empty() {
            log::info!("[BDB:F&B] No identifying targets found");
            self.attributes.commissioning_status =
                crate::attributes::BdbCommissioningStatus::NoIdentifyQueryResponse;
            return Err(BdbStatus::NoIdentifyResponse);
        }

        log::info!("[BDB:F&B] Found {} identifying target(s)", targets.len());

        let mut any_binding_created = false;

        // Step 2–4: For each target, get simple descriptors and create bindings
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

        if any_binding_created {
            self.attributes.commissioning_status =
                crate::attributes::BdbCommissioningStatus::Success;
            Ok(())
        } else {
            self.attributes.commissioning_status =
                crate::attributes::BdbCommissioningStatus::NoIdentifyQueryResponse;
            Err(BdbStatus::NoIdentifyResponse)
        }
    }

    /// Broadcast Identify Query and collect responding targets.
    async fn send_identify_query(&mut self) -> Result<Vec<IdentifyTarget, 8>, BdbStatus> {
        // TODO: Build ZCL Identify Query frame:
        //   Frame control: cluster-specific, client-to-server, disable default response
        //   Seq number
        //   Command ID: 0x01 (Identify Query)
        // Send via APSDE-DATA.request to broadcast (0xFFFF), cluster 0x0003
        //
        // Then collect Identify Query Response frames for up to FB_WINDOW_SECONDS.
        // Each response contains the target's short address.

        log::debug!(
            "[BDB:F&B] Broadcasting Identify Query (window={}s)",
            FB_WINDOW_SECONDS,
        );

        // Placeholder: In a real implementation, we would:
        // 1. Send the broadcast
        // 2. Start a timer for FB_WINDOW_SECONDS
        // 3. Collect responses as they arrive
        // 4. Return the list of targets
        let _ = (CMD_IDENTIFY_QUERY, CLUSTER_IDENTIFY);
        Ok(Vec::new())
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
            bindings_created += self.match_and_bind(local_desc, &remote_desc, target.nwk_addr)?;
        }

        Ok(bindings_created)
    }

    /// Cluster matching algorithm (BDB spec §8.5).
    ///
    /// Creates bindings where:
    /// - Our **output** cluster matches their **input** cluster
    /// - Our **input** cluster matches their **output** cluster
    fn match_and_bind(
        &mut self,
        local: &SimpleDescriptor,
        remote: &SimpleDescriptor,
        remote_addr: ShortAddress,
    ) -> Result<usize, BdbStatus> {
        let our_ieee = self.zdo.nwk().nib().ieee_address;
        let mut count = 0;

        // Our output clusters → their input clusters (client → server binding)
        for &out_cluster in &local.output_clusters {
            if remote.input_clusters.contains(&out_cluster) {
                let entry = BindingEntry::unicast(
                    our_ieee,
                    local.endpoint,
                    out_cluster,
                    // TODO: resolve remote IEEE address from NWK addr
                    [0u8; 8],
                    remote.endpoint,
                );
                match self.create_binding(remote_addr, &entry) {
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
                    [0u8; 8],
                    remote.endpoint,
                );
                match self.create_binding(remote_addr, &entry) {
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
                    match self.create_binding(remote_addr, &entry) {
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

    /// Install a binding in the local APS binding table and optionally
    /// send a ZDP Bind_req to the remote device.
    fn create_binding(
        &mut self,
        _remote_addr: ShortAddress,
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

        // TODO: send ZDP Bind_req to remote device so it also knows about
        // the binding (for bidirectional communication).

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

        // TODO: Set the local Identify cluster's IdentifyTime attribute
        // to bdbcMinCommissioningTime (180 s). This will:
        // 1. Start the identify effect (LED blink, etc.)
        // 2. Cause the device to respond to Identify Query
        // 3. Automatically stop after the timeout

        // The device's normal APS/ZCL processing handles incoming
        // Simple_Desc_req and Bind_req from the initiator.

        self.attributes.commissioning_status = crate::attributes::BdbCommissioningStatus::Success;
        Ok(())
    }
}
