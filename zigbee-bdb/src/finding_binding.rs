//! Finding & Binding commissioning — EZ-Mode (BDB v3.0.1 spec §§8.4–8.5).
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

use zigbee_aps::apsde::ApsdeDataRequest;
use zigbee_aps::binding::BindingEntry;
use zigbee_aps::{ApsAddress, ApsAddressMode, ApsTxOptions};
use zigbee_mac::MacDriver;
use zigbee_types::ShortAddress;
use zigbee_zcl::ClusterDirection;
use zigbee_zcl::clusters::groups::CMD_ADD_GROUP;
use zigbee_zcl::frame::{ZclFrameHeader, ZclFrameType};
use zigbee_zdo::ZdpStatus;
use zigbee_zdo::descriptors::SimpleDescriptor;
use zigbee_zdo::discovery::{IeeeAddrRsp, SimpleDescRsp};

use crate::attributes::BDB_MIN_COMMISSIONING_TIME;
use crate::{BdbLayer, BdbStatus};

// ── Identify / Groups cluster constants ─────────────────────

/// ZCL Identify cluster ID
const CLUSTER_IDENTIFY: u16 = 0x0003;

/// ZCL Groups cluster ID
const CLUSTER_GROUPS: u16 = 0x0004;

/// Identify Query command ID (cluster-specific, client → server)
const CMD_IDENTIFY_QUERY: u8 = 0x01;

/// Default F&B window (seconds) — spec says minimum 180 s.
const FB_WINDOW_SECONDS: u16 = BDB_MIN_COMMISSIONING_TIME;

/// Bounded response window for the event-driven IEEE_addr_req /
/// Simple_Desc_req pending slots used while processing an identifying
/// target. Not a BDB-defined constant — chosen to match the other 5 s
/// ZDP-class response budgets already used across this stack (see
/// `BDBC_TC_LINK_KEY_EXCHANGE_TIMEOUT_US`), long enough for one APS-acked
/// unicast round trip plus normal processing/queueing delay, but always
/// bounded so a silent target can never wedge the initiator procedure.
const FB_ZDP_RESPONSE_TIMEOUT_US: u32 = 5_000_000;

/// A single (nwk_addr, endpoint) that responded to our Identify Query.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct FbTarget {
    /// NWK short address of the responding node.
    pub(crate) short: ShortAddress,
    /// APS source endpoint that sent the Identify Query Response.
    pub(crate) endpoint: u8,
}

/// Stage of the event-driven F&B initiator descriptor/binding state machine.
///
/// Advances by exactly one bounded action per [`BdbLayer::tick_finding_binding`]
/// call: a single non-blocking cache lookup, a single bounded transmit, or a
/// single non-blocking check of an already-delivered ZDP response. Normal
/// ZDO/ZCL/APS traffic (including the asynchronous `IEEE_addr_rsp` /
/// `Simple_Desc_rsp` themselves, delivered by the runtime via
/// [`zigbee_zdo::ZdoLayer::deliver_client_response`]) keeps flowing between
/// steps.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum FbStage {
    /// No initiator procedure running.
    Idle,
    /// Broadcast sent; collecting Identify Query Responses until
    /// `fb_window_remaining` reaches zero.
    Collecting,
    /// About to resolve `fb_current`'s IEEE address (cache check or request).
    ResolveIeee,
    /// Waiting for an in-flight `IEEE_addr_rsp` on the given pending slot.
    AwaitIeee { slot: usize, sent_at_us: u32 },
    /// IEEE address known; about to send `Simple_Desc_req`.
    RequestSimpleDesc,
    /// Waiting for an in-flight `Simple_Desc_rsp` on the given pending slot.
    AwaitSimpleDesc { slot: usize, sent_at_us: u32 },
}

// ── Initiator ───────────────────────────────────────────────

impl<M: MacDriver> BdbLayer<M> {
    /// Run Finding & Binding as **initiator** on the given local endpoint.
    ///
    /// Broadcasts an Identify Query from `local_endpoint` (using that
    /// endpoint's own application profile) to NWK broadcast `0xFFFF`, then
    /// starts a response collection window. Responses arrive asynchronously
    /// via the runtime's ZCL dispatch into `fb_identify_responses`.
    ///
    /// Call [`BdbLayer::tick_finding_binding`] once per second from the event
    /// loop. Once the window expires, each distinct `(nwk_addr, endpoint)`
    /// that responded is resolved and bound one bounded tick step at a time:
    /// IEEE address (cache or `IEEE_addr_req`), then `Simple_Desc_req`, then
    /// cluster matching / binding creation.
    pub async fn finding_binding_initiator(&mut self, local_endpoint: u8) -> Result<(), BdbStatus> {
        if !self.attributes.node_is_on_a_network {
            self.attributes.commissioning_status =
                crate::attributes::BdbCommissioningStatus::NotOnANetwork;
            return Err(BdbStatus::NotOnNetwork);
        }
        if self.fb_stage != FbStage::Idle {
            self.attributes.commissioning_status =
                crate::attributes::BdbCommissioningStatus::NotPermitted;
            return Err(BdbStatus::NotPermitted);
        }

        // Verify we have a local simple descriptor for this endpoint, and
        // capture its profile ID — the Identify Query must be sent from the
        // actual initiator endpoint using that endpoint's real profile, not
        // a hard-coded endpoint/profile.
        let Some(local_desc) = self.zdo.get_local_descriptor(local_endpoint) else {
            self.attributes.commissioning_status =
                crate::attributes::BdbCommissioningStatus::NotPermitted;
            return Err(BdbStatus::NotPermitted);
        };
        let profile_id = local_desc.profile_id;
        let out_cluster_count = local_desc.output_clusters.len();
        self.attributes.commissioning_status =
            crate::attributes::BdbCommissioningStatus::InProgress;

        log::info!(
            "[BDB:F&B] Initiator start on ep {} (profile=0x{:04X}, out_clusters={})",
            local_endpoint,
            profile_id,
            out_cluster_count,
        );

        // Clear stale responses/targets and send the Identify Query broadcast.
        self.fb_identify_responses.clear();
        self.fb_targets.clear();
        self.fb_current = None;
        self.fb_current_ieee = None;
        self.send_identify_query_broadcast(local_endpoint, profile_id)
            .await?;

        // Start the response collection window.
        self.fb_window_remaining = FB_WINDOW_SECONDS;
        self.fb_initiator_endpoint = local_endpoint;
        self.fb_stage = FbStage::Collecting;

        // Descriptor resolution and binding creation happen one bounded step
        // at a time from tick_finding_binding() once the window expires.
        Ok(())
    }

    /// Advance the F&B initiator state machine by exactly one bounded step.
    /// Call once per second (elapsed_secs granularity is only used for the
    /// response-collection window; per-request timeouts use the monotonic
    /// clock) from the event loop.
    ///
    /// Returns `true` if the F&B procedure just reached a terminal state
    /// (success, no response, or binding table full) this call.
    pub async fn tick_finding_binding(&mut self, elapsed_secs: u16) -> bool {
        match self.fb_stage {
            FbStage::Idle => false,
            FbStage::Collecting => self.tick_collecting(elapsed_secs),
            FbStage::ResolveIeee
            | FbStage::AwaitIeee { .. }
            | FbStage::RequestSimpleDesc
            | FbStage::AwaitSimpleDesc { .. } => self.tick_processing().await,
        }
    }

    /// Collecting-window bounded step: only decrements the countdown and,
    /// once it expires, builds the deduplicated target queue.
    fn tick_collecting(&mut self, elapsed_secs: u16) -> bool {
        self.fb_window_remaining = self.fb_window_remaining.saturating_sub(elapsed_secs);
        if self.fb_window_remaining > 0 {
            return false;
        }

        log::info!(
            "[BDB:F&B] Window expired — {} response(s) collected",
            self.fb_identify_responses.len(),
        );

        // Deduplicate (short, endpoint) pairs — a retransmitted or repeated
        // Identify Query Response must not be processed twice.
        self.fb_targets.clear();
        for &(addr, ep) in &self.fb_identify_responses {
            let short = ShortAddress(addr);
            if !self
                .fb_targets
                .iter()
                .any(|t| t.short == short && t.endpoint == ep)
            {
                let _ = self.fb_targets.push(FbTarget {
                    short,
                    endpoint: ep,
                });
            }
        }
        self.fb_identify_responses.clear();

        if self.fb_targets.is_empty() {
            self.fb_stage = FbStage::Idle;
            self.attributes.commissioning_status =
                crate::attributes::BdbCommissioningStatus::NoIdentifyQueryResponse;
            return true;
        }

        self.advance_to_next_target()
    }

    /// Pop the next queued target (if any) into `fb_current` and arm IEEE
    /// resolution for it; otherwise finish the procedure with SUCCESS.
    ///
    /// At least one Identify Query Response was received to reach this point,
    /// so per BDB v3.0.1 §8.5 the procedure completes with SUCCESS once every
    /// target has been processed — regardless of whether any of them actually
    /// matched a cluster — unless the binding table filled up first (that
    /// path returns `BdbStatus::BindingTableFull` directly and never calls
    /// this function again).
    fn advance_to_next_target(&mut self) -> bool {
        if self.fb_targets.is_empty() {
            self.fb_current = None;
            self.fb_current_ieee = None;
            self.fb_stage = FbStage::Idle;
            self.attributes.commissioning_status =
                crate::attributes::BdbCommissioningStatus::Success;
            return true;
        }
        let target = self.fb_targets.remove(0);
        self.fb_current = Some(target);
        self.fb_current_ieee = None;
        self.fb_stage = FbStage::ResolveIeee;
        false
    }

    /// Abort the whole initiator procedure immediately because the binding
    /// table is full (BDB v3.0.1 §8.5: TABLE_FULL terminates the procedure).
    fn fail_binding_table_full(&mut self) {
        self.fb_stage = FbStage::Idle;
        self.fb_current = None;
        self.fb_current_ieee = None;
        self.fb_targets.clear();
        self.attributes.commissioning_status =
            crate::attributes::BdbCommissioningStatus::BindingTableFull;
    }

    /// One bounded step of IEEE-resolution / Simple_Desc_req / matching for
    /// `fb_current`.
    async fn tick_processing(&mut self) -> bool {
        let Some(target) = self.fb_current else {
            // Defensive: no current target while in a processing stage should
            // not happen, but never spin — return to Idle.
            self.fb_stage = FbStage::Idle;
            return true;
        };

        match self.fb_stage {
            FbStage::ResolveIeee => {
                // Group bindings address the configured group directly and
                // therefore do not depend on the respondent's IEEE address.
                if self.attributes.commissioning_group_id != 0xFFFF {
                    self.fb_stage = FbStage::RequestSimpleDesc;
                    return false;
                }

                // Prefer an already-cached IEEE address (e.g. from a prior
                // Device_annce or neighbor entry) and never fabricate one. A
                // stored `[0; 8]` is not a real address — treat it exactly
                // like "unresolved" rather than trusting it as a binding
                // destination.
                if let Some(ieee) = self.zdo.nwk().find_ieee_by_short(target.short)
                    && ieee != [0u8; 8]
                {
                    self.fb_current_ieee = Some(ieee);
                    self.fb_stage = FbStage::RequestSimpleDesc;
                    return false;
                }
                match self.zdo.start_ieee_addr_req(target.short).await {
                    Ok((slot, _tsn)) => {
                        let now = self.zdo.aps().nwk().mac().monotonic_micros();
                        self.fb_stage = FbStage::AwaitIeee {
                            slot,
                            sent_at_us: now,
                        };
                        false
                    }
                    Err(e) => {
                        log::debug!(
                            "[BDB:F&B] IEEE_addr_req send failed for 0x{:04X}: {:?}",
                            target.short.0,
                            e,
                        );
                        self.advance_to_next_target()
                    }
                }
            }
            FbStage::AwaitIeee { slot, sent_at_us } => {
                if let Some(payload) = self.zdo.take_response(slot) {
                    match IeeeAddrRsp::parse(&payload) {
                        Ok(rsp)
                            if rsp.status == ZdpStatus::Success
                                && rsp.nwk_addr == target.short
                                && rsp.ieee_addr != [0u8; 8] =>
                        {
                            self.fb_current_ieee = Some(rsp.ieee_addr);
                            self.fb_stage = FbStage::RequestSimpleDesc;
                            false
                        }
                        _ => {
                            log::debug!(
                                "[BDB:F&B] IEEE_addr_rsp invalid/mismatched for 0x{:04X}",
                                target.short.0,
                            );
                            self.advance_to_next_target()
                        }
                    }
                } else {
                    let now = self.zdo.aps().nwk().mac().monotonic_micros();
                    if now.wrapping_sub(sent_at_us) >= FB_ZDP_RESPONSE_TIMEOUT_US {
                        self.zdo.cancel_pending(slot);
                        log::debug!(
                            "[BDB:F&B] IEEE_addr_rsp timeout for 0x{:04X}",
                            target.short.0,
                        );
                        self.advance_to_next_target()
                    } else {
                        false
                    }
                }
            }
            FbStage::RequestSimpleDesc => {
                match self
                    .zdo
                    .start_simple_desc_req(target.short, target.endpoint)
                    .await
                {
                    Ok((slot, _tsn)) => {
                        let now = self.zdo.aps().nwk().mac().monotonic_micros();
                        self.fb_stage = FbStage::AwaitSimpleDesc {
                            slot,
                            sent_at_us: now,
                        };
                        false
                    }
                    Err(e) => {
                        log::debug!(
                            "[BDB:F&B] Simple_Desc_req send failed for 0x{:04X} ep {}: {:?}",
                            target.short.0,
                            target.endpoint,
                            e,
                        );
                        self.advance_to_next_target()
                    }
                }
            }
            FbStage::AwaitSimpleDesc { slot, sent_at_us } => {
                if let Some(payload) = self.zdo.take_response(slot) {
                    // Parse and consume the descriptor in this single bounded
                    // step — it is never stored across ticks.
                    if let Ok(rsp) = SimpleDescRsp::parse(&payload)
                        && rsp.status == ZdpStatus::Success
                        && rsp.nwk_addr_of_interest == target.short
                        && let Some(remote_desc) = rsp.simple_descriptor
                        && remote_desc.endpoint == target.endpoint
                    {
                        match self.process_descriptor(target, &remote_desc).await {
                            Ok(count) if count > 0 => {
                                log::info!(
                                    "[BDB:F&B] Created {} binding(s) with 0x{:04X} ep {}",
                                    count,
                                    target.short.0,
                                    target.endpoint,
                                );
                            }
                            Ok(_) => {}
                            Err(BdbStatus::BindingTableFull) => {
                                self.fail_binding_table_full();
                                return true;
                            }
                            Err(_) => {}
                        }
                    } else {
                        log::debug!(
                            "[BDB:F&B] Simple_Desc_rsp invalid/mismatched for 0x{:04X} ep {}",
                            target.short.0,
                            target.endpoint,
                        );
                    }
                    self.advance_to_next_target()
                } else {
                    let now = self.zdo.aps().nwk().mac().monotonic_micros();
                    if now.wrapping_sub(sent_at_us) >= FB_ZDP_RESPONSE_TIMEOUT_US {
                        self.zdo.cancel_pending(slot);
                        log::debug!(
                            "[BDB:F&B] Simple_Desc_rsp timeout for 0x{:04X} ep {}",
                            target.short.0,
                            target.endpoint,
                        );
                        self.advance_to_next_target()
                    } else {
                        false
                    }
                }
            }
            FbStage::Idle | FbStage::Collecting => unreachable!(
                "tick_processing is only invoked from a descriptor/binding processing stage"
            ),
        }
    }

    /// Broadcast Identify Query to find F&B targets.
    ///
    /// BDB v3.0.1 §8.5: sent from the actual initiator endpoint, using that
    /// endpoint's own application profile, to NWK broadcast `0xFFFF` (all
    /// devices — including sleepy end devices — not `0xFFFD`), destination
    /// endpoint `0xFF` (broadcast to all endpoints).
    async fn send_identify_query_broadcast(
        &mut self,
        local_endpoint: u8,
        profile_id: u16,
    ) -> Result<(), BdbStatus> {
        log::debug!(
            "[BDB:F&B] Broadcasting Identify Query from ep {} profile=0x{:04X} (window={}s)",
            local_endpoint,
            profile_id,
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
            dst_address: ApsAddress::Short(ShortAddress(0xFFFF)),
            dst_endpoint: 0xFF,
            profile_id,
            cluster_id: CLUSTER_IDENTIFY,
            src_endpoint: local_endpoint,
            payload: &zcl_frame,
            tx_options: ApsTxOptions {
                use_nwk_key: true,
                ..ApsTxOptions::default()
            },
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

    /// Match clusters against a freshly-received remote descriptor and create
    /// bindings for `fb_current`. The descriptor itself is a stack-local
    /// borrow — never stored in `self`.
    async fn process_descriptor(
        &mut self,
        target: FbTarget,
        remote: &SimpleDescriptor,
    ) -> Result<usize, BdbStatus> {
        let local_ep = self.fb_initiator_endpoint;
        let Some(local_desc) = self.zdo.get_local_descriptor(local_ep) else {
            return Ok(0);
        };
        // Profile must match (or one must be wildcard 0xFFFF).
        if local_desc.profile_id != remote.profile_id
            && local_desc.profile_id != 0xFFFF
            && remote.profile_id != 0xFFFF
        {
            return Ok(0);
        }
        let local_desc = local_desc.clone();

        self.match_and_bind(&local_desc, remote, target.short).await
    }

    /// Cluster matching algorithm (BDB spec §8.5).
    ///
    /// Creates bindings where:
    /// - Our **output** cluster matches their **input** cluster
    /// - Our **input** cluster matches their **output** cluster
    ///
    /// If a group binding is configured (`bdbCommissioningGroupID != 0xFFFF`)
    /// and at least one group binding link was actually created for this
    /// respondent endpoint, sends a unicast Groups `Add Group` command to it
    /// (BDB v3.0.1 §8.5 step 12/13) — never claims group-binding success
    /// without having created an entry.
    async fn match_and_bind(
        &mut self,
        local: &SimpleDescriptor,
        remote: &SimpleDescriptor,
        remote_addr: ShortAddress,
    ) -> Result<usize, BdbStatus> {
        let our_ieee = self.zdo.nwk().nib().ieee_address;
        let mut count = 0;

        if self.attributes.commissioning_group_id == 0xFFFF {
            // A unicast binding SHALL NOT be created without a real IEEE
            // address. Group mode deliberately skips IEEE resolution.
            let Some(remote_ieee) = self.fb_current_ieee else {
                log::warn!(
                    "[BDB:F&B] Refusing to bind 0x{:04X} ep {} without a resolved IEEE address",
                    remote_addr.0,
                    remote.endpoint,
                );
                return Ok(0);
            };

            // Our output clusters → their input clusters (client → server)
            for &out_cluster in &local.output_clusters {
                if remote.input_clusters.contains(&out_cluster) {
                    let entry = BindingEntry::unicast(
                        our_ieee,
                        local.endpoint,
                        out_cluster,
                        remote_ieee,
                        remote.endpoint,
                    );
                    match self.create_binding(&entry) {
                        Ok(true) => count += 1,
                        Ok(false) => {}
                        Err(BdbStatus::BindingTableFull) => {
                            return Err(BdbStatus::BindingTableFull);
                        }
                        Err(_) => {}
                    }
                }
            }

            // Our input clusters → their output clusters (server → client)
            for &in_cluster in &local.input_clusters {
                if remote.output_clusters.contains(&in_cluster) {
                    let entry = BindingEntry::unicast(
                        our_ieee,
                        local.endpoint,
                        in_cluster,
                        remote_ieee,
                        remote.endpoint,
                    );
                    match self.create_binding(&entry) {
                        Ok(true) => count += 1,
                        Ok(false) => {}
                        Err(BdbStatus::BindingTableFull) => {
                            return Err(BdbStatus::BindingTableFull);
                        }
                        Err(_) => {}
                    }
                }
            }
        } else {
            let group_id = self.attributes.commissioning_group_id;
            let mut group_binding_created = false;
            for &out_cluster in &local.output_clusters {
                if remote.input_clusters.contains(&out_cluster) {
                    let entry =
                        BindingEntry::group(our_ieee, local.endpoint, out_cluster, group_id);
                    match self.create_binding(&entry) {
                        Ok(true) => {
                            count += 1;
                            group_binding_created = true;
                        }
                        Ok(false) => {}
                        Err(BdbStatus::BindingTableFull) => {
                            return Err(BdbStatus::BindingTableFull);
                        }
                        Err(_) => {}
                    }
                }
            }
            // Only announce the group once a link was actually created for
            // this respondent endpoint — never report binding success without
            // having created a binding.
            if group_binding_created {
                self.send_add_group(remote_addr, remote.endpoint, remote.profile_id, group_id)
                    .await;
            }
        }

        Ok(count)
    }

    /// Install a binding entry in the local APS binding table. Returns
    /// `Ok(true)` if a new entry was created, `Ok(false)` if it was already
    /// present (not an error — just nothing new to report), or
    /// `Err(BdbStatus::BindingTableFull)` if the table has no room.
    ///
    /// Does **not** send a remote ZDP `Bind_req`: BDB v3.0.1 §8.5 creates
    /// bindings in the initiator's own APS binding table only. A non-normative
    /// remote `Bind_req` naming the initiator as source serves no protocol
    /// purpose here and would leak a ZDO pending-response slot that nothing
    /// ever consumes.
    fn create_binding(&mut self, entry: &BindingEntry) -> Result<bool, BdbStatus> {
        let already_present = self
            .zdo
            .aps()
            .binding_table()
            .find_by_source(&entry.src_addr, entry.src_endpoint, entry.cluster_id)
            .any(|existing| existing.dst == entry.dst);
        if already_present {
            return Ok(false);
        }
        if self.zdo.aps().binding_table().is_full() {
            return Err(BdbStatus::BindingTableFull);
        }

        match self.zdo.aps_mut().binding_table_mut().add(entry.clone()) {
            Ok(()) => {
                log::debug!(
                    "[BDB:F&B] Binding created: ep {} cluster 0x{:04X}",
                    entry.src_endpoint,
                    entry.cluster_id,
                );
                Ok(true)
            }
            Err(_) => Err(BdbStatus::BindingTableFull),
        }
    }

    /// Send a unicast ZCL Groups `Add Group` command to the respondent
    /// endpoint, using that remote descriptor's profile and the real local
    /// initiator endpoint as source (BDB v3.0.1 §8.5).
    async fn send_add_group(
        &mut self,
        dst: ShortAddress,
        dst_endpoint: u8,
        profile_id: u16,
        group_id: u16,
    ) {
        let fc = ZclFrameHeader::build_frame_control(
            ZclFrameType::ClusterSpecific,
            false,
            ClusterDirection::ClientToServer,
            true,
        );
        let seq = self.zdo.next_seq();
        // header(3) + group_id(2) + zero-length group-name string(1)
        let mut zcl_frame = [0u8; 6];
        zcl_frame[0] = fc;
        zcl_frame[1] = seq;
        zcl_frame[2] = CMD_ADD_GROUP.0;
        zcl_frame[3..5].copy_from_slice(&group_id.to_le_bytes());
        zcl_frame[5] = 0; // group name: zero-length string

        let req = ApsdeDataRequest {
            dst_addr_mode: ApsAddressMode::Short,
            dst_address: ApsAddress::Short(dst),
            dst_endpoint,
            profile_id,
            cluster_id: CLUSTER_GROUPS,
            src_endpoint: self.fb_initiator_endpoint,
            payload: &zcl_frame,
            tx_options: ApsTxOptions {
                use_nwk_key: true,
                ack_request: true,
                ..ApsTxOptions::default()
            },
            radius: 0,
            alias_src_addr: None,
            alias_seq: None,
        };

        match self.zdo.aps_mut().apsde_data_request(&req).await {
            Ok(_) => log::debug!(
                "[BDB:F&B] Groups Add Group 0x{:04X} sent to 0x{:04X} ep {}",
                group_id,
                dst.0,
                dst_endpoint,
            ),
            Err(e) => log::warn!(
                "[BDB:F&B] Groups Add Group send to 0x{:04X} ep {} failed: {:?}",
                dst.0,
                dst_endpoint,
                e,
            ),
        }
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
            self.attributes.commissioning_status =
                crate::attributes::BdbCommissioningStatus::NotOnANetwork;
            return Err(BdbStatus::NotOnNetwork);
        }

        // Verify we have a local simple descriptor for this endpoint
        if self.zdo.get_local_descriptor(local_endpoint).is_none() {
            self.attributes.commissioning_status =
                crate::attributes::BdbCommissioningStatus::NotPermitted;
            return Err(BdbStatus::NotPermitted);
        }
        self.attributes.commissioning_status =
            crate::attributes::BdbCommissioningStatus::InProgress;

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

// ── Tests ─────────────────────────────────────────────────────

#[cfg(test)]
#[allow(clippy::items_after_test_module)]
mod tests {
    use super::*;
    use core::future::Future;
    use zigbee_aps::ApsLayer;
    use zigbee_aps::apsde::ApsdeDataIndication;
    use zigbee_aps::binding::BindingDst;
    use zigbee_aps::frames::ApsHeader;
    use zigbee_mac::PlatformServices;
    use zigbee_mac::mock::MockMac;
    use zigbee_nwk::frames::NwkHeader;
    use zigbee_nwk::security::{NwkSecurity, NwkSecurityHeader};
    use zigbee_nwk::{DeviceType, NwkLayer};
    use zigbee_types::IeeeAddress;
    use zigbee_zdo::ZdoLayer;
    use zigbee_zdo::descriptors::SimpleDescriptor;
    use zigbee_zdo::discovery::IeeeAddrRsp;

    fn block_on<F: Future>(future: F) -> F::Output {
        use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};
        fn noop(_: *const ()) {}
        fn clone(_: *const ()) -> RawWaker {
            RawWaker::new(core::ptr::null(), &VTABLE)
        }
        static VTABLE: RawWakerVTable = RawWakerVTable::new(clone, noop, noop, noop);
        let waker = unsafe { Waker::from_raw(RawWaker::new(core::ptr::null(), &VTABLE)) };
        let mut context = Context::from_waker(&waker);
        let mut future = core::pin::pin!(future);
        loop {
            if let Poll::Ready(output) = future.as_mut().poll(&mut context) {
                return output;
            }
        }
    }

    const LOCAL_IEEE: IeeeAddress = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];
    const LOCAL_EP: u8 = 1;
    const TARGET_SHORT: ShortAddress = ShortAddress(0xBEEF);
    const TARGET_EP: u8 = 2;
    const TARGET_IEEE: IeeeAddress = [0xAA; 8];
    const NETWORK_KEY: [u8; 16] = [0x5A; 16];
    const ON_OFF_CLUSTER: u16 = 0x0006;
    const PROFILE_HA: u16 = 0x0104;

    fn fb_ready_bdb() -> BdbLayer<MockMac> {
        fb_ready_bdb_with_output_clusters(&[ON_OFF_CLUSTER])
    }

    fn fb_ready_bdb_with_output_clusters(clusters: &[u16]) -> BdbLayer<MockMac> {
        fb_ready_bdb_with_clusters(&[], clusters)
    }

    fn fb_ready_bdb_with_clusters(
        input_clusters: &[u16],
        output_clusters: &[u16],
    ) -> BdbLayer<MockMac> {
        let mac = MockMac::new(LOCAL_IEEE);
        let mut nwk = NwkLayer::new(mac, DeviceType::Router);
        nwk.set_joined(true);
        nwk.security_mut().set_network_key(NETWORK_KEY, 0);
        {
            let nib = nwk.nib_mut();
            nib.network_address = ShortAddress(0x0001);
            nib.ieee_address = LOCAL_IEEE;
            nib.security_enabled = true;
        }
        let aps = ApsLayer::new(nwk);
        let mut zdo = ZdoLayer::new(aps);
        zdo.set_local_nwk_addr(ShortAddress(0x0001));
        zdo.set_local_ieee_addr(LOCAL_IEEE);

        let mut local_inputs: heapless::Vec<u16, 16> = heapless::Vec::new();
        for &cluster in input_clusters {
            let _ = local_inputs.push(cluster);
        }
        let mut local_outputs: heapless::Vec<u16, 16> = heapless::Vec::new();
        for &cluster in output_clusters {
            let _ = local_outputs.push(cluster);
        }
        let local_desc = SimpleDescriptor {
            endpoint: LOCAL_EP,
            profile_id: PROFILE_HA,
            device_id: 0x0000,
            device_version: 0,
            input_clusters: local_inputs,
            output_clusters: local_outputs,
        };
        zdo.register_endpoint(local_desc)
            .expect("register local endpoint");

        let mut bdb = BdbLayer::new(zdo);
        bdb.attributes_mut().node_is_on_a_network = true;
        bdb
    }

    /// Advance the mock MAC's monotonic clock, as the runtime's own scheduler
    /// would between ticks.
    fn advance_time(bdb: &mut BdbLayer<MockMac>, micros: u32) {
        block_on(
            bdb.zdo_mut()
                .aps_mut()
                .nwk_mut()
                .mac_mut()
                .delay_micros(micros),
        );
    }

    fn remote_desc_with_input(endpoint: u8, profile_id: u16, cluster: u16) -> SimpleDescriptor {
        let mut input_clusters: heapless::Vec<u16, 16> = heapless::Vec::new();
        let _ = input_clusters.push(cluster);
        SimpleDescriptor {
            endpoint,
            profile_id,
            device_id: 0x0000,
            device_version: 0,
            input_clusters,
            output_clusters: heapless::Vec::new(),
        }
    }

    fn remote_desc_with_output(endpoint: u8, profile_id: u16, cluster: u16) -> SimpleDescriptor {
        let mut output_clusters: heapless::Vec<u16, 16> = heapless::Vec::new();
        let _ = output_clusters.push(cluster);
        SimpleDescriptor {
            endpoint,
            profile_id,
            device_id: 0x0000,
            device_version: 0,
            input_clusters: heapless::Vec::new(),
            output_clusters,
        }
    }

    /// Parse the NWK destination, APS header, and trailing ZCL/ZDP payload of
    /// a transmitted frame, returning owned copies so the caller need not
    /// juggle borrows of `bdb`.
    fn parsed_frame(
        bdb: &BdbLayer<MockMac>,
        index: usize,
    ) -> (ShortAddress, ApsHeader, heapless::Vec<u8, 128>) {
        let record = bdb
            .zdo()
            .nwk()
            .mac()
            .tx_history()
            .get(index)
            .expect("expected transmitted frame");
        let bytes = record.payload.as_slice();
        let (nwk_header, nwk_len) =
            NwkHeader::parse(bytes).expect("transmitted frame must contain a NWK header");
        assert!(
            nwk_header.frame_control.security,
            "F&B traffic on an authenticated network must be NWK-secured"
        );
        let (security_header, security_len) =
            NwkSecurityHeader::parse(&bytes[nwk_len..]).expect("NWK auxiliary header");
        let aad_len = nwk_len + security_len;
        let mut aad = [0u8; 64];
        aad[..aad_len].copy_from_slice(&bytes[..aad_len]);
        aad[nwk_len] = (aad[nwk_len] & !0x07) | 0x05;
        let plaintext = NwkSecurity::new()
            .decrypt(
                &aad[..aad_len],
                &bytes[aad_len..],
                &NETWORK_KEY,
                &security_header,
            )
            .expect("outgoing F&B frame must authenticate");
        let (aps_header, aps_len) =
            ApsHeader::parse(&plaintext).expect("transmitted frame must contain an APS header");
        let mut tail: heapless::Vec<u8, 128> = heapless::Vec::new();
        for &b in &plaintext[aps_len..] {
            let _ = tail.push(b);
        }
        (nwk_header.dst_addr, aps_header, tail)
    }

    /// Simulate an asynchronous ZDP response arriving from `src`, exactly as
    /// the runtime would deliver real incoming APS traffic.
    fn deliver_zdp_response(
        bdb: &mut BdbLayer<MockMac>,
        src: ShortAddress,
        cluster: u16,
        tsn: u8,
        body: &[u8],
    ) -> bool {
        let mut payload: heapless::Vec<u8, 128> = heapless::Vec::new();
        let _ = payload.push(tsn);
        for &b in body {
            let _ = payload.push(b);
        }
        let indication = ApsdeDataIndication {
            dst_addr_mode: ApsAddressMode::Short,
            dst_address: ApsAddress::Short(bdb.zdo().local_nwk_addr()),
            dst_endpoint: zigbee_zdo::ZDO_ENDPOINT,
            src_addr_mode: ApsAddressMode::Short,
            src_address: ApsAddress::Short(src),
            src_endpoint: zigbee_zdo::ZDO_ENDPOINT,
            profile_id: zigbee_zdo::ZDP_PROFILE_ID,
            cluster_id: cluster,
            payload: payload.as_slice(),
            aps_counter: 0,
            security_status: false,
            lqi: 200,
        };
        bdb.zdo_mut().deliver_client_response(&indication)
    }

    /// Extract `(slot, sent_at_us)` from an `AwaitIeee`/`AwaitSimpleDesc`
    /// stage, panicking if the machine is not in one of those stages.
    fn await_slot(bdb: &BdbLayer<MockMac>) -> usize {
        match bdb.fb_stage {
            FbStage::AwaitIeee { slot, .. } | FbStage::AwaitSimpleDesc { slot, .. } => slot,
            other => panic!("expected an Await* stage, got {other:?}"),
        }
    }

    /// Give the mock NWK layer a resolvable route to `TARGET_SHORT` without
    /// caching a real IEEE address for it — the neighbor table doubles as
    /// this stack's short→IEEE cache, so a `[0; 8]` placeholder here models a
    /// device that is routable (e.g. a known child) but whose IEEE address
    /// has not actually been learned yet.
    fn make_target_routable_without_cached_ieee(bdb: &mut BdbLayer<MockMac>) {
        bdb.zdo_mut()
            .nwk_mut()
            .update_neighbor_address(TARGET_SHORT, [0u8; 8]);
    }

    #[test]
    fn identify_query_broadcast_uses_nwk_ffff_actual_endpoint_and_profile() {
        let mut bdb = fb_ready_bdb();
        assert_eq!(block_on(bdb.finding_binding_initiator(LOCAL_EP)), Ok(()));
        assert_eq!(bdb.zdo().nwk().mac().tx_history().len(), 1);

        let (dst, aps_header, zcl) = parsed_frame(&bdb, 0);
        assert_eq!(
            dst,
            ShortAddress(0xFFFF),
            "must broadcast to all devices, not 0xFFFD"
        );
        assert_eq!(aps_header.dst_endpoint, Some(0xFF));
        assert_eq!(
            aps_header.src_endpoint,
            Some(LOCAL_EP),
            "must originate from the actual initiator endpoint"
        );
        assert_eq!(
            aps_header.profile_id,
            Some(PROFILE_HA),
            "must use the initiator endpoint's own profile, not a hard-coded one"
        );
        assert_eq!(aps_header.cluster_id, Some(CLUSTER_IDENTIFY));
        assert_eq!(zcl[2], CMD_IDENTIFY_QUERY);
    }

    #[test]
    fn missing_ieee_cache_triggers_ieee_addr_req_before_simple_desc_req() {
        let mut bdb = fb_ready_bdb();
        assert_eq!(block_on(bdb.finding_binding_initiator(LOCAL_EP)), Ok(()));
        make_target_routable_without_cached_ieee(&mut bdb);
        bdb.zdo_mut()
            .aps_mut()
            .nwk_mut()
            .mac_mut()
            .clear_tx_history();

        let _ = bdb.fb_identify_responses.push((TARGET_SHORT.0, TARGET_EP));
        assert!(!block_on(bdb.tick_finding_binding(FB_WINDOW_SECONDS)));
        assert_eq!(bdb.fb_stage, FbStage::ResolveIeee);

        assert!(!block_on(bdb.tick_finding_binding(0)));
        assert!(
            matches!(bdb.fb_stage, FbStage::AwaitIeee { .. }),
            "no cached IEEE address must resolve it before any Simple_Desc_req"
        );

        assert_eq!(bdb.zdo().nwk().mac().tx_history().len(), 1);
        let (dst, aps_header, _zcl) = parsed_frame(&bdb, 0);
        assert_eq!(dst, TARGET_SHORT);
        assert_eq!(
            aps_header.cluster_id,
            Some(zigbee_zdo::IEEE_ADDR_REQ),
            "must request the IEEE address before Simple_Desc_req"
        );

        assert!(
            bdb.zdo().aps().binding_table().is_empty(),
            "no binding may exist before the target's IEEE address is known"
        );

        // The response never arrives: the bounded timeout must cancel the
        // slot and complete the procedure without ever binding to [0;8].
        let slot = await_slot(&bdb);
        advance_time(&mut bdb, FB_ZDP_RESPONSE_TIMEOUT_US);
        assert!(block_on(bdb.tick_finding_binding(0)));
        assert_eq!(bdb.fb_stage, FbStage::Idle);
        assert!(
            bdb.zdo().pending_tsn(slot).is_none(),
            "the timed-out slot must not leak"
        );
        assert!(
            bdb.zdo().aps().binding_table().is_empty(),
            "a missing IEEE address must never fall back to a [0;8] binding"
        );
        assert_eq!(
            bdb.attributes().commissioning_status,
            crate::attributes::BdbCommissioningStatus::Success,
            "at least one Identify Query Response was received, so a descriptor \
             timeout still completes with SUCCESS"
        );
    }

    #[test]
    fn async_ieee_and_simple_desc_responses_create_the_expected_binding() {
        let mut bdb = fb_ready_bdb();
        assert_eq!(block_on(bdb.finding_binding_initiator(LOCAL_EP)), Ok(()));
        make_target_routable_without_cached_ieee(&mut bdb);
        let _ = bdb.fb_identify_responses.push((TARGET_SHORT.0, TARGET_EP));
        assert!(!block_on(bdb.tick_finding_binding(FB_WINDOW_SECONDS)));
        assert!(!block_on(bdb.tick_finding_binding(0)));
        assert!(matches!(bdb.fb_stage, FbStage::AwaitIeee { .. }));

        // Deliver IEEE_addr_rsp asynchronously.
        let ieee_slot = await_slot(&bdb);
        let tsn = bdb
            .zdo()
            .pending_tsn(ieee_slot)
            .expect("pending IEEE_addr_req must be registered");
        let ieee_rsp = IeeeAddrRsp {
            status: ZdpStatus::Success,
            ieee_addr: TARGET_IEEE,
            nwk_addr: TARGET_SHORT,
            num_assoc_dev: 0,
            start_index: 0,
            assoc_dev_list: heapless::Vec::new(),
        };
        let mut buf = [0u8; 32];
        let n = ieee_rsp.serialize(&mut buf).unwrap();
        assert!(deliver_zdp_response(
            &mut bdb,
            TARGET_SHORT,
            zigbee_zdo::IEEE_ADDR_RSP,
            tsn,
            &buf[..n],
        ));

        assert!(!block_on(bdb.tick_finding_binding(0)));
        assert_eq!(bdb.fb_stage, FbStage::RequestSimpleDesc);
        assert!(!block_on(bdb.tick_finding_binding(0)));
        assert!(matches!(bdb.fb_stage, FbStage::AwaitSimpleDesc { .. }));

        let (dst, aps_header, _zcl) = parsed_frame(&bdb, 2);
        assert_eq!(dst, TARGET_SHORT);
        assert_eq!(aps_header.cluster_id, Some(zigbee_zdo::SIMPLE_DESC_REQ));

        // Deliver Simple_Desc_rsp asynchronously.
        let desc_slot = await_slot(&bdb);
        let tsn = bdb
            .zdo()
            .pending_tsn(desc_slot)
            .expect("pending Simple_Desc_req must be registered");
        let remote = remote_desc_with_input(TARGET_EP, PROFILE_HA, ON_OFF_CLUSTER);
        let desc_rsp = zigbee_zdo::discovery::SimpleDescRsp {
            status: ZdpStatus::Success,
            nwk_addr_of_interest: TARGET_SHORT,
            simple_descriptor: Some(remote),
        };
        let mut buf = [0u8; 64];
        let n = desc_rsp.serialize(&mut buf).unwrap();
        assert!(deliver_zdp_response(
            &mut bdb,
            TARGET_SHORT,
            zigbee_zdo::SIMPLE_DESC_RSP,
            tsn,
            &buf[..n],
        ));

        assert!(block_on(bdb.tick_finding_binding(0)));
        assert_eq!(bdb.fb_stage, FbStage::Idle);
        assert_eq!(
            bdb.attributes().commissioning_status,
            crate::attributes::BdbCommissioningStatus::Success
        );

        let entries = bdb.zdo().aps().binding_table().entries();
        assert_eq!(entries.len(), 1);
        assert_eq!(entries[0].src_addr, LOCAL_IEEE);
        assert_eq!(entries[0].src_endpoint, LOCAL_EP);
        assert_eq!(entries[0].cluster_id, ON_OFF_CLUSTER);
        assert_eq!(
            entries[0].dst,
            BindingDst::Unicast {
                dst_addr: TARGET_IEEE,
                dst_endpoint: TARGET_EP,
            }
        );
    }

    #[test]
    fn local_input_matching_remote_output_creates_reverse_unicast_binding() {
        let mut bdb = fb_ready_bdb_with_clusters(&[ON_OFF_CLUSTER], &[]);
        bdb.fb_initiator_endpoint = LOCAL_EP;
        bdb.fb_current_ieee = Some(TARGET_IEEE);
        let target = FbTarget {
            short: TARGET_SHORT,
            endpoint: TARGET_EP,
        };
        let remote = remote_desc_with_output(TARGET_EP, PROFILE_HA, ON_OFF_CLUSTER);

        assert_eq!(block_on(bdb.process_descriptor(target, &remote)), Ok(1),);
        let entries = bdb.zdo().aps().binding_table().entries();
        assert_eq!(entries.len(), 1);
        assert_eq!(
            entries[0].dst,
            BindingDst::Unicast {
                dst_addr: TARGET_IEEE,
                dst_endpoint: TARGET_EP,
            },
        );
        assert_eq!(entries[0].cluster_id, ON_OFF_CLUSTER);
    }

    #[test]
    fn zero_ieee_address_response_is_rejected_without_creating_a_binding() {
        let mut bdb = fb_ready_bdb();
        assert_eq!(block_on(bdb.finding_binding_initiator(LOCAL_EP)), Ok(()));
        make_target_routable_without_cached_ieee(&mut bdb);
        let _ = bdb.fb_identify_responses.push((TARGET_SHORT.0, TARGET_EP));
        assert!(!block_on(bdb.tick_finding_binding(FB_WINDOW_SECONDS)));
        assert!(!block_on(bdb.tick_finding_binding(0)));

        let slot = await_slot(&bdb);
        let tsn = bdb.zdo().pending_tsn(slot).unwrap();
        let response = IeeeAddrRsp {
            status: ZdpStatus::Success,
            ieee_addr: [0u8; 8],
            nwk_addr: TARGET_SHORT,
            num_assoc_dev: 0,
            start_index: 0,
            assoc_dev_list: heapless::Vec::new(),
        };
        let mut body = [0u8; 32];
        let len = response.serialize(&mut body).unwrap();
        assert!(deliver_zdp_response(
            &mut bdb,
            TARGET_SHORT,
            zigbee_zdo::IEEE_ADDR_RSP,
            tsn,
            &body[..len],
        ));

        assert!(block_on(bdb.tick_finding_binding(0)));
        assert!(bdb.zdo().aps().binding_table().is_empty());
        assert_eq!(
            bdb.attributes().commissioning_status,
            crate::attributes::BdbCommissioningStatus::Success,
        );
    }

    #[test]
    fn repeated_identify_responses_for_the_same_target_are_deduplicated() {
        let mut bdb = fb_ready_bdb();
        assert_eq!(block_on(bdb.finding_binding_initiator(LOCAL_EP)), Ok(()));

        let other = ShortAddress(0x9999);
        let _ = bdb.fb_identify_responses.push((TARGET_SHORT.0, TARGET_EP));
        let _ = bdb.fb_identify_responses.push((TARGET_SHORT.0, TARGET_EP));
        let _ = bdb.fb_identify_responses.push((TARGET_SHORT.0, TARGET_EP));
        let _ = bdb.fb_identify_responses.push((other.0, TARGET_EP));

        assert!(!block_on(bdb.tick_finding_binding(FB_WINDOW_SECONDS)));

        // The first unique target is already popped into `fb_current`; only
        // the second distinct target remains queued — the three duplicate
        // reports of the first target must not appear three times.
        assert_eq!(
            bdb.fb_current,
            Some(FbTarget {
                short: TARGET_SHORT,
                endpoint: TARGET_EP,
            })
        );
        assert_eq!(bdb.fb_targets.len(), 1);
        assert_eq!(
            bdb.fb_targets[0],
            FbTarget {
                short: other,
                endpoint: TARGET_EP,
            }
        );
    }

    #[test]
    fn group_binding_sends_groups_add_group_to_the_respondent_endpoint() {
        let mut bdb = fb_ready_bdb();
        const GROUP_ID: u16 = 0x1234;
        bdb.attributes_mut().commissioning_group_id = GROUP_ID;

        assert_eq!(block_on(bdb.finding_binding_initiator(LOCAL_EP)), Ok(()));
        make_target_routable_without_cached_ieee(&mut bdb);
        bdb.zdo_mut()
            .aps_mut()
            .nwk_mut()
            .mac_mut()
            .clear_tx_history();
        let _ = bdb.fb_identify_responses.push((TARGET_SHORT.0, TARGET_EP));
        assert!(!block_on(bdb.tick_finding_binding(FB_WINDOW_SECONDS)));
        assert!(!block_on(bdb.tick_finding_binding(0))); // skip IEEE resolution
        assert!(bdb.zdo().nwk().mac().tx_history().is_empty());
        assert!(!block_on(bdb.tick_finding_binding(0))); // send Simple_Desc_req

        let desc_slot = await_slot(&bdb);
        let tsn = bdb.zdo().pending_tsn(desc_slot).unwrap();
        let remote = remote_desc_with_input(TARGET_EP, PROFILE_HA, ON_OFF_CLUSTER);
        let desc_rsp = zigbee_zdo::discovery::SimpleDescRsp {
            status: ZdpStatus::Success,
            nwk_addr_of_interest: TARGET_SHORT,
            simple_descriptor: Some(remote),
        };
        let mut buf = [0u8; 64];
        let n = desc_rsp.serialize(&mut buf).unwrap();
        deliver_zdp_response(
            &mut bdb,
            TARGET_SHORT,
            zigbee_zdo::SIMPLE_DESC_RSP,
            tsn,
            &buf[..n],
        );
        bdb.zdo_mut()
            .aps_mut()
            .nwk_mut()
            .mac_mut()
            .clear_tx_history();
        assert!(block_on(bdb.tick_finding_binding(0)));

        // A group binding entry must have been created …
        let entries = bdb.zdo().aps().binding_table().entries();
        assert!(
            entries
                .iter()
                .any(|e| e.dst == BindingDst::Group(GROUP_ID) && e.cluster_id == ON_OFF_CLUSTER),
            "expected a group binding entry for group 0x{GROUP_ID:04X}"
        );
        assert_eq!(entries.len(), 1);

        // … and exactly one Groups Add Group command must have been unicast
        // to the respondent endpoint, using its profile and our real
        // initiator endpoint as source.
        assert_eq!(bdb.zdo().nwk().mac().tx_history().len(), 1);
        let (dst, aps_header, zcl) = parsed_frame(&bdb, 0);
        assert_eq!(dst, TARGET_SHORT);
        assert_eq!(aps_header.dst_endpoint, Some(TARGET_EP));
        assert_eq!(aps_header.src_endpoint, Some(LOCAL_EP));
        assert_eq!(aps_header.profile_id, Some(PROFILE_HA));
        assert_eq!(aps_header.cluster_id, Some(CLUSTER_GROUPS));
        assert!(aps_header.frame_control.ack_request);
        assert_eq!(zcl[2], CMD_ADD_GROUP.0);
        assert_eq!(u16::from_le_bytes([zcl[3], zcl[4]]), GROUP_ID);
    }

    #[test]
    fn binding_table_full_terminates_with_binding_table_full() {
        const OVERFLOW_CLUSTER: u16 = 0x00AA;
        let mut bdb = fb_ready_bdb_with_output_clusters(&[ON_OFF_CLUSTER, OVERFLOW_CLUSTER]);
        // Fill the binding table completely with unrelated entries.
        for i in 0..zigbee_aps::binding::MAX_BINDING_ENTRIES as u16 {
            let entry = BindingEntry::unicast(LOCAL_IEEE, LOCAL_EP, i, [0xCC; 8], 9);
            bdb.zdo_mut()
                .aps_mut()
                .binding_table_mut()
                .add(entry)
                .expect("table must accept entries up to capacity");
        }
        assert!(bdb.zdo().aps().binding_table().is_full());

        assert_eq!(block_on(bdb.finding_binding_initiator(LOCAL_EP)), Ok(()));
        make_target_routable_without_cached_ieee(&mut bdb);
        let _ = bdb.fb_identify_responses.push((TARGET_SHORT.0, TARGET_EP));
        assert!(!block_on(bdb.tick_finding_binding(FB_WINDOW_SECONDS)));
        assert!(!block_on(bdb.tick_finding_binding(0))); // send IEEE_addr_req

        let ieee_slot = await_slot(&bdb);
        let tsn = bdb.zdo().pending_tsn(ieee_slot).unwrap();
        let ieee_rsp = IeeeAddrRsp {
            status: ZdpStatus::Success,
            ieee_addr: TARGET_IEEE,
            nwk_addr: TARGET_SHORT,
            num_assoc_dev: 0,
            start_index: 0,
            assoc_dev_list: heapless::Vec::new(),
        };
        let mut buf = [0u8; 32];
        let n = ieee_rsp.serialize(&mut buf).unwrap();
        deliver_zdp_response(
            &mut bdb,
            TARGET_SHORT,
            zigbee_zdo::IEEE_ADDR_RSP,
            tsn,
            &buf[..n],
        );
        assert!(!block_on(bdb.tick_finding_binding(0))); // -> RequestSimpleDesc
        assert!(!block_on(bdb.tick_finding_binding(0))); // send Simple_Desc_req

        let desc_slot = await_slot(&bdb);
        let tsn = bdb.zdo().pending_tsn(desc_slot).unwrap();
        let remote = remote_desc_with_input(TARGET_EP, PROFILE_HA, OVERFLOW_CLUSTER);
        let desc_rsp = zigbee_zdo::discovery::SimpleDescRsp {
            status: ZdpStatus::Success,
            nwk_addr_of_interest: TARGET_SHORT,
            simple_descriptor: Some(remote),
        };
        let mut buf = [0u8; 64];
        let n = desc_rsp.serialize(&mut buf).unwrap();
        deliver_zdp_response(
            &mut bdb,
            TARGET_SHORT,
            zigbee_zdo::SIMPLE_DESC_RSP,
            tsn,
            &buf[..n],
        );

        assert!(block_on(bdb.tick_finding_binding(0)));
        assert_eq!(bdb.fb_stage, FbStage::Idle);
        assert!(bdb.fb_targets.is_empty());
        assert!(bdb.fb_current.is_none());
        assert_eq!(
            bdb.attributes().commissioning_status,
            crate::attributes::BdbCommissioningStatus::BindingTableFull
        );
    }
}
