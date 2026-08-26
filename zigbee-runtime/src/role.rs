//! Typed logical device roles for [`ZigbeeDevice`].
//!
//! The Zigbee logical role a product plays — a leaf **end device**, a
//! forwarding-only **relay router**, or a child-accepting **router**/parent —
//! is expressed here as a Rust *type* rather than the Cargo `router` feature
//! alone. The Cargo feature still gates table capacities and the additive
//! routing/parent code paths during this migration; the role type adds a
//! compile-time capability boundary on top of it and, via
//! [`DeviceRole::run_role_nwk_maintenance`], selects the periodic maintenance a
//! role runs by *static dispatch* so a leaf/relay monomorphization never
//! materializes the child-serving futures.
//!
//! # Why a type and not just a feature
//!
//! A MAC backend that cannot accept children (see
//! [`zigbee_mac::ParentMacDriver`]) must not be able to *construct or
//! advertise* a router. Router and relay construction is additionally gated on
//! the `router` Cargo feature. Encoding the role as a type lets router
//! construction be bounded on a genuine parent MAC, and lets parent-only
//! operational APIs live in trait-bounded impl blocks so an end device never
//! exposes them as success-shaped no-ops.
//!
//! # The three roles
//!
//! | role | [`CAN_ROUTE`] | [`IS_PARENT`] | builder | MAC bound |
//! |------|---------------|---------------|---------|-----------|
//! | [`EndDevice`]   | `false` | `false` | [`build`]        | [`MacDriver`] |
//! | [`RelayRouter`] | `true`  | `false` | [`build_relay`]  | [`ParentMacDriver`] + `router` |
//! | [`Router`]      | `true`  | `true`  | [`build_router`] | [`ParentMacDriver`] + `router` |
//!
//! A [`RelayRouter`] forwards NWK traffic (it is an FFD that relays and runs
//! link-status / route maintenance) but does not retain child lifecycle state.
//! It still requires a `router` build and [`ParentMacDriver`] to advertise the
//! Zigbee Router device type; a non-parent backend must use [`EndDevice`].
//!
//! # Source compatibility
//!
//! [`EndDevice`] is the default role parameter, so existing `ZigbeeDevice<M>`
//! source keeps resolving to an end-device instance unchanged.
//!
//! [`CAN_ROUTE`]: DeviceRole::CAN_ROUTE
//! [`IS_PARENT`]: DeviceRole::IS_PARENT
//! [`build`]: crate::builder::DeviceBuilder::build
//! [`build_relay`]: crate::builder::DeviceBuilder::build_relay
//! [`build_router`]: crate::builder::DeviceBuilder::build_router
//! [`MacDriver`]: zigbee_mac::MacDriver
//! [`ParentMacDriver`]: zigbee_mac::ParentMacDriver

use zigbee_mac::MacDriver;

use crate::ZigbeeDevice;

mod sealed {
    /// Prevents downstream crates from inventing new roles, keeping the role
    /// set (and the invariants the runtime relies on) closed.
    pub trait Sealed {}
}

/// Per-role runtime state stored *inline* in a [`ZigbeeDevice`].
///
/// This is the mechanism that keeps each role's runtime RAM (and code) off the
/// other roles: each role names its own [`DeviceRole::State`], and the device
/// holds exactly one value of it. The three roles select three distinct states:
/// a [`RelayRouter`] names the zero-sized [`NonParentState`] (its `role_state`
/// field occupies no bytes); an [`EndDevice`] names [`EndDeviceState`], which
/// owns the R22 End Device Timeout *client* lifecycle; and a [`Router`] names
/// [`ParentState`], which owns the deferred Trust Center Update-Device queue and
/// the pending Parent Announce flag.
///
/// The trait is sealed (via [`DeviceRole`]'s sealed set — only the state types
/// declared here implement it), so downstream crates cannot invent a state
/// type and the RAM footprint of every role is fixed by this crate.
pub trait RoleState: sealed::Sealed + Sized {
    /// Construct the role's fresh, unjoined runtime state.
    fn new() -> Self;
}

/// Zero-sized runtime state for the forwarding-only [`RelayRouter`] role.
///
/// Holds none of the parent-only data (no deferred child-update queue, no
/// Parent Announce flag) and none of the end-device client lifecycle either, so
/// a relay `ZigbeeDevice` carries zero bytes of role runtime state — the
/// leanest of the three roles. An [`EndDevice`] instead selects
/// [`EndDeviceState`] (the End Device Timeout client) and a [`Router`] selects
/// [`ParentState`] (the parent/server state).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct NonParentState;

impl sealed::Sealed for NonParentState {}
impl RoleState for NonParentState {
    #[inline]
    fn new() -> Self {
        NonParentState
    }
}

/// Runtime state owned exclusively by an [`EndDevice`] role: the R22 End Device
/// Timeout **client** lifecycle.
///
/// This is the mechanism that keeps the client lifecycle RAM (and code) off a
/// router/relay device. The whole client lifecycle payload —
/// [`EndDeviceTimeoutState`](crate::EndDeviceTimeoutState), which tracks the
/// keepalive countdown, the outstanding-response wait, the retransmission
/// budget, the bounded failure counter and the forced-poll flag — lives inside
/// this state, and *only* an [`EndDevice`] names it as its
/// [`DeviceRole::State`]. A [`RelayRouter`] selects the zero-sized
/// [`NonParentState`] and a [`Router`] selects [`ParentState`], so neither
/// carries the client timeout state, and — because the client lifecycle methods
/// are bounded on [`EndDeviceRole`] and dispatched statically — neither links
/// the client lifecycle code either.
///
/// Reached only through [`EndDeviceRole::ed_timeout`]/
/// [`EndDeviceRole::ed_timeout_mut`], giving the client helpers safe,
/// `unsafe`-free access to the concrete state through the role type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EndDeviceState {
    /// R22 End Device Timeout client lifecycle timers/counters.
    pub(crate) end_device_timeout: crate::EndDeviceTimeoutState,
}

impl sealed::Sealed for EndDeviceState {}
impl RoleState for EndDeviceState {
    #[inline]
    fn new() -> Self {
        EndDeviceState {
            end_device_timeout: crate::EndDeviceTimeoutState::new(),
        }
    }
}

/// Runtime state owned exclusively by a [`Router`]/parent role.
///
/// Contains precisely the parent-only fields that previously lived on every
/// `ZigbeeDevice` monomorphization:
/// - the bounded queue of deferred Trust Center Update-Device notifications
///   awaiting an indirect Rejoin Response delivery, and
/// - the fingerprint of the last child table committed to durable storage,
///   which is what makes child-table persistence self-tracking.
///
/// A non-parent role never allocates any of this because its
/// [`DeviceRole::State`] is [`NonParentState`] (relay) or [`EndDeviceState`]
/// (end device) instead. Only reachable through
/// [`ParentRole::parent_state`]/[`ParentRole::parent_state_mut`], so parent
/// helpers get safe, `unsafe`-free access to the concrete state.
pub struct ParentState {
    /// Trust Center notifications deferred until an indirect Rejoin Response is
    /// actually transmitted and acknowledged.
    pub(crate) pending_child_updates: heapless::Vec<crate::PendingChildUpdate, 8>,
    /// Fingerprint of the authenticated child table as last committed to the
    /// product's durable [`ChildTableStore`](crate::child_store::ChildTableStore),
    /// and whether any snapshot has been committed at all this power cycle.
    ///
    /// Compared against
    /// [`NwkLayer::child_table_fingerprint`](zigbee_nwk::NwkLayer::child_table_fingerprint)
    /// so admissions, evictions, address changes and restores all mark the
    /// table dirty without instrumenting each call site.
    pub(crate) persisted_child_fingerprint: u32,
    pub(crate) child_table_persisted: bool,
}

impl sealed::Sealed for ParentState {}
impl RoleState for ParentState {
    #[inline]
    fn new() -> Self {
        ParentState {
            pending_child_updates: heapless::Vec::new(),
            persisted_child_fingerprint: 0,
            child_table_persisted: false,
        }
    }
}

/// A logical Zigbee device role selected at compile time.
///
/// Implemented only by [`EndDevice`], [`RelayRouter`] and [`Router`]. The
/// associated constants let role-generic code observe the role without
/// branching on a feature, and [`run_role_nwk_maintenance`] statically
/// dispatches the periodic NWK maintenance so each role's `tick` future only
/// contains the work that role actually performs.
///
/// [`run_role_nwk_maintenance`]: DeviceRole::run_role_nwk_maintenance
pub trait DeviceRole: sealed::Sealed + Sized {
    /// Whether this role accepts and serves children (router/coordinator side).
    const IS_PARENT: bool;
    /// Whether this role relays NWK traffic and runs router maintenance.
    ///
    /// `true` for both [`RelayRouter`] and [`Router`]; a [`RelayRouter`] routes
    /// without parenting.
    const CAN_ROUTE: bool;
    /// Human-readable role name for diagnostics.
    const NAME: &'static str;

    /// Per-role runtime state stored inline in the device (see [`RoleState`]).
    ///
    /// Each role selects a distinct state: a [`RelayRouter`] the zero-sized
    /// [`NonParentState`], an [`EndDevice`] the client [`EndDeviceState`], and a
    /// [`Router`] the [`ParentState`]. This is what keeps each role's runtime
    /// RAM off the others instead of carrying every field on every
    /// monomorphization.
    type State: RoleState;

    /// Run the role-specific periodic NWK maintenance from the joined tick.
    ///
    /// Common (role-independent) maintenance runs before this in the event
    /// loop; this hook adds only what the role owns:
    /// - [`EndDevice`]: nothing (the common path ages its neighbour cache).
    /// - [`RelayRouter`]: permit-join expiry, router / link-status / route-table
    ///   / concentrator maintenance and pending routing transmission — but
    ///   **no** child aging, MAC parent-command servicing or Parent Announce.
    /// - [`Router`]: the full parent maintenance sequence in the pre-split
    ///   order, followed by a due Parent Announce.
    ///
    /// Because each role provides its own body, a non-parent monomorphization's
    /// `tick` future never contains the child-serving futures — the split is by
    /// static dispatch, not a runtime `if R::IS_PARENT`.
    #[doc(hidden)]
    fn run_role_nwk_maintenance<M: MacDriver>(
        device: &mut ZigbeeDevice<M, Self>,
        elapsed_secs: u16,
    ) -> impl core::future::Future<Output = ()>;

    /// Apply the runtime policy for a *parent-only* NWK command outcome
    /// (child Rejoin Request, End Device Timeout Request).
    ///
    /// Dispatched statically from `handle_nwk_command_outcome` so a
    /// non-parent role's receive path never materializes the child
    /// rejoin / Update-Device / End Device Timeout *server* futures — closing
    /// the hole where a [`RelayRouter`] (whose `NwkLayer::can_route` is `true`)
    /// could otherwise answer a parent-only outcome. [`EndDevice`] and
    /// [`RelayRouter`] ignore the outcome entirely; only [`Router`] acts on it.
    #[doc(hidden)]
    fn service_parent_nwk_outcome<M: MacDriver>(
        device: &mut ZigbeeDevice<M, Self>,
        outcome: crate::ParentNwkOutcome,
    ) -> impl core::future::Future<Output = ()>;

    /// Interleave bounded MAC parent-command servicing around a receive window.
    ///
    /// Dispatched statically from the receive path so only a [`Router`]
    /// monomorphization links (and runs) MAC parent-command servicing;
    /// [`EndDevice`] and [`RelayRouter`] are inert no-ops that materialize no
    /// parent future.
    #[doc(hidden)]
    fn run_role_parent_servicing<M: MacDriver>(
        device: &mut ZigbeeDevice<M, Self>,
    ) -> impl core::future::Future<Output = ()>;

    // ── R22 End Device Timeout CLIENT lifecycle static dispatch ──────────────
    //
    // The client lifecycle (one request after a real join/rejoin, persisted
    // resume keepalive selection, forced polls, bounded response/retry/
    // keepalive/rejoin recovery, and accepted/refused processing) is owned
    // exclusively by an [`EndDevice`]: its state lives in [`EndDeviceState`]
    // and its methods are bounded on [`EndDeviceRole`]. Generic join/rejoin/
    // resume/tick/receive call sites reach it *only* through these hooks, so a
    // [`RelayRouter`] or [`Router`] monomorphization never materializes the
    // client futures nor references the (absent) client state — the split is by
    // static dispatch, exactly like the parent hooks above.

    /// Begin a fresh End Device Timeout negotiation after a real join / secured
    /// rejoin. [`EndDevice`] sends exactly one initial request; the other roles
    /// are inert.
    #[doc(hidden)]
    fn ed_begin_negotiation<M: MacDriver>(
        device: &mut ZigbeeDevice<M, Self>,
    ) -> impl core::future::Future<Output = ()>;

    /// Choose the first keepalive after a silent persisted resume. Inert on the
    /// non-end-device roles.
    #[doc(hidden)]
    fn ed_resume<M: MacDriver>(
        device: &mut ZigbeeDevice<M, Self>,
    ) -> impl core::future::Future<Output = ()>;

    /// Age the client lifecycle timers by one tick. Inert on the non-end-device
    /// roles (they hold no client timers).
    #[doc(hidden)]
    fn ed_advance_timers<M: MacDriver>(device: &mut ZigbeeDevice<M, Self>, elapsed_secs: u16);

    /// Run the due client lifecycle work for this tick (response timeout /
    /// keepalive). Inert on the non-end-device roles.
    #[doc(hidden)]
    fn ed_service<M: MacDriver>(
        device: &mut ZigbeeDevice<M, Self>,
    ) -> impl core::future::Future<Output = ()>;

    /// Consume a client forced-poll request. Always `false` for the
    /// non-end-device roles, which never schedule a keepalive poll.
    #[doc(hidden)]
    fn ed_take_forced_poll<M: MacDriver>(device: &mut ZigbeeDevice<M, Self>) -> bool;

    /// Feed the outcome of a forced keepalive poll into the client's bounded
    /// failure counter. Inert on the non-end-device roles.
    #[doc(hidden)]
    fn ed_note_forced_poll_result<M: MacDriver>(device: &mut ZigbeeDevice<M, Self>, success: bool);

    /// Note a completed MAC poll for the client keepalive deadline. Inert on the
    /// non-end-device roles.
    #[doc(hidden)]
    fn ed_note_poll<M: MacDriver>(device: &mut ZigbeeDevice<M, Self>);

    /// Apply the client-side effect of an accepted/refused End Device Timeout
    /// Response detected around NWK receive processing. Inert on the
    /// non-end-device roles.
    #[doc(hidden)]
    fn ed_apply_timeout_change<M: MacDriver>(
        device: &mut ZigbeeDevice<M, Self>,
        before: crate::EndDeviceTimeoutSnapshot,
    ) -> impl core::future::Future<Output = ()>;

    /// Reset the client lifecycle to its fresh, unjoined state (Leave / factory
    /// reset). Inert on the non-end-device roles.
    #[doc(hidden)]
    fn ed_reset<M: MacDriver>(device: &mut ZigbeeDevice<M, Self>);
}

/// A role that relays NWK traffic (an FFD). Implemented by [`RelayRouter`] and
/// [`Router`]. Bounds routing-only operational APIs that a leaf end device must
/// not expose.
pub trait RoutingRole: DeviceRole {}

/// A role that accepts children and therefore requires parent MAC support.
///
/// Used as the bound for parent-only inherent impl blocks so those APIs are
/// only present on a router-typed device. Implemented only by [`Router`].
///
/// It also provides safe, `unsafe`-free access to the concrete
/// [`ParentState`]: because [`Router::State`](DeviceRole::State) *is*
/// [`ParentState`], the accessors are the identity function, letting
/// parent-only helpers reach the parent runtime state through the role type
/// without a downcast.
pub trait ParentRole: RoutingRole<State = ParentState> {
    /// Shared access to the concrete parent runtime state.
    fn parent_state(state: &Self::State) -> &ParentState;
    /// Exclusive access to the concrete parent runtime state.
    fn parent_state_mut(state: &mut Self::State) -> &mut ParentState;
}

/// A role that runs the R22 End Device Timeout **client** lifecycle.
///
/// Implemented only by [`EndDevice`]. Used as the bound for the client
/// lifecycle inherent methods so they are present (and monomorphized) *only*
/// for an end-device-typed device — a [`RelayRouter`] or [`Router`] neither
/// exposes nor links them.
///
/// Like [`ParentRole`], it provides safe, `unsafe`-free access to the concrete
/// role state: because [`EndDevice::State`](DeviceRole::State) *is*
/// [`EndDeviceState`], the accessors are the identity projection onto the
/// contained [`EndDeviceTimeoutState`](crate::EndDeviceTimeoutState), letting
/// the client helpers reach the timeout state through the role type without a
/// downcast or an `Option`.
pub trait EndDeviceRole: DeviceRole<State = EndDeviceState> {
    /// Shared access to the client End Device Timeout state.
    fn ed_timeout(state: &Self::State) -> &crate::EndDeviceTimeoutState;
    /// Exclusive access to the client End Device Timeout state.
    fn ed_timeout_mut(state: &mut Self::State) -> &mut crate::EndDeviceTimeoutState;
}

/// Leaf end-device role (the default): joins a parent, never accepts children.
///
/// An end device never exposes parent operational APIs (permit-join, Parent
/// Announce transmit, MAC parent-command servicing, child-table persistence) —
/// those live behind [`ParentRole`].
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct EndDevice;

impl sealed::Sealed for EndDevice {}
impl DeviceRole for EndDevice {
    const IS_PARENT: bool = false;
    const CAN_ROUTE: bool = false;
    const NAME: &'static str = "end-device";
    type State = EndDeviceState;

    #[inline]
    async fn run_role_nwk_maintenance<M: MacDriver>(
        device: &mut ZigbeeDevice<M, Self>,
        elapsed_secs: u16,
    ) {
        // A leaf end device owns no router/parent maintenance; it only ages its
        // small neighbour cache so LRU eviction stays meaningful.
        device.run_end_device_nwk_maintenance(elapsed_secs);
    }

    #[inline]
    async fn service_parent_nwk_outcome<M: MacDriver>(
        device: &mut ZigbeeDevice<M, Self>,
        outcome: crate::ParentNwkOutcome,
    ) {
        // A leaf end device never answers a parent-only NWK outcome.
        let _ = (device, outcome);
    }

    #[inline]
    async fn run_role_parent_servicing<M: MacDriver>(device: &mut ZigbeeDevice<M, Self>) {
        // A leaf end device never services MAC parent commands.
        let _ = device;
    }

    // The end device is the sole owner of the client lifecycle: each hook
    // forwards to the `EndDeviceRole`-bounded inherent method. The `async`
    // hooks return the inner future *directly* (rather than `async { … .await }`)
    // so no extra wrapper state machine is generated around the
    // `#[inline(never)]` client helpers.
    #[inline]
    fn ed_begin_negotiation<M: MacDriver>(
        device: &mut ZigbeeDevice<M, Self>,
    ) -> impl core::future::Future<Output = ()> {
        device.begin_end_device_timeout_negotiation()
    }

    #[inline]
    fn ed_resume<M: MacDriver>(
        device: &mut ZigbeeDevice<M, Self>,
    ) -> impl core::future::Future<Output = ()> {
        device.resume_end_device_timeout()
    }

    #[inline]
    fn ed_advance_timers<M: MacDriver>(device: &mut ZigbeeDevice<M, Self>, elapsed_secs: u16) {
        device.advance_end_device_timeout(elapsed_secs);
    }

    #[inline]
    fn ed_service<M: MacDriver>(
        device: &mut ZigbeeDevice<M, Self>,
    ) -> impl core::future::Future<Output = ()> {
        device.service_end_device_timeout()
    }

    #[inline]
    fn ed_take_forced_poll<M: MacDriver>(device: &mut ZigbeeDevice<M, Self>) -> bool {
        device.take_forced_poll()
    }

    #[inline]
    fn ed_note_forced_poll_result<M: MacDriver>(device: &mut ZigbeeDevice<M, Self>, success: bool) {
        if success {
            device.record_end_device_keepalive_success();
        } else {
            device.record_end_device_keepalive_failure();
        }
    }

    #[inline]
    fn ed_note_poll<M: MacDriver>(device: &mut ZigbeeDevice<M, Self>) {
        device.note_end_device_poll();
    }

    #[inline]
    fn ed_apply_timeout_change<M: MacDriver>(
        device: &mut ZigbeeDevice<M, Self>,
        before: crate::EndDeviceTimeoutSnapshot,
    ) -> impl core::future::Future<Output = ()> {
        device.apply_end_device_timeout_change(before)
    }

    #[inline]
    fn ed_reset<M: MacDriver>(device: &mut ZigbeeDevice<M, Self>) {
        device.reset_end_device_timeout_state();
    }
}
impl EndDeviceRole for EndDevice {
    #[inline]
    fn ed_timeout(state: &Self::State) -> &crate::EndDeviceTimeoutState {
        &state.end_device_timeout
    }

    #[inline]
    fn ed_timeout_mut(state: &mut Self::State) -> &mut crate::EndDeviceTimeoutState {
        &mut state.end_device_timeout
    }
}

/// Forwarding-only router role: relays NWK traffic and runs router maintenance,
/// but cannot accept or serve children.
///
/// This role does not retain child lifecycle state, but router construction is
/// still gated on the `router` feature and a
/// [`ParentMacDriver`](zigbee_mac::ParentMacDriver). A MAC lacking those
/// parent primitives must use [`EndDevice`] and cannot advertise
/// `DeviceType::Router`. Constructed with
/// [`build_relay`](crate::builder::DeviceBuilder::build_relay).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct RelayRouter;

impl sealed::Sealed for RelayRouter {}
impl DeviceRole for RelayRouter {
    const IS_PARENT: bool = false;
    const CAN_ROUTE: bool = true;
    const NAME: &'static str = "relay-router";
    type State = NonParentState;

    #[inline]
    async fn run_role_nwk_maintenance<M: MacDriver>(
        device: &mut ZigbeeDevice<M, Self>,
        elapsed_secs: u16,
    ) {
        // Routing-only maintenance: permit-join expiry, router / link-status /
        // route-table / concentrator maintenance and pending routing TX. No
        // child aging, parent-command servicing or Parent Announce.
        #[cfg(feature = "router")]
        device.run_relay_nwk_maintenance(elapsed_secs).await;
        #[cfg(not(feature = "router"))]
        let _ = (device, elapsed_secs);
    }

    #[inline]
    async fn service_parent_nwk_outcome<M: MacDriver>(
        device: &mut ZigbeeDevice<M, Self>,
        outcome: crate::ParentNwkOutcome,
    ) {
        // A relay routes but is not a parent: even though `NwkLayer::can_route`
        // is true, it must not answer a child Rejoin Request or serve End
        // Device Timeout — that is the correctness hole this dispatch closes.
        let _ = (device, outcome);
    }

    #[inline]
    async fn run_role_parent_servicing<M: MacDriver>(device: &mut ZigbeeDevice<M, Self>) {
        // A relay never services MAC parent commands.
        let _ = device;
    }

    // A relay never negotiates the End Device Timeout client lifecycle: it holds
    // no client state ([`NonParentState`]) and materializes no client future.
    // The `async` hooks return a zero-sized ready future so no state machine is
    // generated for a relay monomorphization.
    #[inline]
    fn ed_begin_negotiation<M: MacDriver>(
        device: &mut ZigbeeDevice<M, Self>,
    ) -> impl core::future::Future<Output = ()> {
        let _ = device;
        core::future::ready(())
    }
    #[inline]
    fn ed_resume<M: MacDriver>(
        device: &mut ZigbeeDevice<M, Self>,
    ) -> impl core::future::Future<Output = ()> {
        let _ = device;
        core::future::ready(())
    }
    #[inline]
    fn ed_advance_timers<M: MacDriver>(device: &mut ZigbeeDevice<M, Self>, elapsed_secs: u16) {
        let _ = (device, elapsed_secs);
    }
    #[inline]
    fn ed_service<M: MacDriver>(
        device: &mut ZigbeeDevice<M, Self>,
    ) -> impl core::future::Future<Output = ()> {
        let _ = device;
        core::future::ready(())
    }
    #[inline]
    fn ed_take_forced_poll<M: MacDriver>(device: &mut ZigbeeDevice<M, Self>) -> bool {
        let _ = device;
        false
    }
    #[inline]
    fn ed_note_forced_poll_result<M: MacDriver>(device: &mut ZigbeeDevice<M, Self>, success: bool) {
        let _ = (device, success);
    }
    #[inline]
    fn ed_note_poll<M: MacDriver>(device: &mut ZigbeeDevice<M, Self>) {
        let _ = device;
    }
    #[inline]
    fn ed_apply_timeout_change<M: MacDriver>(
        device: &mut ZigbeeDevice<M, Self>,
        before: crate::EndDeviceTimeoutSnapshot,
    ) -> impl core::future::Future<Output = ()> {
        let _ = (device, before);
        core::future::ready(())
    }
    #[inline]
    fn ed_reset<M: MacDriver>(device: &mut ZigbeeDevice<M, Self>) {
        let _ = device;
    }
}
impl RoutingRole for RelayRouter {}

/// Router/parent role: accepts children and serves them.
///
/// A router-typed device can only be constructed from a
/// [`zigbee_mac::ParentMacDriver`] MAC backend, so the logical role and the
/// physical parent capability cannot disagree.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Router;

impl sealed::Sealed for Router {}
impl DeviceRole for Router {
    const IS_PARENT: bool = true;
    const CAN_ROUTE: bool = true;
    const NAME: &'static str = "router";
    type State = ParentState;

    #[inline]
    async fn run_role_nwk_maintenance<M: MacDriver>(
        device: &mut ZigbeeDevice<M, Self>,
        elapsed_secs: u16,
    ) {
        // Full parent maintenance in the pre-split order, then a due Parent
        // Announce — matching the historical sequence exactly.
        #[cfg(feature = "router")]
        {
            device.run_parent_nwk_maintenance(elapsed_secs).await;
            device.service_due_parent_annce(elapsed_secs).await;
        }
        #[cfg(not(feature = "router"))]
        let _ = (device, elapsed_secs);
    }

    #[inline]
    async fn service_parent_nwk_outcome<M: MacDriver>(
        device: &mut ZigbeeDevice<M, Self>,
        outcome: crate::ParentNwkOutcome,
    ) {
        // Only a router acts on a parent-only NWK outcome: answer the child
        // Rejoin Request (with the coupled Update-Device) or transmit the 0x0C
        // End Device Timeout Response.
        #[cfg(feature = "router")]
        device.dispatch_parent_nwk_outcome(outcome).await;
        #[cfg(not(feature = "router"))]
        let _ = (device, outcome);
    }

    #[inline]
    async fn run_role_parent_servicing<M: MacDriver>(device: &mut ZigbeeDevice<M, Self>) {
        // A router interleaves bounded MAC parent-command servicing; the inner
        // helper self-gates on `parent_mode_active`, so it is inert until the
        // device is a joined, child-capable parent.
        #[cfg(feature = "router")]
        {
            let _ = device.service_parent_commands_inner().await;
        }
        #[cfg(not(feature = "router"))]
        let _ = device;
    }

    // A router runs the parent/server End Device Timeout path (child aging and
    // the 0x0C Response — see the parent hooks/impls above), never the CLIENT
    // lifecycle: it holds [`ParentState`], not the client state, and links no
    // client future. The `async` hooks return a zero-sized ready future so no
    // state machine is generated for a router monomorphization.
    #[inline]
    fn ed_begin_negotiation<M: MacDriver>(
        device: &mut ZigbeeDevice<M, Self>,
    ) -> impl core::future::Future<Output = ()> {
        let _ = device;
        core::future::ready(())
    }
    #[inline]
    fn ed_resume<M: MacDriver>(
        device: &mut ZigbeeDevice<M, Self>,
    ) -> impl core::future::Future<Output = ()> {
        let _ = device;
        core::future::ready(())
    }
    #[inline]
    fn ed_advance_timers<M: MacDriver>(device: &mut ZigbeeDevice<M, Self>, elapsed_secs: u16) {
        let _ = (device, elapsed_secs);
    }
    #[inline]
    fn ed_service<M: MacDriver>(
        device: &mut ZigbeeDevice<M, Self>,
    ) -> impl core::future::Future<Output = ()> {
        let _ = device;
        core::future::ready(())
    }
    #[inline]
    fn ed_take_forced_poll<M: MacDriver>(device: &mut ZigbeeDevice<M, Self>) -> bool {
        let _ = device;
        false
    }
    #[inline]
    fn ed_note_forced_poll_result<M: MacDriver>(device: &mut ZigbeeDevice<M, Self>, success: bool) {
        let _ = (device, success);
    }
    #[inline]
    fn ed_note_poll<M: MacDriver>(device: &mut ZigbeeDevice<M, Self>) {
        let _ = device;
    }
    #[inline]
    fn ed_apply_timeout_change<M: MacDriver>(
        device: &mut ZigbeeDevice<M, Self>,
        before: crate::EndDeviceTimeoutSnapshot,
    ) -> impl core::future::Future<Output = ()> {
        let _ = (device, before);
        core::future::ready(())
    }
    #[inline]
    fn ed_reset<M: MacDriver>(device: &mut ZigbeeDevice<M, Self>) {
        let _ = device;
    }
}
impl RoutingRole for Router {}
impl ParentRole for Router {
    #[inline]
    fn parent_state(state: &Self::State) -> &ParentState {
        state
    }

    #[inline]
    fn parent_state_mut(state: &mut Self::State) -> &mut ParentState {
        state
    }
}

/// Alias for the parent role, for products that speak of a "parent" rather than
/// a "router" (e.g. a coordinator-adjacent parent).
pub type Parent = Router;

/// Alias highlighting the forwarding-only nature of a [`RelayRouter`].
pub type RoutingOnly = RelayRouter;
