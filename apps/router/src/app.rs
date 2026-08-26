//! Shared router/coordinator lifecycle and compile-time public frontends.

use core::future::Future;
use core::marker::PhantomData;

#[cfg(feature = "router")]
use zigbee_mac::ParentMacDriver;
use zigbee_mac::{MacDriver, MacError};
use zigbee_nwk::DeviceType;
use zigbee_runtime::child_store::ChildStoreError;
#[cfg(feature = "router")]
use zigbee_runtime::child_store::ChildTableStore;
use zigbee_runtime::event_loop::{StackEvent, StartError, TickResult};
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::profile::ApplicationProfile;
use zigbee_runtime::role::{DeviceRole, EndDevice};
#[cfg(feature = "router")]
use zigbee_runtime::role::{RelayRouter, Router};
use zigbee_runtime::security_store::SecurityStateStore;

use crate::capabilities::{NodeArchetype, RouterStatus, StatusSink, Supervisor};
use crate::children::NoChildren;
#[cfg(feature = "router")]
use crate::children::PersistentChildren;
use crate::diagnostics::{DiagnosticEvent, Diagnostics, summarize_stack_event};
use crate::error::RouterAppError;
use crate::observer::{NoObserver, RouterObserver};
use crate::parts::RouterParts;
use crate::policy::RouterPolicy;

type RouterNode<'a, M, S, P, R> = ZigbeeNode<'a, M, S, P, R>;

/// Application events produced during one finite application step.
///
/// At most one event can come from the bounded receive path and at most one
/// from the subsequent runtime tick. Both are returned by value so a plug or
/// light composition root can synchronize fitted hardware from the profile
/// state after the stack has applied the command.
#[derive(Debug, Default)]
pub struct StepEvents {
    pub incoming: Option<StackEvent>,
    pub tick: Option<StackEvent>,
}

impl StepEvents {
    pub const fn is_empty(&self) -> bool {
        self.incoming.is_none() && self.tick.is_none()
    }

    pub fn iter(&self) -> impl Iterator<Item = &StackEvent> {
        self.incoming.iter().chain(self.tick.iter())
    }
}

trait Archetype<R: DeviceRole> {
    const ID: NodeArchetype;
    const DEVICE_TYPE: DeviceType;
}

/// Statically selected startup path for one public frontend.
///
/// This is deliberately narrower than a platform or lifecycle trait: it
/// chooses only the first network transition. The shared steady-state loop
/// remains in [`RouterCore`], while each concrete archetype returns the exact
/// runtime future it is allowed to use.
trait StartupPath<R: DeviceRole>: Archetype<R> {
    fn start<'a, M, S, P>(
        node: &'a mut RouterNode<'_, M, S, P, R>,
    ) -> impl Future<Output = Result<u16, StartError>> + 'a
    where
        M: MacDriver,
        S: SecurityStateStore,
        P: ApplicationProfile;
}

/// Statically selected pending-action tick path for one public frontend.
trait TickPath<R: DeviceRole>: Archetype<R> {
    fn tick<'a, M, S, P>(
        node: &'a mut RouterNode<'_, M, S, P, R>,
        elapsed_secs: u16,
    ) -> impl Future<Output = Result<TickResult, zigbee_runtime::node::NodeError>> + 'a
    where
        M: MacDriver,
        S: SecurityStateStore,
        P: ApplicationProfile;
}

struct AlwaysOnEndDeviceArchetype;
#[cfg(feature = "router")]
struct RelayArchetype;
#[cfg(feature = "router")]
struct ParentArchetype;
#[cfg(feature = "router")]
struct CoordinatorArchetype;

impl Archetype<EndDevice> for AlwaysOnEndDeviceArchetype {
    const ID: NodeArchetype = NodeArchetype::AlwaysOnEndDevice;
    const DEVICE_TYPE: DeviceType = DeviceType::EndDevice;
}

#[cfg(feature = "router")]
impl Archetype<RelayRouter> for RelayArchetype {
    const ID: NodeArchetype = NodeArchetype::RelayRouter;
    const DEVICE_TYPE: DeviceType = DeviceType::Router;
}

#[cfg(feature = "router")]
impl Archetype<Router> for ParentArchetype {
    const ID: NodeArchetype = NodeArchetype::ParentRouter;
    const DEVICE_TYPE: DeviceType = DeviceType::Router;
}

#[cfg(feature = "router")]
impl Archetype<Router> for CoordinatorArchetype {
    const ID: NodeArchetype = NodeArchetype::Coordinator;
    const DEVICE_TYPE: DeviceType = DeviceType::Coordinator;
}

impl StartupPath<EndDevice> for AlwaysOnEndDeviceArchetype {
    fn start<'a, M, S, P>(
        node: &'a mut RouterNode<'_, M, S, P, EndDevice>,
    ) -> impl Future<Output = Result<u16, StartError>> + 'a
    where
        M: MacDriver,
        S: SecurityStateStore,
        P: ApplicationProfile,
    {
        node.start_or_resume_steering()
    }
}

#[cfg(feature = "router")]
impl StartupPath<RelayRouter> for RelayArchetype {
    fn start<'a, M, S, P>(
        node: &'a mut RouterNode<'_, M, S, P, RelayRouter>,
    ) -> impl Future<Output = Result<u16, StartError>> + 'a
    where
        M: MacDriver,
        S: SecurityStateStore,
        P: ApplicationProfile,
    {
        node.start_or_resume_steering()
    }
}

#[cfg(feature = "router")]
impl StartupPath<Router> for ParentArchetype {
    fn start<'a, M, S, P>(
        node: &'a mut RouterNode<'_, M, S, P, Router>,
    ) -> impl Future<Output = Result<u16, StartError>> + 'a
    where
        M: MacDriver,
        S: SecurityStateStore,
        P: ApplicationProfile,
    {
        node.start_or_resume_steering()
    }
}

#[cfg(feature = "router")]
impl StartupPath<Router> for CoordinatorArchetype {
    fn start<'a, M, S, P>(
        node: &'a mut RouterNode<'_, M, S, P, Router>,
    ) -> impl Future<Output = Result<u16, StartError>> + 'a
    where
        M: MacDriver,
        S: SecurityStateStore,
        P: ApplicationProfile,
    {
        node.start_or_resume_coordinator()
    }
}

impl TickPath<EndDevice> for AlwaysOnEndDeviceArchetype {
    fn tick<'a, M, S, P>(
        node: &'a mut RouterNode<'_, M, S, P, EndDevice>,
        elapsed_secs: u16,
    ) -> impl Future<Output = Result<TickResult, zigbee_runtime::node::NodeError>> + 'a
    where
        M: MacDriver,
        S: SecurityStateStore,
        P: ApplicationProfile,
    {
        node.tick_steering_deferred_reset(elapsed_secs)
    }
}

#[cfg(feature = "router")]
impl TickPath<RelayRouter> for RelayArchetype {
    fn tick<'a, M, S, P>(
        node: &'a mut RouterNode<'_, M, S, P, RelayRouter>,
        elapsed_secs: u16,
    ) -> impl Future<Output = Result<TickResult, zigbee_runtime::node::NodeError>> + 'a
    where
        M: MacDriver,
        S: SecurityStateStore,
        P: ApplicationProfile,
    {
        node.tick_steering_deferred_reset(elapsed_secs)
    }
}

#[cfg(feature = "router")]
impl TickPath<Router> for ParentArchetype {
    fn tick<'a, M, S, P>(
        node: &'a mut RouterNode<'_, M, S, P, Router>,
        elapsed_secs: u16,
    ) -> impl Future<Output = Result<TickResult, zigbee_runtime::node::NodeError>> + 'a
    where
        M: MacDriver,
        S: SecurityStateStore,
        P: ApplicationProfile,
    {
        node.tick_steering_deferred_reset(elapsed_secs)
    }
}

#[cfg(feature = "router")]
impl TickPath<Router> for CoordinatorArchetype {
    fn tick<'a, M, S, P>(
        node: &'a mut RouterNode<'_, M, S, P, Router>,
        elapsed_secs: u16,
    ) -> impl Future<Output = Result<TickResult, zigbee_runtime::node::NodeError>> + 'a
    where
        M: MacDriver,
        S: SecurityStateStore,
        P: ApplicationProfile,
    {
        node.tick_coordinator_deferred_reset(elapsed_secs)
    }
}

enum ChildRestore {
    NotApplicable,
    #[cfg(feature = "router")]
    Restored(usize),
    #[cfg(feature = "router")]
    Discarded(ChildStoreError),
}

trait ChildLifecycle<M, S, P, R>
where
    M: MacDriver,
    S: SecurityStateStore,
    P: ApplicationProfile,
    R: DeviceRole,
{
    fn restore(
        &mut self,
        node: &mut RouterNode<'_, M, S, P, R>,
    ) -> Result<ChildRestore, ChildStoreError>;

    fn persist_if_dirty(
        &mut self,
        node: &mut RouterNode<'_, M, S, P, R>,
    ) -> Result<bool, ChildStoreError>;

    fn clear(&mut self, node: &mut RouterNode<'_, M, S, P, R>) -> Result<bool, ChildStoreError>;

    fn clear_stale_before_fresh(
        &mut self,
        node: &mut RouterNode<'_, M, S, P, R>,
    ) -> Result<bool, ChildStoreError>;
}

#[cfg(feature = "router")]
impl<M, S, P> ChildLifecycle<M, S, P, RelayRouter> for NoChildren
where
    M: MacDriver,
    S: SecurityStateStore,
    P: ApplicationProfile,
{
    fn restore(
        &mut self,
        _node: &mut RouterNode<'_, M, S, P, RelayRouter>,
    ) -> Result<ChildRestore, ChildStoreError> {
        Ok(ChildRestore::NotApplicable)
    }

    fn persist_if_dirty(
        &mut self,
        _node: &mut RouterNode<'_, M, S, P, RelayRouter>,
    ) -> Result<bool, ChildStoreError> {
        Ok(false)
    }

    fn clear(
        &mut self,
        _node: &mut RouterNode<'_, M, S, P, RelayRouter>,
    ) -> Result<bool, ChildStoreError> {
        Ok(false)
    }

    fn clear_stale_before_fresh(
        &mut self,
        _node: &mut RouterNode<'_, M, S, P, RelayRouter>,
    ) -> Result<bool, ChildStoreError> {
        Ok(false)
    }
}

impl<M, S, P> ChildLifecycle<M, S, P, EndDevice> for NoChildren
where
    M: MacDriver,
    S: SecurityStateStore,
    P: ApplicationProfile,
{
    fn restore(
        &mut self,
        _node: &mut RouterNode<'_, M, S, P, EndDevice>,
    ) -> Result<ChildRestore, ChildStoreError> {
        Ok(ChildRestore::NotApplicable)
    }

    fn persist_if_dirty(
        &mut self,
        _node: &mut RouterNode<'_, M, S, P, EndDevice>,
    ) -> Result<bool, ChildStoreError> {
        Ok(false)
    }

    fn clear(
        &mut self,
        _node: &mut RouterNode<'_, M, S, P, EndDevice>,
    ) -> Result<bool, ChildStoreError> {
        Ok(false)
    }

    fn clear_stale_before_fresh(
        &mut self,
        _node: &mut RouterNode<'_, M, S, P, EndDevice>,
    ) -> Result<bool, ChildStoreError> {
        Ok(false)
    }
}

#[cfg(feature = "router")]
impl<M, S, P, C> ChildLifecycle<M, S, P, Router> for PersistentChildren<C>
where
    M: MacDriver,
    S: SecurityStateStore,
    P: ApplicationProfile,
    C: ChildTableStore,
{
    fn restore(
        &mut self,
        node: &mut RouterNode<'_, M, S, P, Router>,
    ) -> Result<ChildRestore, ChildStoreError> {
        match node.device_mut().restore_child_table(self.store_mut()) {
            Ok(count) => Ok(ChildRestore::Restored(count)),
            Err(error @ (ChildStoreError::Corrupt | ChildStoreError::ForeignNetwork)) => {
                // A corrupt or foreign snapshot must not remain available for a
                // later restart. Clearing through the runtime also drops any
                // live child state and pending Parent Announce work.
                node.device_mut()
                    .clear_persisted_child_table(self.store_mut())?;
                Ok(ChildRestore::Discarded(error))
            }
            Err(error) => Err(error),
        }
    }

    fn persist_if_dirty(
        &mut self,
        node: &mut RouterNode<'_, M, S, P, Router>,
    ) -> Result<bool, ChildStoreError> {
        node.device_mut()
            .save_child_table_if_dirty(self.store_mut())
    }

    fn clear(
        &mut self,
        node: &mut RouterNode<'_, M, S, P, Router>,
    ) -> Result<bool, ChildStoreError> {
        node.device_mut()
            .clear_persisted_child_table(self.store_mut())?;
        Ok(true)
    }

    fn clear_stale_before_fresh(
        &mut self,
        node: &mut RouterNode<'_, M, S, P, Router>,
    ) -> Result<bool, ChildStoreError> {
        let should_clear = match self.store_mut().load() {
            Ok(None) => false,
            Ok(Some(table)) => !table.is_empty(),
            // A fresh network cannot trust a corrupt child record. Recover by
            // replacing it with the explicit empty snapshot.
            Err(ChildStoreError::Corrupt | ChildStoreError::ForeignNetwork) => true,
            Err(error) => return Err(error),
        };
        if !should_clear {
            return Ok(false);
        }
        node.device_mut()
            .clear_persisted_child_table(self.store_mut())?;
        Ok(true)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum EventControl {
    Continue,
    Stop,
}

struct RouterCore<'a, M, S, P, R, C, K, St, Sv, D, O>
where
    M: MacDriver,
    S: SecurityStateStore,
    P: ApplicationProfile,
    R: DeviceRole,
    C: ChildLifecycle<M, S, P, R>,
    K: StartupPath<R> + TickPath<R>,
    St: StatusSink,
    Sv: Supervisor,
    D: Diagnostics,
    O: RouterObserver<M, R>,
{
    node: RouterNode<'a, M, S, P, R>,
    children: C,
    policy: &'static RouterPolicy,
    parts: RouterParts<St, Sv, D>,
    last_tick_us: u32,
    retry_deadline_us: u32,
    retry_delay_ms: u32,
    commissioning_attempts: u32,
    secure_rejoin_failures: u8,
    run_again_deadline_us: Option<u32>,
    last_identifying: Option<bool>,
    pending_factory_reset: bool,
    initialized: bool,
    _archetype: PhantomData<K>,
    _observer: PhantomData<O>,
}

impl<'a, M, S, P, R, C, K, St, Sv, D, O> RouterCore<'a, M, S, P, R, C, K, St, Sv, D, O>
where
    M: MacDriver,
    S: SecurityStateStore,
    P: ApplicationProfile,
    R: DeviceRole,
    C: ChildLifecycle<M, S, P, R>,
    K: StartupPath<R> + TickPath<R>,
    St: StatusSink,
    Sv: Supervisor,
    D: Diagnostics,
    O: RouterObserver<M, R>,
{
    fn new(
        node: RouterNode<'a, M, S, P, R>,
        children: C,
        policy: &'static RouterPolicy,
        parts: RouterParts<St, Sv, D>,
    ) -> Result<Self, RouterAppError> {
        if !policy.is_valid() || matches!(parts.supervisor.max_wait_ms(), Some(0)) {
            return Err(RouterAppError::InvalidPolicy);
        }
        if node.device().is_sleepy() || !node.device().rx_on_when_idle() {
            return Err(RouterAppError::NotAlwaysOnDevice);
        }
        let actual = node.device().device_type();
        if actual != K::DEVICE_TYPE {
            return Err(RouterAppError::WrongDeviceType {
                expected: K::DEVICE_TYPE,
                actual,
            });
        }
        let now = node.device().mac().monotonic_micros();
        Ok(Self {
            node,
            children,
            policy,
            parts,
            last_tick_us: now,
            retry_deadline_us: now,
            retry_delay_ms: policy.join_retry_initial_ms,
            commissioning_attempts: 0,
            secure_rejoin_failures: 0,
            run_again_deadline_us: None,
            last_identifying: None,
            pending_factory_reset: false,
            initialized: false,
            _archetype: PhantomData,
            _observer: PhantomData,
        })
    }

    fn now_us(&self) -> u32 {
        self.node.device().mac().monotonic_micros()
    }

    fn deadline_due(now: u32, deadline: u32) -> bool {
        now.wrapping_sub(deadline) < 0x8000_0000
    }

    fn remaining_us(now: u32, deadline: u32) -> u32 {
        if Self::deadline_due(now, deadline) {
            0
        } else {
            deadline.wrapping_sub(now)
        }
    }

    fn max_wait_us(&self) -> u32 {
        let watchdog_limit = self
            .parts
            .supervisor
            .max_wait_ms()
            .map(|duration_ms| duration_ms.saturating_mul(1_000))
            .unwrap_or(self.policy.max_receive_slice_us);
        self.policy.max_receive_slice_us.min(watchdog_limit)
    }

    fn next_wait_us(&self, include_retry: bool) -> u32 {
        let now = self.now_us();
        let mut wait_us = self.max_wait_us();
        if let Some(deadline) = self.run_again_deadline_us {
            wait_us = wait_us.min(Self::remaining_us(now, deadline));
        }
        if include_retry {
            wait_us = wait_us.min(Self::remaining_us(now, self.retry_deadline_us));
        }
        wait_us
    }

    fn elapsed_tick_secs(&mut self) -> u16 {
        let now = self.now_us();
        let elapsed_secs = now.wrapping_sub(self.last_tick_us) / 1_000_000;
        let elapsed_secs = elapsed_secs.min(u32::from(u16::MAX)) as u16;
        self.last_tick_us = self
            .last_tick_us
            .wrapping_add(u32::from(elapsed_secs) * 1_000_000);
        elapsed_secs
    }

    fn set_status(&mut self, status: RouterStatus) {
        if St::PRESENT {
            self.parts.status.set(status);
        }
    }

    fn checkpoint_security(&mut self) -> Result<(), RouterAppError> {
        let changed = self.node.checkpoint_security()?;
        self.parts
            .diagnostics
            .record(DiagnosticEvent::SecurityCheckpoint { changed });
        Ok(())
    }

    fn restore_children(&mut self) -> Result<(), RouterAppError> {
        match self.children.restore(&mut self.node)? {
            ChildRestore::NotApplicable => {}
            #[cfg(feature = "router")]
            ChildRestore::Restored(count) => self
                .parts
                .diagnostics
                .record(DiagnosticEvent::ChildrenRestored { count }),
            #[cfg(feature = "router")]
            ChildRestore::Discarded(error) => self
                .parts
                .diagnostics
                .record(DiagnosticEvent::ChildTableDiscarded { error }),
        }
        Ok(())
    }

    fn persist_children_if_dirty(&mut self) -> Result<(), RouterAppError> {
        if self.children.persist_if_dirty(&mut self.node)? {
            self.parts
                .diagnostics
                .record(DiagnosticEvent::ChildTableSaved);
        }
        Ok(())
    }

    fn clear_children(&mut self) -> Result<(), RouterAppError> {
        if self.children.clear(&mut self.node)? {
            self.parts
                .diagnostics
                .record(DiagnosticEvent::ChildTableCleared);
        }
        Ok(())
    }

    fn clear_stale_children_before_fresh(&mut self) -> Result<(), RouterAppError> {
        if self.children.clear_stale_before_fresh(&mut self.node)? {
            self.parts
                .diagnostics
                .record(DiagnosticEvent::ChildTableCleared);
        }
        Ok(())
    }

    fn refresh_online_status(&mut self) {
        if !St::PRESENT || !self.node.device().is_joined() {
            return;
        }
        let identifying = self
            .node
            .device()
            .is_identifying(self.node.profile().endpoint());
        if self.last_identifying != Some(identifying) {
            self.last_identifying = Some(identifying);
            self.parts.status.set(RouterStatus::Online {
                archetype: K::ID,
                short_address: self.node.device().short_address(),
                identifying,
            });
        }
    }

    fn activate_network(&mut self, short_address: u16) -> Result<(), RouterAppError> {
        // No receive/tick/parent-command path is entered before this returns.
        self.checkpoint_security()?;
        self.restore_children()?;
        self.node.reset_remote_reporting();
        self.retry_delay_ms = self.policy.join_retry_initial_ms;
        self.secure_rejoin_failures = 0;
        self.run_again_deadline_us = None;
        self.last_tick_us = self.now_us();
        self.last_identifying = None;
        self.parts
            .diagnostics
            .record(DiagnosticEvent::NetworkReady {
                archetype: K::ID,
                short_address,
                channel: self.node.device().channel(),
                pan_id: self.node.device().pan_id(),
            });
        self.refresh_online_status();
        O::on_network_ready(self.node.device());
        Ok(())
    }

    fn schedule_failed_attempt(&mut self) {
        let delay_ms = self.retry_delay_ms;
        self.retry_deadline_us = self.now_us().wrapping_add(delay_ms.saturating_mul(1_000));
        self.parts
            .diagnostics
            .record(DiagnosticEvent::RetryScheduled {
                attempt: self.commissioning_attempts.wrapping_add(1),
                delay_ms,
            });
        self.set_status(RouterStatus::Recommissioning {
            archetype: K::ID,
            attempt: self.commissioning_attempts.wrapping_add(1),
            retry_in_ms: delay_ms,
        });
        self.retry_delay_ms = self
            .retry_delay_ms
            .saturating_mul(2)
            .min(self.policy.join_retry_max_ms);
    }

    fn schedule_immediate_recommission(&mut self) {
        self.retry_delay_ms = self.policy.join_retry_initial_ms;
        self.retry_deadline_us = self.now_us();
        self.run_again_deadline_us = None;
        self.last_identifying = None;
        self.parts
            .diagnostics
            .record(DiagnosticEvent::RetryScheduled {
                attempt: self.commissioning_attempts.wrapping_add(1),
                delay_ms: 0,
            });
        self.set_status(RouterStatus::Recommissioning {
            archetype: K::ID,
            attempt: self.commissioning_attempts.wrapping_add(1),
            retry_in_ms: 0,
        });
    }

    async fn reset_network(&mut self) -> Result<(), RouterAppError> {
        self.node
            .factory_reset()
            .await
            .map_err(RouterAppError::Start)?;
        self.parts.diagnostics.record(DiagnosticEvent::FactoryReset);
        self.clear_children()?;
        self.node.reset_remote_reporting();
        self.run_again_deadline_us = None;
        self.last_identifying = None;
        Ok(())
    }

    fn request_factory_reset(&mut self) {
        self.pending_factory_reset = true;
        self.run_again_deadline_us = None;
        self.last_identifying = None;
        self.set_status(RouterStatus::Resetting { archetype: K::ID });
    }

    const fn factory_reset_pending(&self) -> bool {
        self.pending_factory_reset
    }

    async fn complete_pending_factory_reset_and_recommission(
        &mut self,
    ) -> Result<(), RouterAppError> {
        if !self.pending_factory_reset {
            return Ok(());
        }
        let result = self.reset_network().await;
        O::on_urgent_factory_reset_result(self.node.device(), result);
        result?;
        self.pending_factory_reset = false;
        self.secure_rejoin_failures = 0;
        self.schedule_immediate_recommission();
        self.parts.supervisor.heartbeat();
        Ok(())
    }

    /// Complete an urgent product-requested reset before yielding to the
    /// normal receive/retry loop.
    ///
    /// The security journal is reset first through [`ZigbeeNode`], then any
    /// persistent child table is cleared. Only after both operations succeed
    /// is fresh commissioning made immediately due. This ordering lets a
    /// composition root call the operation before [`Self::step`] even when an
    /// older retry deadline is already due.
    async fn urgent_factory_reset_and_recommission(&mut self) -> Result<(), RouterAppError> {
        self.request_factory_reset();
        self.complete_pending_factory_reset_and_recommission().await
    }

    async fn note_secure_rejoin_failure(
        &mut self,
        error: Option<StartError>,
    ) -> Result<(), RouterAppError> {
        if let Some(error @ StartError::PersistenceFailed(_)) = error {
            return Err(RouterAppError::Start(error));
        }
        self.secure_rejoin_failures = self.secure_rejoin_failures.saturating_add(1);
        match error {
            Some(error) => self
                .parts
                .diagnostics
                .record(DiagnosticEvent::SecureRejoinFailed {
                    error,
                    failures: self.secure_rejoin_failures,
                }),
            None => self
                .parts
                .diagnostics
                .record(DiagnosticEvent::SecureRejoinRetryFailed {
                    failures: self.secure_rejoin_failures,
                }),
        }
        if self.secure_rejoin_failures < self.policy.secure_rejoin_failure_limit {
            self.parts
                .diagnostics
                .record(DiagnosticEvent::SecureRejoinPending {
                    failures: self.secure_rejoin_failures,
                });
            self.set_status(RouterStatus::Rejoining {
                archetype: K::ID,
                failures: self.secure_rejoin_failures,
            });
            return Ok(());
        }

        self.parts
            .diagnostics
            .record(DiagnosticEvent::SecureRejoinLimitReached {
                failures: self.secure_rejoin_failures,
            });
        self.secure_rejoin_failures = 0;
        self.request_factory_reset();
        Ok(())
    }

    async fn attempt_start(&mut self) -> Result<(), RouterAppError> {
        self.commissioning_attempts = self.commissioning_attempts.wrapping_add(1);
        self.set_status(RouterStatus::Commissioning {
            archetype: K::ID,
            attempt: self.commissioning_attempts,
        });
        self.parts
            .diagnostics
            .record(DiagnosticEvent::CommissioningAttempt {
                archetype: K::ID,
                attempt: self.commissioning_attempts,
            });
        let started_us = self.now_us();
        O::on_commissioning_attempt(self.node.device(), self.commissioning_attempts, started_us);
        let pending_before = self.node.device().secure_rejoin_pending();
        let result = K::start(&mut self.node).await;
        O::on_start_result(self.node.device(), result);

        match result {
            Ok(short_address) => self.activate_network(short_address),
            Err(error @ StartError::PersistenceFailed(_)) => {
                self.parts
                    .diagnostics
                    .record(DiagnosticEvent::StartFailed { error });
                Err(RouterAppError::Start(error))
            }
            Err(error) => {
                self.parts
                    .diagnostics
                    .record(DiagnosticEvent::StartFailed { error });
                if pending_before || self.node.device().secure_rejoin_pending() {
                    self.note_secure_rejoin_failure(Some(error)).await
                } else {
                    if self
                        .node
                        .device()
                        .steering_diagnostics()
                        .device_annce_exhausted()
                    {
                        self.request_factory_reset();
                        return Ok(());
                    }
                    self.schedule_failed_attempt();
                    Ok(())
                }
            }
        }
    }

    fn set_run_again(&mut self, delay_ms: u32) -> Result<(), RouterAppError> {
        if delay_ms > RouterPolicy::max_relative_delay_us() / 1_000 {
            return Err(RouterAppError::InvalidRunAgainDelay { delay_ms });
        }
        self.run_again_deadline_us =
            Some(self.now_us().wrapping_add(delay_ms.saturating_mul(1_000)));
        self.parts
            .diagnostics
            .record(DiagnosticEvent::RunAgain { delay_ms });
        Ok(())
    }

    async fn handle_stack_event(
        &mut self,
        event: &StackEvent,
    ) -> Result<EventControl, RouterAppError> {
        O::on_stack_event(self.node.device(), event);
        self.parts
            .diagnostics
            .record(DiagnosticEvent::StackEvent(summarize_stack_event(event)));

        match event {
            StackEvent::Joined { short_address, .. } => {
                self.activate_network(*short_address)?;
                Ok(EventControl::Continue)
            }
            StackEvent::Left => {
                self.request_factory_reset();
                Ok(EventControl::Stop)
            }
            StackEvent::AttributeReport { .. } => Ok(EventControl::Continue),
            StackEvent::CommandReceived { .. } => Ok(EventControl::Continue),
            StackEvent::CommissioningComplete { success: true } => {
                self.secure_rejoin_failures = 0;
                self.checkpoint_security()?;
                Ok(EventControl::Continue)
            }
            StackEvent::CommissioningComplete { success: false } => {
                self.checkpoint_security()?;
                if self.node.device().secure_rejoin_pending() {
                    self.note_secure_rejoin_failure(None).await?;
                } else {
                    self.clear_children()?;
                    self.node.reset_remote_reporting();
                    self.schedule_immediate_recommission();
                }
                Ok(EventControl::Stop)
            }
            StackEvent::DefaultResponse { .. } => Ok(EventControl::Continue),
            StackEvent::ReportingConfigured { .. } => Ok(EventControl::Continue),
            StackEvent::PermitJoinChanged { .. } => Ok(EventControl::Continue),
            StackEvent::ReportSent => Ok(EventControl::Continue),
            StackEvent::OtaImageAvailable { .. } => Ok(EventControl::Continue),
            StackEvent::OtaProgress { .. } => Ok(EventControl::Continue),
            StackEvent::OtaComplete => Ok(EventControl::Continue),
            StackEvent::OtaFailed => Ok(EventControl::Continue),
            StackEvent::OtaDelayedActivation { .. } => Ok(EventControl::Continue),
            // Local ZCL dispatch already restored writable application
            // attributes. Basic Reset is not Leave/factory-new: network
            // membership, credentials, counter floors, bindings, groups, and
            // parent state remain intact. Return to the composition root
            // immediately so fitted outputs can be synchronized before any
            // subsequent maintenance tick.
            StackEvent::BasicResetToFactoryDefaults => Ok(EventControl::Stop),
            StackEvent::LeaveRequested => {
                self.request_factory_reset();
                Ok(EventControl::Stop)
            }
            StackEvent::RejoinRequested => {
                self.set_status(RouterStatus::Rejoining {
                    archetype: K::ID,
                    failures: self.secure_rejoin_failures,
                });
                let started_us = self.now_us();
                O::on_secure_rejoin_attempt(self.node.device(), started_us);
                let result = self.node.secure_rejoin().await;
                O::on_secure_rejoin_result(self.node.device(), result);
                match result {
                    Ok(short_address) => {
                        self.parts
                            .diagnostics
                            .record(DiagnosticEvent::SecureRejoinSucceeded { short_address });
                        self.activate_network(short_address)?;
                    }
                    Err(error) => self.note_secure_rejoin_failure(Some(error)).await?,
                }
                // The network relationship changed underneath this service
                // iteration. Never continue into a second tick against it.
                Ok(EventControl::Stop)
            }
        }
    }

    async fn handle_tick_result(
        &mut self,
        elapsed_secs: u16,
        result: TickResult,
        events: &mut StepEvents,
    ) -> Result<EventControl, RouterAppError> {
        O::on_tick(self.node.device(), elapsed_secs, &result);
        match result {
            TickResult::Idle => {
                self.run_again_deadline_us = None;
                Ok(EventControl::Continue)
            }
            TickResult::RunAgain(delay_ms) => {
                self.set_run_again(delay_ms)?;
                Ok(EventControl::Continue)
            }
            TickResult::Event(event) => {
                self.run_again_deadline_us = None;
                let control = self.handle_stack_event(&event).await?;
                events.tick = Some(event);
                Ok(control)
            }
        }
    }

    async fn step_joined(&mut self) -> Result<StepEvents, RouterAppError> {
        let mut events = StepEvents::default();
        self.parts.supervisor.heartbeat();

        let timeout_us = self.next_wait_us(false);
        if timeout_us != 0 {
            let started_us = self.now_us();
            O::on_before_receive(self.node.device(), timeout_us);
            match self.node.device_mut().receive_timeout(timeout_us).await {
                Ok(indication) => {
                    let receive_elapsed_us = self.now_us().wrapping_sub(started_us);
                    self.parts
                        .diagnostics
                        .record(DiagnosticEvent::FrameReceived);
                    O::on_frame_received(self.node.device(), receive_elapsed_us);
                    let event = self
                        .node
                        .process_incoming_deferred_reset(&indication)
                        .await?;
                    let elapsed_us = self.now_us().wrapping_sub(started_us);
                    O::on_frame_processed(self.node.device(), event.as_ref(), elapsed_us);
                    if let Some(event) = event {
                        let control = self.handle_stack_event(&event).await?;
                        events.incoming = Some(event);
                        if !self.pending_factory_reset {
                            self.persist_children_if_dirty()?;
                        }
                        if matches!(control, EventControl::Stop) {
                            self.parts.supervisor.heartbeat();
                            return Ok(events);
                        }
                    }
                }
                Err(MacError::NoData) => {}
                Err(error) => return Err(RouterAppError::Mac(error)),
            }
        }

        // Parent command servicing surrounds receive_timeout and may mutate
        // the child table even when no normal data indication arrived.
        self.persist_children_if_dirty()?;

        let elapsed_secs = self.elapsed_tick_secs();
        self.run_again_deadline_us = None;
        let result = K::tick(&mut self.node, elapsed_secs).await?;
        let control = self
            .handle_tick_result(elapsed_secs, result, &mut events)
            .await?;
        if !self.pending_factory_reset {
            self.persist_children_if_dirty()?;
        }
        if matches!(control, EventControl::Continue) {
            self.refresh_online_status();
        }
        self.parts.supervisor.heartbeat();
        Ok(events)
    }

    async fn step_unjoined(&mut self) -> Result<StepEvents, RouterAppError> {
        let mut events = StepEvents::default();
        self.parts.supervisor.heartbeat();

        if !self.node.device().secure_rejoin_pending()
            && Self::deadline_due(self.now_us(), self.retry_deadline_us)
        {
            self.attempt_start().await?;
            self.parts.supervisor.heartbeat();
            return Ok(events);
        }

        let wait_us = self.next_wait_us(!self.node.device().secure_rejoin_pending());
        if wait_us != 0 {
            self.node.device_mut().mac_mut().delay_micros(wait_us).await;
        }

        let elapsed_secs = self.elapsed_tick_secs();
        self.run_again_deadline_us = None;
        let result = K::tick(&mut self.node, elapsed_secs).await?;
        self.handle_tick_result(elapsed_secs, result, &mut events)
            .await?;
        // An unjoined parent cannot admit or age children. In particular, do
        // not let the role state's initially-dirty empty table overwrite a
        // valid durable snapshot while a persisted secured rejoin is retrying.
        // A successful Joined event restores the snapshot before the next
        // receive; a real recommission path clears it explicitly.
        self.parts.supervisor.heartbeat();
        Ok(events)
    }

    async fn initialize_deferred_factory_reset(&mut self) -> Result<(), RouterAppError> {
        if self.initialized {
            return Err(RouterAppError::AlreadyInitialized);
        }
        self.initialized = true;
        self.set_status(RouterStatus::Starting { archetype: K::ID });
        self.parts
            .diagnostics
            .record(DiagnosticEvent::InitializationStarted { archetype: K::ID });
        let commissioned = self
            .node
            .load_security_state()?
            .is_some_and(|state| state.commissioned);
        if !commissioned {
            // Without a commissioned security record there is no current
            // parent membership to which a stored child table can belong.
            // Clear before fresh steering/formation, including the
            // same-extended-PAN-ID case that EPID binding cannot distinguish.
            self.clear_stale_children_before_fresh()?;
        }
        self.node
            .configure_default_reporting()
            .map_err(|error| RouterAppError::Node(error.into()))?;
        self.parts
            .diagnostics
            .record(DiagnosticEvent::DefaultReportingConfigured);
        self.attempt_start().await
    }

    async fn initialize(&mut self) -> Result<(), RouterAppError> {
        self.initialize_deferred_factory_reset().await?;
        self.complete_pending_factory_reset_and_recommission().await
    }

    async fn step_deferred_factory_reset(&mut self) -> Result<StepEvents, RouterAppError> {
        if !self.initialized {
            return Err(RouterAppError::NotInitialized);
        }
        if self.pending_factory_reset {
            Ok(StepEvents::default())
        } else if self.node.device().is_joined() {
            self.step_joined().await
        } else {
            self.step_unjoined().await
        }
    }

    async fn step(&mut self) -> Result<StepEvents, RouterAppError> {
        let events = self.step_deferred_factory_reset().await?;
        self.complete_pending_factory_reset_and_recommission()
            .await?;
        Ok(events)
    }

    fn fatal(&mut self, error: RouterAppError) -> ! {
        O::on_fault(self.node.device(), error);
        self.parts.diagnostics.record(DiagnosticEvent::Fatal(error));
        self.set_status(RouterStatus::Fault { archetype: K::ID });
        self.parts.supervisor.reset()
    }

    async fn run(&mut self) -> ! {
        if let Err(error) = self.initialize().await {
            self.fatal(error);
        }
        loop {
            if let Err(error) = self.step().await {
                self.fatal(error);
            }
        }
    }
}

#[cfg(feature = "router")]
type RelayCore<'a, M, S, P, St, Sv, D, O> =
    RouterCore<'a, M, S, P, RelayRouter, NoChildren, RelayArchetype, St, Sv, D, O>;

type AlwaysOnEndDeviceCore<'a, M, S, P, St, Sv, D, O> =
    RouterCore<'a, M, S, P, EndDevice, NoChildren, AlwaysOnEndDeviceArchetype, St, Sv, D, O>;

/// Mains-powered, non-routing Zigbee end-device application.
///
/// This frontend is intentionally distinct from the router frontends: its
/// [`zigbee_runtime::role::EndDevice`] role accepts only
/// [`DeviceType::EndDevice`], carries no routing or child-serving state, and
/// uses Network Steering for fresh join and persisted rejoin. Construction
/// rejects a sleepy or receiver-off device, so an `AlwaysOnEndDeviceApp`
/// cannot advertise `macRxOnWhenIdle = false`.
pub struct AlwaysOnEndDeviceApp<'a, M, S, P, St, Sv, D, O = NoObserver>
where
    M: MacDriver,
    S: SecurityStateStore,
    P: ApplicationProfile,
    St: StatusSink,
    Sv: Supervisor,
    D: Diagnostics,
    O: RouterObserver<M, EndDevice>,
{
    core: AlwaysOnEndDeviceCore<'a, M, S, P, St, Sv, D, O>,
}

impl<'a, M, S, P, St, Sv, D, O> AlwaysOnEndDeviceApp<'a, M, S, P, St, Sv, D, O>
where
    M: MacDriver,
    S: SecurityStateStore,
    P: ApplicationProfile,
    St: StatusSink,
    Sv: Supervisor,
    D: Diagnostics,
    O: RouterObserver<M, EndDevice>,
{
    /// Construct with a statically selected non-default observer type `O`.
    pub fn new_observed(
        node: RouterNode<'a, M, S, P, EndDevice>,
        policy: &'static RouterPolicy,
        parts: RouterParts<St, Sv, D>,
    ) -> Result<Self, RouterAppError> {
        Ok(Self {
            core: RouterCore::new(node, NoChildren, policy, parts)?,
        })
    }

    pub const fn node(&self) -> &RouterNode<'a, M, S, P, EndDevice> {
        &self.core.node
    }

    pub fn node_mut(&mut self) -> &mut RouterNode<'a, M, S, P, EndDevice> {
        &mut self.core.node
    }

    pub fn parts(&self) -> &RouterParts<St, Sv, D> {
        &self.core.parts
    }

    pub fn parts_mut(&mut self) -> &mut RouterParts<St, Sv, D> {
        &mut self.core.parts
    }

    pub async fn initialize(&mut self) -> Result<(), RouterAppError> {
        self.core.initialize().await
    }

    /// Initialize without committing an automatically requested network
    /// reset. Safety-critical composition roots can make fitted outputs and
    /// application state durable before calling
    /// [`Self::complete_pending_factory_reset_and_recommission`].
    pub async fn initialize_deferred_factory_reset(&mut self) -> Result<(), RouterAppError> {
        self.core.initialize_deferred_factory_reset().await
    }

    pub const fn factory_reset_pending(&self) -> bool {
        self.core.factory_reset_pending()
    }

    pub async fn complete_pending_factory_reset_and_recommission(
        &mut self,
    ) -> Result<(), RouterAppError> {
        self.core
            .complete_pending_factory_reset_and_recommission()
            .await
    }

    /// Urgently factory-reset durable network state and schedule fresh
    /// Network Steering before the next [`Self::step`].
    pub async fn urgent_factory_reset_and_recommission(&mut self) -> Result<(), RouterAppError> {
        self.core.urgent_factory_reset_and_recommission().await
    }

    pub async fn step(&mut self) -> Result<StepEvents, RouterAppError> {
        self.core.step().await
    }

    /// Run one finite step while deferring any network reset requested by the
    /// stack until the composition root has completed its safety transaction.
    pub async fn step_deferred_factory_reset(&mut self) -> Result<StepEvents, RouterAppError> {
        self.core.step_deferred_factory_reset().await
    }

    pub async fn run(&mut self) -> ! {
        self.core.run().await
    }
}

impl<'a, M, S, P, St, Sv, D> AlwaysOnEndDeviceApp<'a, M, S, P, St, Sv, D, NoObserver>
where
    M: MacDriver,
    S: SecurityStateStore,
    P: ApplicationProfile,
    St: StatusSink,
    Sv: Supervisor,
    D: Diagnostics,
{
    pub fn new(
        node: RouterNode<'a, M, S, P, EndDevice>,
        policy: &'static RouterPolicy,
        parts: RouterParts<St, Sv, D>,
    ) -> Result<Self, RouterAppError> {
        Self::new_observed(node, policy, parts)
    }
}

/// Forwarding-only router application.
#[cfg(feature = "router")]
pub struct RelayRouterApp<'a, M, S, P, St, Sv, D, O = NoObserver>
where
    M: MacDriver,
    S: SecurityStateStore,
    P: ApplicationProfile,
    St: StatusSink,
    Sv: Supervisor,
    D: Diagnostics,
    O: RouterObserver<M, RelayRouter>,
{
    core: RelayCore<'a, M, S, P, St, Sv, D, O>,
}

#[cfg(feature = "router")]
impl<'a, M, S, P, St, Sv, D, O> RelayRouterApp<'a, M, S, P, St, Sv, D, O>
where
    M: MacDriver,
    S: SecurityStateStore,
    P: ApplicationProfile,
    St: StatusSink,
    Sv: Supervisor,
    D: Diagnostics,
    O: RouterObserver<M, RelayRouter>,
{
    /// Construct with a statically selected non-default observer type `O`.
    pub fn new_observed(
        node: RouterNode<'a, M, S, P, RelayRouter>,
        children: NoChildren,
        policy: &'static RouterPolicy,
        parts: RouterParts<St, Sv, D>,
    ) -> Result<Self, RouterAppError> {
        Ok(Self {
            core: RouterCore::new(node, children, policy, parts)?,
        })
    }

    pub const fn node(&self) -> &RouterNode<'a, M, S, P, RelayRouter> {
        &self.core.node
    }

    pub fn node_mut(&mut self) -> &mut RouterNode<'a, M, S, P, RelayRouter> {
        &mut self.core.node
    }

    pub const fn children(&self) -> &NoChildren {
        &self.core.children
    }

    pub fn parts(&self) -> &RouterParts<St, Sv, D> {
        &self.core.parts
    }

    pub fn parts_mut(&mut self) -> &mut RouterParts<St, Sv, D> {
        &mut self.core.parts
    }

    pub async fn initialize(&mut self) -> Result<(), RouterAppError> {
        self.core.initialize().await
    }

    pub async fn initialize_deferred_factory_reset(&mut self) -> Result<(), RouterAppError> {
        self.core.initialize_deferred_factory_reset().await
    }

    pub const fn factory_reset_pending(&self) -> bool {
        self.core.factory_reset_pending()
    }

    pub async fn complete_pending_factory_reset_and_recommission(
        &mut self,
    ) -> Result<(), RouterAppError> {
        self.core
            .complete_pending_factory_reset_and_recommission()
            .await
    }

    /// Urgently factory-reset durable network state and schedule fresh
    /// Network Steering.
    ///
    /// Call this directly from the product event loop before [`Self::step`]
    /// when a fitted control requests reset. The operation itself never starts
    /// steering: the next `step()` owns the fresh commissioning attempt after
    /// the security journal reset has completed.
    pub async fn urgent_factory_reset_and_recommission(&mut self) -> Result<(), RouterAppError> {
        self.core.urgent_factory_reset_and_recommission().await
    }

    pub async fn step(&mut self) -> Result<StepEvents, RouterAppError> {
        self.core.step().await
    }

    pub async fn step_deferred_factory_reset(&mut self) -> Result<StepEvents, RouterAppError> {
        self.core.step_deferred_factory_reset().await
    }

    pub async fn run(&mut self) -> ! {
        self.core.run().await
    }
}

#[cfg(feature = "router")]
impl<'a, M, S, P, St, Sv, D> RelayRouterApp<'a, M, S, P, St, Sv, D, NoObserver>
where
    M: MacDriver,
    S: SecurityStateStore,
    P: ApplicationProfile,
    St: StatusSink,
    Sv: Supervisor,
    D: Diagnostics,
{
    pub fn new(
        node: RouterNode<'a, M, S, P, RelayRouter>,
        children: NoChildren,
        policy: &'static RouterPolicy,
        parts: RouterParts<St, Sv, D>,
    ) -> Result<Self, RouterAppError> {
        Self::new_observed(node, children, policy, parts)
    }
}

#[cfg(feature = "router")]
type ParentCore<'a, M, S, P, C, St, Sv, D, O> =
    RouterCore<'a, M, S, P, Router, PersistentChildren<C>, ParentArchetype, St, Sv, D, O>;

/// Child-capable persistent router application.
#[cfg(feature = "router")]
pub struct ParentRouterApp<'a, M, S, P, C, St, Sv, D, O = NoObserver>
where
    M: ParentMacDriver,
    S: SecurityStateStore,
    P: ApplicationProfile,
    C: ChildTableStore,
    St: StatusSink,
    Sv: Supervisor,
    D: Diagnostics,
    O: RouterObserver<M, Router>,
{
    core: ParentCore<'a, M, S, P, C, St, Sv, D, O>,
}

#[cfg(feature = "router")]
impl<'a, M, S, P, C, St, Sv, D, O> ParentRouterApp<'a, M, S, P, C, St, Sv, D, O>
where
    M: ParentMacDriver,
    S: SecurityStateStore,
    P: ApplicationProfile,
    C: ChildTableStore,
    St: StatusSink,
    Sv: Supervisor,
    D: Diagnostics,
    O: RouterObserver<M, Router>,
{
    /// Construct with a statically selected non-default observer type `O`.
    pub fn new_observed(
        node: RouterNode<'a, M, S, P, Router>,
        children: PersistentChildren<C>,
        policy: &'static RouterPolicy,
        parts: RouterParts<St, Sv, D>,
    ) -> Result<Self, RouterAppError> {
        Ok(Self {
            core: RouterCore::new(node, children, policy, parts)?,
        })
    }

    pub const fn node(&self) -> &RouterNode<'a, M, S, P, Router> {
        &self.core.node
    }

    pub fn node_mut(&mut self) -> &mut RouterNode<'a, M, S, P, Router> {
        &mut self.core.node
    }

    pub const fn children(&self) -> &PersistentChildren<C> {
        &self.core.children
    }

    pub fn children_mut(&mut self) -> &mut PersistentChildren<C> {
        &mut self.core.children
    }

    pub fn parts(&self) -> &RouterParts<St, Sv, D> {
        &self.core.parts
    }

    pub fn parts_mut(&mut self) -> &mut RouterParts<St, Sv, D> {
        &mut self.core.parts
    }

    pub async fn initialize(&mut self) -> Result<(), RouterAppError> {
        self.core.initialize().await
    }

    pub async fn initialize_deferred_factory_reset(&mut self) -> Result<(), RouterAppError> {
        self.core.initialize_deferred_factory_reset().await
    }

    pub const fn factory_reset_pending(&self) -> bool {
        self.core.factory_reset_pending()
    }

    pub async fn complete_pending_factory_reset_and_recommission(
        &mut self,
    ) -> Result<(), RouterAppError> {
        self.core
            .complete_pending_factory_reset_and_recommission()
            .await
    }

    /// Urgently factory-reset durable network and child state, then schedule
    /// fresh Network Steering.
    ///
    /// Call this directly from the product event loop before [`Self::step`].
    /// The journal-aware node reset and persistent child clear both complete
    /// before the next `step()` can begin steering.
    pub async fn urgent_factory_reset_and_recommission(&mut self) -> Result<(), RouterAppError> {
        self.core.urgent_factory_reset_and_recommission().await
    }

    pub async fn step(&mut self) -> Result<StepEvents, RouterAppError> {
        self.core.step().await
    }

    pub async fn step_deferred_factory_reset(&mut self) -> Result<StepEvents, RouterAppError> {
        self.core.step_deferred_factory_reset().await
    }

    pub async fn run(&mut self) -> ! {
        self.core.run().await
    }
}

#[cfg(feature = "router")]
impl<'a, M, S, P, C, St, Sv, D> ParentRouterApp<'a, M, S, P, C, St, Sv, D, NoObserver>
where
    M: ParentMacDriver,
    S: SecurityStateStore,
    P: ApplicationProfile,
    C: ChildTableStore,
    St: StatusSink,
    Sv: Supervisor,
    D: Diagnostics,
{
    pub fn new(
        node: RouterNode<'a, M, S, P, Router>,
        children: PersistentChildren<C>,
        policy: &'static RouterPolicy,
        parts: RouterParts<St, Sv, D>,
    ) -> Result<Self, RouterAppError> {
        Self::new_observed(node, children, policy, parts)
    }
}

#[cfg(feature = "router")]
type CoordinatorCore<'a, M, S, P, C, St, Sv, D, O> =
    RouterCore<'a, M, S, P, Router, PersistentChildren<C>, CoordinatorArchetype, St, Sv, D, O>;

/// Coordinator composition frontend over the parent runtime role.
#[cfg(feature = "router")]
pub struct CoordinatorApp<'a, M, S, P, C, St, Sv, D, O = NoObserver>
where
    M: ParentMacDriver,
    S: SecurityStateStore,
    P: ApplicationProfile,
    C: ChildTableStore,
    St: StatusSink,
    Sv: Supervisor,
    D: Diagnostics,
    O: RouterObserver<M, Router>,
{
    core: CoordinatorCore<'a, M, S, P, C, St, Sv, D, O>,
}

#[cfg(feature = "router")]
impl<'a, M, S, P, C, St, Sv, D, O> CoordinatorApp<'a, M, S, P, C, St, Sv, D, O>
where
    M: ParentMacDriver,
    S: SecurityStateStore,
    P: ApplicationProfile,
    C: ChildTableStore,
    St: StatusSink,
    Sv: Supervisor,
    D: Diagnostics,
    O: RouterObserver<M, Router>,
{
    /// Construct with a statically selected non-default observer type `O`.
    pub fn new_observed(
        node: RouterNode<'a, M, S, P, Router>,
        children: PersistentChildren<C>,
        policy: &'static RouterPolicy,
        parts: RouterParts<St, Sv, D>,
    ) -> Result<Self, RouterAppError> {
        Ok(Self {
            core: RouterCore::new(node, children, policy, parts)?,
        })
    }

    pub const fn node(&self) -> &RouterNode<'a, M, S, P, Router> {
        &self.core.node
    }

    pub fn node_mut(&mut self) -> &mut RouterNode<'a, M, S, P, Router> {
        &mut self.core.node
    }

    pub const fn children(&self) -> &PersistentChildren<C> {
        &self.core.children
    }

    pub fn children_mut(&mut self) -> &mut PersistentChildren<C> {
        &mut self.core.children
    }

    pub fn parts(&self) -> &RouterParts<St, Sv, D> {
        &self.core.parts
    }

    pub fn parts_mut(&mut self) -> &mut RouterParts<St, Sv, D> {
        &mut self.core.parts
    }

    pub async fn initialize(&mut self) -> Result<(), RouterAppError> {
        self.core.initialize().await
    }

    pub async fn initialize_deferred_factory_reset(&mut self) -> Result<(), RouterAppError> {
        self.core.initialize_deferred_factory_reset().await
    }

    pub const fn factory_reset_pending(&self) -> bool {
        self.core.factory_reset_pending()
    }

    pub async fn complete_pending_factory_reset_and_recommission(
        &mut self,
    ) -> Result<(), RouterAppError> {
        self.core
            .complete_pending_factory_reset_and_recommission()
            .await
    }

    pub async fn step(&mut self) -> Result<StepEvents, RouterAppError> {
        self.core.step().await
    }

    pub async fn step_deferred_factory_reset(&mut self) -> Result<StepEvents, RouterAppError> {
        self.core.step_deferred_factory_reset().await
    }

    pub async fn run(&mut self) -> ! {
        self.core.run().await
    }
}

#[cfg(feature = "router")]
impl<'a, M, S, P, C, St, Sv, D> CoordinatorApp<'a, M, S, P, C, St, Sv, D, NoObserver>
where
    M: ParentMacDriver,
    S: SecurityStateStore,
    P: ApplicationProfile,
    C: ChildTableStore,
    St: StatusSink,
    Sv: Supervisor,
    D: Diagnostics,
{
    pub fn new(
        node: RouterNode<'a, M, S, P, Router>,
        children: PersistentChildren<C>,
        policy: &'static RouterPolicy,
        parts: RouterParts<St, Sv, D>,
    ) -> Result<Self, RouterAppError> {
        Self::new_observed(node, children, policy, parts)
    }
}
