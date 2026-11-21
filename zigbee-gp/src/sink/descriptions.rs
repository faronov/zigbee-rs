//! GPD Application Description collection (gecko_sdk
//! `gpCommissioningNotificationApplicationDescriptionGpdf`,
//! `saveReportDescriptor`, the multi-sensor commissioning timeout and GP
//! Pairing Configuration action 0b101).
//!
//! A GPD whose Commissioning command announces "application description
//! follows" is not paired at once: its entry waits in the pending list
//! while the sink collects the report descriptors of its Application
//! Description commands (direct or in GP Commissioning Notifications) for
//! [`APP_DESCRIPTION_TIMEOUT_MS`]. Every frame with new reports becomes a
//! [`SinkEvent::ApplicationDescription`]; once all `total_reports` are known
//! the pairing completes, with a Commissioning Reply when the frame holding
//! the last report asked for one (rxAfterTx). One GPD is collected at a
//! time, as gecko_sdk's single multi-sensor timer does. The sink does not
//! interpret the descriptors (no Compact Attribute Reporting translation):
//! the application or host does, from the events.
//!
//! Without the `app-description` feature the hooks below keep the previous
//! behaviour (Application Description commands are unsupported, Pairing
//! Configuration action 0b101 fails) and compile to the same code.

use super::*;

#[cfg(not(feature = "app-description"))]
pub(super) type PendingState = heapless::Vec<Pending, MAX_PENDING_COMMISSIONING>;

#[cfg(not(feature = "app-description"))]
impl<const N: usize, const Q: usize> Sink<N, Q> {
    #[inline(always)]
    pub(super) fn on_application_description(&mut self, _rx: &Received<'_>) -> SinkOutcome {
        SinkOutcome::Dropped(SinkDrop::Unsupported)
    }

    #[inline(always)]
    pub(super) fn finish_unidirectional(
        &mut self,
        entry: SinkEntry,
        _p: &CommissioningPayload,
        _rx: &Received<'_>,
    ) -> SinkOutcome {
        self.finalize(entry)
    }

    #[inline(always)]
    pub(super) fn completable_index(&self, gpd: &GpdId) -> Option<usize> {
        self.pending_index(gpd)
    }

    #[inline(always)]
    pub(super) fn description_timers(&self, _now_ms: u32) -> &Option<CommissioningState> {
        &self.commissioning
    }

    #[inline(always)]
    pub(super) fn description_deadline(&self, deadline: Option<u32>) -> Option<u32> {
        deadline
    }
}

#[cfg(all(feature = "translation-table", not(feature = "app-description")))]
impl<const N: usize, const Q: usize> Sink<N, Q> {
    #[inline(always)]
    pub(super) fn refuses_description(&self, a: &ApplicationInformation<'_>) -> bool {
        a.bits & app_info::APPLICATION_DESCRIPTION != 0
    }

    #[inline(always)]
    pub(super) fn configured_description(
        &mut self,
        _pc: &PairingConfiguration<'_>,
    ) -> (u8, Option<SinkOutcome>) {
        (FAILURE, None)
    }

    #[inline(always)]
    pub(super) fn configured_with_description(
        &mut self,
        _pc: &PairingConfiguration<'_>,
    ) -> (u8, Option<SinkOutcome>) {
        (zcl_status::SUCCESS, Some(SinkOutcome::Commissioned))
    }
}

#[cfg(feature = "translation-table")]
use crate::config::{ApplicationInformation, PairingConfiguration, app_info};
/// ZCL FAILURE.
#[cfg(all(feature = "translation-table", not(feature = "app-description")))]
const FAILURE: u8 = 0x01;

#[cfg(feature = "app-description")]
pub(super) use imp::PendingState;
#[cfg(feature = "app-description")]
pub use imp::{APP_DESCRIPTION_TIMEOUT_MS, ApplicationDescriptionReports, MAX_REPORTS};

#[cfg(feature = "app-description")]
mod imp {
    use core::ops::{Deref, DerefMut};

    use super::*;
    use crate::app_description::ApplicationDescription;
    use crate::frame::MAX_GPD_PAYLOAD;

    /// How long the sink collects report descriptors after the
    /// Commissioning command (gecko_sdk
    /// `EMBER_AF_PLUGIN_GREEN_POWER_SERVER_MULTI_SENSOR_COMMISSIONING_TIMEOUT_IN_SEC`
    /// default, 20 s).
    pub const APP_DESCRIPTION_TIMEOUT_MS: u32 = 20_000;

    /// Reports one GPD may describe (report identifiers `0..32`).
    pub const MAX_REPORTS: u8 = 32;

    /// Report descriptors the sink learned for a GPD.
    #[derive(Debug, Clone, PartialEq, Eq)]
    pub struct ApplicationDescriptionReports {
        /// GPD identity.
        pub gpd_id: GpdId,
        /// Reports the GPD describes in total.
        pub total_reports: u8,
        /// Report descriptors in `reports`.
        pub report_count: u8,
        /// Report descriptors not reported before, back to back as
        /// received ([`crate::app_description::Reports`] parses them).
        pub reports: heapless::Vec<u8, MAX_GPD_PAYLOAD>,
    }

    /// The GPD whose report descriptors are being collected; its entry
    /// waits in the pending list.
    #[derive(Debug, Clone, PartialEq, Eq)]
    pub(in crate::sink) struct Collecting {
        gpd: GpdId,
        /// Commissioning Reply, sent if the last report asks for one.
        reply: CommissioningReply,
        deadline_ms: u32,
        /// `total_reports` of the first Application Description (0: none).
        total: u8,
        /// Report identifiers received.
        received: u32,
    }

    /// The pending commissionings plus the GPD being collected.
    pub(in crate::sink) struct PendingState {
        list: heapless::Vec<Pending, MAX_PENDING_COMMISSIONING>,
        collecting: Option<Collecting>,
    }

    impl PendingState {
        pub(in crate::sink) const fn new() -> Self {
            Self {
                list: heapless::Vec::new(),
                collecting: None,
            }
        }

        /// Forget every pending commissioning and the collection.
        pub(in crate::sink) fn clear(&mut self) {
            self.list.clear();
            self.collecting = None;
        }
    }

    impl Deref for PendingState {
        type Target = heapless::Vec<Pending, MAX_PENDING_COMMISSIONING>;

        fn deref(&self) -> &Self::Target {
            &self.list
        }
    }

    impl DerefMut for PendingState {
        fn deref_mut(&mut self) -> &mut Self::Target {
            &mut self.list
        }
    }

    fn all_reports(total: u8) -> u32 {
        u32::MAX >> (32 - u32::from(total))
    }

    impl<const N: usize, const Q: usize> Sink<N, Q> {
        /// The GPD whose descriptors are being collected, if any.
        pub fn collecting_application_description(&self) -> Option<GpdId> {
            self.pending.collecting.as_ref().map(|c| c.gpd)
        }

        /// The entry of a GPD whose commissioning is pending (waiting for
        /// its Application Description or its Success), e.g. to decrypt a
        /// frame for a report.
        pub fn pending_entry(&self, gpd: &GpdId) -> Option<&SinkEntry> {
            self.pending_index(gpd).map(|i| &self.pending[i].entry)
        }

        /// GPD command 0xE4 (and the unsupported Channel Request).
        pub(in crate::sink) fn on_application_description(
            &mut self,
            rx: &Received<'_>,
        ) -> SinkOutcome {
            use SinkDrop::*;
            if rx.command_id != cmd::APPLICATION_DESCRIPTION {
                return SinkOutcome::Dropped(Unsupported);
            }
            // gecko_sdk processes Application Descriptions received
            // directly or in Commissioning Notifications only.
            if rx.source == Source::Notification {
                return SinkOutcome::Dropped(NotCommissioning);
            }
            if !self.is_collecting(&rx.gpd) {
                return SinkOutcome::Dropped(NoEntry);
            }
            let Ok(desc) = ApplicationDescription::decode(rx.payload) else {
                return SinkOutcome::Dropped(Invalid);
            };
            match self.absorb(&desc) {
                Err(drop) => SinkOutcome::Dropped(drop),
                Ok(false) => SinkOutcome::CommissioningStep,
                Ok(true) => {
                    let Some((entry, reply)) = self.take_collected() else {
                        return SinkOutcome::Dropped(NoEntry);
                    };
                    if rx.rx_after_tx {
                        self.send_reply(entry, rx, &reply)
                    } else {
                        self.finalize(entry)
                    }
                }
            }
        }

        /// Unidirectional Commissioning: pair, or start collecting the
        /// announced Application Description.
        pub(in crate::sink) fn finish_unidirectional(
            &mut self,
            entry: SinkEntry,
            p: &CommissioningPayload,
            rx: &Received<'_>,
        ) -> SinkOutcome {
            if !p
                .app_info
                .as_ref()
                .is_some_and(|a| a.app_description_follows)
            {
                return self.finalize(entry);
            }
            let (security_level, key_type) = p
                .security
                .as_ref()
                .map_or((SecurityLevel::None, KeyType::NoKey), |s| {
                    (s.level, s.key_type)
                });
            let reply = CommissioningReply {
                pan_id: p.pan_id_request.then_some(self.pan_id),
                security_level,
                key_type,
                key: None,
            };
            let gpd = entry.gpd_id;
            // A repeated Commissioning keeps the reports collected so far
            // (gecko_sdk resets them only once its timer has expired).
            let (total, received) = match self.pending.collecting.take() {
                Some(c) if c.gpd.matches(&gpd) => (c.total, c.received),
                Some(c) => {
                    self.drop_pending(&c.gpd);
                    (0, 0)
                }
                None => (0, 0),
            };
            self.drop_pending(&gpd);
            if self.pending.is_full() {
                self.pending.remove(0);
            }
            let _ = self.pending.push(Pending { entry });
            self.pending.collecting = Some(Collecting {
                gpd,
                reply,
                deadline_ms: rx.now_ms.wrapping_add(APP_DESCRIPTION_TIMEOUT_MS),
                total,
                received,
            });
            SinkOutcome::CommissioningStep
        }

        /// A pending commissioning that other GPD commands may complete:
        /// not one still collecting its Application Description.
        pub(in crate::sink) fn completable_index(&self, gpd: &GpdId) -> Option<usize> {
            if self.is_collecting(gpd) {
                return None;
            }
            self.pending_index(gpd)
        }

        /// Expire the collection; returns the commissioning state for the
        /// commissioning-window check that follows.
        pub(in crate::sink) fn description_timers(
            &mut self,
            now_ms: u32,
        ) -> &Option<CommissioningState> {
            if let Some(c) = &self.pending.collecting
                && due(now_ms, c.deadline_ms)
            {
                let gpd_id = c.gpd;
                self.pending.collecting = None;
                self.drop_pending(&gpd_id);
                self.event(SinkEvent::CommissioningFailed {
                    gpd_id,
                    reason: SinkDrop::DescriptionTimeout,
                });
            }
            &self.commissioning
        }

        /// The earlier of `deadline` and the collection's.
        pub(in crate::sink) fn description_deadline(&self, deadline: Option<u32>) -> Option<u32> {
            let Some(c) = &self.pending.collecting else {
                return deadline;
            };
            match deadline {
                Some(d) if due(c.deadline_ms, d) => Some(d),
                _ => Some(c.deadline_ms),
            }
        }

        fn is_collecting(&self, gpd: &GpdId) -> bool {
            self.pending
                .collecting
                .as_ref()
                .is_some_and(|c| c.gpd.matches(gpd))
        }

        fn drop_pending(&mut self, gpd: &GpdId) {
            if let Some(i) = self.pending_index(gpd) {
                self.pending.swap_remove(i);
            }
        }

        /// Record the reports of `desc` for the GPD being collected and
        /// report the new ones; `Ok(true)` once all are known.
        fn absorb(
            &mut self,
            desc: &ApplicationDescription<'_>,
        ) -> core::result::Result<bool, SinkDrop> {
            let Some(c) = self.pending.collecting.as_mut() else {
                return Err(SinkDrop::NoEntry);
            };
            let total = desc.total_reports;
            if total > MAX_REPORTS {
                return Err(SinkDrop::Unsupported);
            }
            if (c.total != 0 && c.total != total) || desc.iter().any(|r| r.report_id >= total) {
                return Err(SinkDrop::Invalid);
            }
            c.total = total;
            let mut reports = heapless::Vec::new();
            let mut report_count = 0;
            for r in desc.iter() {
                let bit = 1u32 << r.report_id;
                if c.received & bit == 0 && reports.extend_from_slice(r.raw).is_ok() {
                    c.received |= bit;
                    report_count += 1;
                }
            }
            let complete = c.received == all_reports(total);
            let gpd_id = c.gpd;
            if report_count != 0 {
                self.event(SinkEvent::ApplicationDescription(
                    ApplicationDescriptionReports {
                        gpd_id,
                        total_reports: total,
                        report_count,
                        reports,
                    },
                ));
            }
            Ok(complete)
        }

        /// End the collection, taking the waiting entry.
        fn take_collected(&mut self) -> Option<(SinkEntry, CommissioningReply)> {
            let c = self.pending.collecting.take()?;
            let i = self.pending_index(&c.gpd)?;
            Some((self.pending.swap_remove(i).entry, c.reply))
        }

        /// Commissioning Reply through the TempMaster of the frame that
        /// completed the description; the pairing completes on Success
        /// (as the end of [`Sink::commission`]).
        fn send_reply(
            &mut self,
            entry: SinkEntry,
            rx: &Received<'_>,
            reply: &CommissioningReply,
        ) -> SinkOutcome {
            use SinkDrop::*;
            let Some(temp_master) = rx.temp_master else {
                return SinkOutcome::Dropped(NoTempMaster);
            };
            if !(11..=26).contains(&self.operational_channel) {
                return SinkOutcome::Dropped(NoTempMaster);
            }
            let mut reply_buf = [0u8; MAX_REPLY_LEN];
            let Ok(reply_len) = reply.encode(&mut reply_buf) else {
                return SinkOutcome::Dropped(Invalid);
            };
            if self.pending.is_full() {
                self.pending.remove(0);
            }
            if !self.queue(Out::Response {
                gpd_id: rx.gpd,
                temp_master,
                tx_channel: self.operational_channel - 11,
                reply: reply_buf,
                reply_len: reply_len as u8,
            }) {
                return SinkOutcome::Dropped(QueueFull);
            }
            let _ = self.pending.push(Pending { entry });
            SinkOutcome::CommissioningReplySent
        }
    }

    #[cfg(feature = "translation-table")]
    impl<const N: usize, const Q: usize> Sink<N, Q> {
        /// Extend / Replace with application description: accepted when the
        /// descriptors are well formed.
        pub(in crate::sink) fn refuses_description(&self, a: &ApplicationInformation<'_>) -> bool {
            a.bits & app_info::APPLICATION_DESCRIPTION != 0
                && !a
                    .application_description
                    .is_some_and(|(total, count, reports)| {
                        ApplicationDescription::from_parts(total, count, reports).is_ok()
                    })
        }

        /// Action 0b101: descriptors for the GPD being collected (completing
        /// its pairing once all are known) or for a paired GPD.
        pub(in crate::sink) fn configured_description(
            &mut self,
            pc: &PairingConfiguration<'_>,
        ) -> (u8, Option<SinkOutcome>) {
            let Some(Ok(desc)) = pc
                .application_information
                .and_then(|a| a.application_description)
                .map(|(total, count, reports)| {
                    ApplicationDescription::from_parts(total, count, reports)
                })
            else {
                return (zcl_status::INVALID_FIELD, None);
            };
            if self.is_collecting(&pc.gpd_id) {
                return match self.absorb(&desc) {
                    Err(_) => (zcl_status::INVALID_FIELD, None),
                    Ok(false) => (zcl_status::SUCCESS, Some(SinkOutcome::CommissioningStep)),
                    Ok(true) => match self.take_collected() {
                        Some((entry, _)) => (zcl_status::SUCCESS, Some(self.finalize(entry))),
                        None => (zcl_status::NOT_FOUND, None),
                    },
                };
            }
            if self.find(&pc.gpd_id).is_none() {
                return (zcl_status::NOT_FOUND, None);
            }
            self.report_configured(pc.gpd_id, &desc);
            (zcl_status::SUCCESS, None)
        }

        /// Extend / Replace done: report the descriptors it carried.
        pub(in crate::sink) fn configured_with_description(
            &mut self,
            pc: &PairingConfiguration<'_>,
        ) -> (u8, Option<SinkOutcome>) {
            if let Some(Ok(desc)) = pc
                .application_information
                .and_then(|a| a.application_description)
                .map(|(total, count, reports)| {
                    ApplicationDescription::from_parts(total, count, reports)
                })
            {
                self.report_configured(pc.gpd_id, &desc);
            }
            (zcl_status::SUCCESS, Some(SinkOutcome::Commissioned))
        }

        fn report_configured(&mut self, gpd_id: GpdId, desc: &ApplicationDescription<'_>) {
            if let Ok(reports) = heapless::Vec::from_slice(desc.reports) {
                self.event(SinkEvent::ApplicationDescription(
                    ApplicationDescriptionReports {
                        gpd_id,
                        total_reports: desc.total_reports,
                        report_count: desc.report_count,
                        reports,
                    },
                ));
            }
        }
    }
}

#[cfg(all(test, feature = "app-description"))]
#[path = "descriptions/tests.rs"]
mod tests;
