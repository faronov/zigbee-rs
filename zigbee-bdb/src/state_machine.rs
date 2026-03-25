//! BDB top-level commissioning state machine (BDB v3.0.1 spec §8.1–§8.2).
//!
//! The state machine orchestrates the four commissioning methods in priority
//! order: Touchlink → Steering → Formation → Finding & Binding.
//!
//! ```text
//!                         ┌──────────┐
//!              ┌─────────►│   Idle   │◄────────────────┐
//!              │          └────┬─────┘                  │
//!              │               │ commission()           │
//!              │          ┌────▼──────────┐             │
//!              │          │ Initializing  │             │
//!              │          └────┬──────────┘             │
//!              │               │                        │
//!              │       ┌───────▼────────┐               │
//!              │  TL?  │   Touchlink    │──► fail ──┐   │
//!              │       └───────┬────────┘           │   │
//!              │               │ skip/done          │   │
//!              │       ┌───────▼────────┐           │   │
//!              │  NS?  │ NetworkSteering│──► fail ──┤   │
//!              │       └───────┬────────┘           │   │
//!              │               │ skip/done          │   │
//!              │       ┌───────▼────────┐           │   │
//!              │  NF?  │NetworkFormation│──► fail ──┤   │
//!              │       └───────┬────────┘           │   │
//!              │               │ skip/done          │   │
//!              │       ┌───────▼────────┐           │   │
//!              │  FB?  │FindingBinding  │──► fail ──┘   │
//!              │       └───────┬────────┘               │
//!              │               │                        │
//!              └───────────────┴────────────────────────┘
//! ```

use zigbee_mac::MacDriver;
use zigbee_nwk::DeviceType;

use crate::{BdbLayer, BdbStatus};

// ── Commissioning mode bitmask ──────────────────────────────

/// Bitmask of enabled commissioning methods (BDB spec Table 5).
///
/// The application sets this before calling [`BdbLayer::commission`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CommissioningMode(pub u8);

impl CommissioningMode {
    /// Touchlink commissioning (BDB §8.7)
    pub const TOUCHLINK: Self = Self(0x01);
    /// Network Steering (BDB §8.3) — join existing network / open permit joining
    pub const STEERING: Self = Self(0x02);
    /// Network Formation (BDB §8.4) — create new PAN (coordinator only)
    pub const FORMATION: Self = Self(0x04);
    /// Finding & Binding (BDB §8.5) — EZ-Mode automatic binding
    pub const FINDING_BINDING: Self = Self(0x08);
    /// All methods enabled
    pub const ALL: Self = Self(0x0F);

    pub const fn contains(self, other: Self) -> bool {
        (self.0 & other.0) != 0
    }

    pub const fn or(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }

    pub const fn empty() -> Self {
        Self(0)
    }

    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }
}

// ── BDB state ───────────────────────────────────────────────

/// Current state of the BDB commissioning state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BdbState {
    /// No commissioning in progress.
    Idle,
    /// Running BDB initialisation (spec §8.1).
    Initializing,
    /// Network Steering is in progress (spec §8.3).
    NetworkSteering,
    /// Network Formation is in progress (spec §8.4).
    NetworkFormation,
    /// Finding & Binding is in progress (spec §8.5).
    FindingBinding,
    /// Touchlink commissioning is in progress (spec §8.7).
    Touchlink,
}

// ── State machine implementation ────────────────────────────

impl<M: MacDriver> BdbLayer<M> {
    /// BDB initialisation procedure (BDB spec §8.1).
    ///
    /// Must be called once after power-on/reset before any commissioning.
    /// Sets up the device-type–dependent commissioning capabilities and
    /// optionally restores network state from NV storage.
    pub async fn initialize(&mut self) -> Result<(), BdbStatus> {
        self.state = BdbState::Initializing;
        log::info!("[BDB] Initializing…");

        // Reset lower layers
        if self.zdo.nlme_reset(false).await.is_err() {
            self.state = BdbState::Idle;
            return Err(BdbStatus::NotPermitted);
        }

        // Determine commissioning capabilities based on device type
        let device_type = self.zdo.nwk().device_type();
        let cap = match device_type {
            DeviceType::Coordinator => CommissioningMode::STEERING
                .or(CommissioningMode::FORMATION)
                .or(CommissioningMode::FINDING_BINDING),
            DeviceType::Router => CommissioningMode::STEERING
                .or(CommissioningMode::FINDING_BINDING)
                .or(CommissioningMode::TOUCHLINK),
            DeviceType::EndDevice => CommissioningMode::STEERING
                .or(CommissioningMode::FINDING_BINDING)
                .or(CommissioningMode::TOUCHLINK),
        };
        self.attributes.node_commissioning_capability = cap;

        // Sync on-network state with NWK layer
        self.attributes.node_is_on_a_network = self.zdo.nwk().is_joined();

        self.state = BdbState::Idle;
        log::info!(
            "[BDB] Initialized (type={:?}, cap=0x{:02X})",
            device_type,
            cap.0
        );
        Ok(())
    }

    /// Top-level commissioning dispatcher (BDB spec §8.2).
    ///
    /// Runs each enabled commissioning method in the spec-defined order:
    /// 1. Touchlink (if enabled)
    /// 2. Network Steering (if enabled)
    /// 3. Network Formation (if enabled and coordinator-capable)
    /// 4. Finding & Binding (if enabled and on a network)
    ///
    /// Returns `Ok` if at least one method succeeded.
    pub async fn commission(&mut self) -> Result<(), BdbStatus> {
        let mode = self.attributes.commissioning_mode;
        log::info!("[BDB] Commissioning start (mode=0x{:02X})", mode.0);

        let mut any_success = false;

        // ── 1. Touchlink ────────────────────────────────────
        if mode.contains(CommissioningMode::TOUCHLINK) {
            self.state = BdbState::Touchlink;
            match self.touchlink_commissioning().await {
                Ok(()) => {
                    log::info!("[BDB] Touchlink succeeded");
                    any_success = true;
                }
                Err(e) => {
                    log::warn!("[BDB] Touchlink failed: {:?}", e);
                }
            }
        }

        // ── 2. Network Steering ─────────────────────────────
        if mode.contains(CommissioningMode::STEERING) {
            self.state = BdbState::NetworkSteering;
            match self.network_steering().await {
                Ok(()) => {
                    log::info!("[BDB] Network Steering succeeded");
                    any_success = true;
                }
                Err(e) => {
                    log::warn!("[BDB] Network Steering failed: {:?}", e);
                }
            }
        }

        // ── 3. Network Formation ────────────────────────────
        if mode.contains(CommissioningMode::FORMATION) {
            self.state = BdbState::NetworkFormation;
            match self.network_formation().await {
                Ok(()) => {
                    log::info!("[BDB] Network Formation succeeded");
                    any_success = true;
                }
                Err(e) => {
                    log::warn!("[BDB] Network Formation failed: {:?}", e);
                }
            }
        }

        // ── 4. Finding & Binding ────────────────────────────
        if mode.contains(CommissioningMode::FINDING_BINDING) {
            self.state = BdbState::FindingBinding;
            match self.finding_binding_initiator(1).await {
                Ok(()) => {
                    log::info!("[BDB] Finding & Binding succeeded");
                    any_success = true;
                }
                Err(e) => {
                    log::warn!("[BDB] Finding & Binding failed: {:?}", e);
                }
            }
        }

        self.state = BdbState::Idle;

        if any_success {
            Ok(())
        } else {
            Err(BdbStatus::SteeringFailure)
        }
    }
}
