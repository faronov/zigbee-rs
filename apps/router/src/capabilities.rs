//! Narrow composition-root capabilities for status and supervision.

/// The statically selected public application archetype.
///
/// This value is used only for semantic status and diagnostics. The actual
/// runtime behavior is selected by the concrete public frontend and
/// `zigbee-runtime` role type, never by matching this enum.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NodeArchetype {
    /// A mains-powered non-routing end device with its receiver on while idle.
    AlwaysOnEndDevice,
    RelayRouter,
    ParentRouter,
    Coordinator,
}

/// Product-independent router lifecycle status.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RouterStatus {
    Starting {
        archetype: NodeArchetype,
    },
    Commissioning {
        archetype: NodeArchetype,
        attempt: u32,
    },
    Online {
        archetype: NodeArchetype,
        short_address: u16,
        identifying: bool,
    },
    Rejoining {
        archetype: NodeArchetype,
        failures: u8,
    },
    Recommissioning {
        archetype: NodeArchetype,
        attempt: u32,
        retry_in_ms: u32,
    },
    Resetting {
        archetype: NodeArchetype,
    },
    Fault {
        archetype: NodeArchetype,
    },
}

/// Maps semantic lifecycle state to fitted product indicators.
pub trait StatusSink {
    /// Whether this product has a fitted status indicator.
    const PRESENT: bool = true;

    fn set(&mut self, status: RouterStatus);
}

#[derive(Debug, Default, Clone, Copy)]
pub struct NoStatus;

impl StatusSink for NoStatus {
    const PRESENT: bool = false;

    fn set(&mut self, _status: RouterStatus) {}
}

/// Reset and watchdog supervision supplied by the composition root.
pub trait Supervisor {
    /// Feed or service the watchdog after useful application progress.
    fn heartbeat(&mut self);

    /// Maximum safe blocking interval before the watchdog must be serviced.
    ///
    /// The app caps receive and retry waits to this value. `None` means there
    /// is no watchdog-imposed bound beyond [`crate::RouterPolicy`].
    fn max_wait_ms(&self) -> Option<u32>;

    /// Reset after an unrecoverable application error.
    fn reset(&mut self) -> !;
}

/// No-op supervision for host tests and products without a watchdog.
#[derive(Debug, Default, Clone, Copy)]
pub struct NoSupervisor;

impl Supervisor for NoSupervisor {
    fn heartbeat(&mut self) {}

    fn max_wait_ms(&self) -> Option<u32> {
        None
    }

    fn reset(&mut self) -> ! {
        loop {
            core::hint::spin_loop();
        }
    }
}
