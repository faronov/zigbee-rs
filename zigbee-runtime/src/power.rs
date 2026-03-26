//! Power management for Zigbee devices.
//!
//! Manages sleep/wake cycles for Sleepy End Devices (SEDs):
//! - Poll Control cluster integration
//! - Sleep scheduling based on reporting intervals
//! - MAC poll timing
//! - Deep sleep with wake-on-timer or wake-on-GPIO

/// Power mode for the device.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PowerMode {
    /// Always on — router or mains-powered end device.
    AlwaysOn,
    /// Sleepy End Device — periodic wake for polling.
    Sleepy {
        /// Poll interval in milliseconds.
        poll_interval_ms: u32,
        /// How long to stay awake after activity (ms).
        wake_duration_ms: u32,
    },
    /// Deep sleep — wake only on timer or external event.
    DeepSleep {
        /// Wake interval in seconds.
        wake_interval_s: u32,
    },
}

/// Sleep decision made by the power manager.
#[derive(Debug, Clone, Copy)]
pub enum SleepDecision {
    /// Stay awake — pending work.
    StayAwake,
    /// Light sleep for the given duration (ms). CPU halted, RAM retained.
    LightSleep(u32),
    /// Deep sleep for the given duration (ms). Only RTC + wake sources active.
    DeepSleep(u32),
}

/// Power manager state.
pub struct PowerManager {
    mode: PowerMode,
    last_activity_ms: u32,
    last_poll_ms: u32,
    pending_tx: bool,
    pending_reports: bool,
}

impl PowerManager {
    pub fn new(mode: PowerMode) -> Self {
        Self {
            mode,
            last_activity_ms: 0,
            last_poll_ms: 0,
            pending_tx: false,
            pending_reports: false,
        }
    }

    /// Get the current power mode.
    pub fn mode(&self) -> PowerMode {
        self.mode
    }

    /// Record that activity occurred (rx/tx, user input, sensor read).
    pub fn record_activity(&mut self, now_ms: u32) {
        self.last_activity_ms = now_ms;
    }

    /// Record that a MAC poll was sent.
    pub fn record_poll(&mut self, now_ms: u32) {
        self.last_poll_ms = now_ms;
    }

    /// Set whether there are pending transmissions.
    pub fn set_pending_tx(&mut self, pending: bool) {
        self.pending_tx = pending;
    }

    /// Set whether there are pending attribute reports.
    pub fn set_pending_reports(&mut self, pending: bool) {
        self.pending_reports = pending;
    }

    /// Decide whether to sleep based on current state.
    pub fn decide(&self, now_ms: u32) -> SleepDecision {
        // Never sleep if pending work
        if self.pending_tx || self.pending_reports {
            return SleepDecision::StayAwake;
        }

        match self.mode {
            PowerMode::AlwaysOn => SleepDecision::StayAwake,

            PowerMode::Sleepy {
                poll_interval_ms,
                wake_duration_ms,
            } => {
                let since_activity = now_ms.wrapping_sub(self.last_activity_ms);
                if since_activity < wake_duration_ms {
                    return SleepDecision::StayAwake;
                }

                let since_poll = now_ms.wrapping_sub(self.last_poll_ms);
                if since_poll >= poll_interval_ms {
                    // Need to poll soon
                    return SleepDecision::StayAwake;
                }

                let sleep_until_poll = poll_interval_ms - since_poll;
                SleepDecision::LightSleep(sleep_until_poll)
            }

            PowerMode::DeepSleep { wake_interval_s } => {
                let since_activity = now_ms.wrapping_sub(self.last_activity_ms);
                if since_activity < 1000 {
                    return SleepDecision::StayAwake;
                }
                SleepDecision::DeepSleep(wake_interval_s * 1000)
            }
        }
    }

    /// Check if it's time to send a MAC poll (for SEDs).
    pub fn should_poll(&self, now_ms: u32) -> bool {
        match self.mode {
            PowerMode::Sleepy {
                poll_interval_ms, ..
            } => now_ms.wrapping_sub(self.last_poll_ms) >= poll_interval_ms,
            _ => false,
        }
    }
}

impl Default for PowerManager {
    fn default() -> Self {
        Self::new(PowerMode::AlwaysOn)
    }
}
