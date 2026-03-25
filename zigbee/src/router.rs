//! Zigbee Router — frame relay, child management, route discovery.
//!
//! A router:
//! - Relays frames between devices (extends network range)
//! - Participates in route discovery (AODV)
//! - Accepts child devices (end devices and other routers)
//! - Sends link status messages
//! - Maintains neighbor table for its children

use zigbee_types::*;

/// Router configuration.
pub struct RouterConfig {
    /// Maximum number of child end devices.
    pub max_children: u8,
    /// Maximum number of child routers.
    pub max_routers: u8,
    /// Whether to accept join requests.
    pub permit_joining: bool,
    /// Link status period (in seconds).
    pub link_status_period: u16,
}

impl Default for RouterConfig {
    fn default() -> Self {
        Self {
            max_children: 20,
            max_routers: 5,
            permit_joining: false,
            link_status_period: 15,
        }
    }
}

/// Child device record.
#[derive(Debug, Clone)]
pub struct ChildDevice {
    pub ieee_address: IeeeAddress,
    pub short_address: ShortAddress,
    pub is_ffd: bool,
    pub rx_on_when_idle: bool,
    /// Timeout for end device children (seconds).
    pub timeout: u16,
    /// Age since last communication.
    pub age: u16,
    pub active: bool,
}

/// Router state.
pub struct Router {
    config: RouterConfig,
    /// Child devices table.
    children: [Option<ChildDevice>; 32],
    /// Count of active children.
    child_count: u8,
    /// Link status sequence number.
    #[allow(dead_code)]
    link_status_seq: u8,
    /// Whether router has started.
    started: bool,
}

impl Router {
    pub fn new(config: RouterConfig) -> Self {
        Self {
            config,
            children: core::array::from_fn(|_| None),
            child_count: 0,
            link_status_seq: 0,
            started: false,
        }
    }

    /// Add a child device.
    #[allow(clippy::result_unit_err)]
    pub fn add_child(
        &mut self,
        ieee: IeeeAddress,
        short: ShortAddress,
        is_ffd: bool,
        rx_on: bool,
    ) -> Result<(), ()> {
        if self.child_count >= self.config.max_children {
            return Err(());
        }

        if let Some(slot) = self.children.iter_mut().find(|c| c.is_none()) {
            *slot = Some(ChildDevice {
                ieee_address: ieee,
                short_address: short,
                is_ffd,
                rx_on_when_idle: rx_on,
                timeout: 300, // 5 minutes default
                age: 0,
                active: true,
            });
            self.child_count += 1;
            Ok(())
        } else {
            Err(())
        }
    }

    /// Remove a child device by short address.
    pub fn remove_child(&mut self, addr: ShortAddress) {
        if let Some(slot) = self
            .children
            .iter_mut()
            .find(|c| c.as_ref().is_some_and(|d| d.short_address == addr))
        {
            *slot = None;
            self.child_count = self.child_count.saturating_sub(1);
        }
    }

    /// Find a child by short address.
    pub fn find_child(&self, addr: ShortAddress) -> Option<&ChildDevice> {
        self.children
            .iter()
            .flatten()
            .find(|c| c.short_address == addr)
    }

    /// Find a child by IEEE address.
    pub fn find_child_by_ieee(&self, ieee: &IeeeAddress) -> Option<&ChildDevice> {
        self.children
            .iter()
            .flatten()
            .find(|c| &c.ieee_address == ieee)
    }

    /// Check if a frame destination is one of our children.
    pub fn is_child(&self, addr: ShortAddress) -> bool {
        self.children
            .iter()
            .flatten()
            .any(|c| c.short_address == addr)
    }

    /// Whether we can accept more children.
    pub fn can_accept_child(&self) -> bool {
        self.child_count < self.config.max_children
    }

    /// Age all children; remove timed-out ones.
    pub fn age_children(&mut self, elapsed_seconds: u16) {
        for slot in self.children.iter_mut() {
            if let Some(child) = slot {
                child.age = child.age.saturating_add(elapsed_seconds);
                if child.age > child.timeout && !child.rx_on_when_idle {
                    // Sleepy end device timed out
                    *slot = None;
                    self.child_count = self.child_count.saturating_sub(1);
                }
            }
        }
    }

    /// Record activity from a child (reset age).
    pub fn child_activity(&mut self, addr: ShortAddress) {
        if let Some(child) = self
            .children
            .iter_mut()
            .flatten()
            .find(|c| c.short_address == addr)
        {
            child.age = 0;
        }
    }

    /// Number of active children.
    pub fn child_count(&self) -> u8 {
        self.child_count
    }

    /// Whether the router has started.
    pub fn is_started(&self) -> bool {
        self.started
    }

    /// Mark router as started.
    pub fn mark_started(&mut self) {
        self.started = true;
    }
}
