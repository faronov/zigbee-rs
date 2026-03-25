//! Zigbee Coordinator — network formation and Trust Center management.
//!
//! A coordinator:
//! - Forms the network (selects channel + PAN ID)
//! - Acts as the Trust Center (distributes network key)
//! - Assigns addresses to joining devices
//! - Manages permit-joining state
//! - Routes frames (also a router)

use zigbee_types::*;

/// Coordinator configuration.
pub struct CoordinatorConfig {
    /// Channel mask for formation (ED scan).
    pub channel_mask: ChannelMask,
    /// Extended PAN ID (0 = auto-generate from IEEE address).
    pub extended_pan_id: IeeeAddress,
    /// Whether to use centralized security (Trust Center).
    pub centralized_security: bool,
    /// Whether to use install codes for joining.
    pub require_install_codes: bool,
    /// Maximum number of child devices.
    pub max_children: u8,
    /// Maximum network depth.
    pub max_depth: u8,
    /// Default permit-join duration after formation (seconds, 0=closed).
    pub initial_permit_join_duration: u8,
}

impl Default for CoordinatorConfig {
    fn default() -> Self {
        Self {
            channel_mask: ChannelMask::ALL_2_4GHZ,
            extended_pan_id: [0; 8],
            centralized_security: true,
            require_install_codes: false,
            max_children: 20,
            max_depth: 5,
            initial_permit_join_duration: 0,
        }
    }
}

/// Coordinator state.
pub struct Coordinator {
    config: CoordinatorConfig,
    /// Network key (set during formation).
    network_key: [u8; 16],
    /// Frame counter for outgoing secured frames.
    frame_counter: u32,
    /// Number of joined children.
    child_count: u8,
    /// Next short address to assign (stochastic: random, but tracked).
    next_address_seed: u16,
    /// Whether the network has been formed.
    formed: bool,
}

impl Coordinator {
    pub fn new(config: CoordinatorConfig) -> Self {
        Self {
            config,
            network_key: [0; 16],
            frame_counter: 0,
            child_count: 0,
            next_address_seed: 1,
            formed: false,
        }
    }

    /// Generate a network key (should use hardware RNG).
    pub fn generate_network_key(&mut self) {
        // TODO: use platform-specific hardware RNG
        // Placeholder: deterministic for testing
        self.network_key = [
            0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x0F, 0x00, 0x02, 0x04, 0x06, 0x08, 0x0A,
            0x0C, 0x0E,
        ];
    }

    /// Get the network key.
    pub fn network_key(&self) -> &[u8; 16] {
        &self.network_key
    }

    /// Set a specific network key.
    pub fn set_network_key(&mut self, key: [u8; 16]) {
        self.network_key = key;
    }

    /// Allocate a short address for a joining device (stochastic method).
    pub fn allocate_address(&mut self) -> ShortAddress {
        // Stochastic address assignment — avoids tree topology constraints
        // Real impl: generate random, check for conflicts, retry
        let addr = self.next_address_seed;
        self.next_address_seed = self.next_address_seed.wrapping_add(1);
        if self.next_address_seed == 0 || self.next_address_seed >= 0xFFF8 {
            self.next_address_seed = 1;
        }
        self.child_count += 1;
        ShortAddress(addr)
    }

    /// Check if we can accept more children.
    pub fn can_accept_child(&self) -> bool {
        self.child_count < self.config.max_children
    }

    /// Whether the network is formed.
    pub fn is_formed(&self) -> bool {
        self.formed
    }

    /// Mark network as formed.
    pub fn mark_formed(&mut self) {
        self.formed = true;
    }

    /// Get the next frame counter value (and increment).
    pub fn next_frame_counter(&mut self) -> u32 {
        let fc = self.frame_counter;
        self.frame_counter += 1;
        fc
    }
}
