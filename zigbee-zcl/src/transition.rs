//! Generic attribute transition engine for ZCL clusters.
//! Supports smooth transitions for Level Control and Color Control.

/// A single active transition on one attribute.
#[derive(Debug, Clone)]
pub struct Transition {
    pub start_value: i32,
    pub target_value: i32,
    pub remaining_ds: u16,
    pub total_ds: u16,
}

impl Transition {
    pub fn new(start: i32, target: i32, time_ds: u16) -> Self {
        Self {
            start_value: start,
            target_value: target,
            remaining_ds: time_ds,
            total_ds: time_ds,
        }
    }

    /// Advance the transition by `elapsed_ds` deciseconds.
    /// Returns the current interpolated value.
    pub fn tick(&mut self, elapsed_ds: u16) -> i32 {
        if self.remaining_ds <= elapsed_ds {
            self.remaining_ds = 0;
            return self.target_value;
        }
        self.remaining_ds -= elapsed_ds;
        if self.total_ds == 0 {
            return self.target_value;
        }
        let elapsed_total = (self.total_ds - self.remaining_ds) as i64;
        let total = self.total_ds as i64;
        let delta = self.target_value as i64 - self.start_value as i64;
        (self.start_value as i64 + delta * elapsed_total / total) as i32
    }

    pub fn is_complete(&self) -> bool {
        self.remaining_ds == 0
    }
}

/// Manages up to N concurrent transitions (one per attribute being transitioned).
#[derive(Debug, Clone)]
pub struct TransitionManager<const N: usize> {
    transitions: heapless::Vec<(u16, Transition), N>,
}

impl<const N: usize> Default for TransitionManager<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> TransitionManager<N> {
    pub fn new() -> Self {
        Self {
            transitions: heapless::Vec::new(),
        }
    }

    /// Start a transition for the given attribute. Replaces any existing.
    pub fn start(&mut self, attr_id: u16, start: i32, target: i32, time_ds: u16) {
        // Remove existing transition for this attribute
        self.stop(attr_id);
        let t = Transition::new(start, target, time_ds);
        let _ = self.transitions.push((attr_id, t));
    }

    /// Stop a specific attribute's transition.
    pub fn stop(&mut self, attr_id: u16) {
        if let Some(pos) = self.transitions.iter().position(|(id, _)| *id == attr_id) {
            self.transitions.swap_remove(pos);
        }
    }

    /// Stop all transitions.
    pub fn stop_all(&mut self) {
        self.transitions.clear();
    }

    /// Tick all active transitions. Returns list of (attr_id, current_value) that changed.
    pub fn tick(&mut self, elapsed_ds: u16) -> heapless::Vec<(u16, i32), N> {
        let mut results: heapless::Vec<(u16, i32), N> = heapless::Vec::new();
        for (attr_id, transition) in self.transitions.iter_mut() {
            let val = transition.tick(elapsed_ds);
            let _ = results.push((*attr_id, val));
        }
        // Remove completed transitions
        self.transitions.retain(|(_, t)| !t.is_complete());
        results
    }

    /// Check if any transitions are active.
    pub fn is_active(&self) -> bool {
        !self.transitions.is_empty()
    }

    /// Get remaining time for a specific attribute transition.
    pub fn remaining_ds(&self, attr_id: u16) -> u16 {
        self.transitions
            .iter()
            .find(|(id, _)| *id == attr_id)
            .map(|(_, t)| t.remaining_ds)
            .unwrap_or(0)
    }

    /// Get the maximum remaining time across all transitions.
    pub fn max_remaining_ds(&self) -> u16 {
        self.transitions
            .iter()
            .map(|(_, t)| t.remaining_ds)
            .max()
            .unwrap_or(0)
    }
}
