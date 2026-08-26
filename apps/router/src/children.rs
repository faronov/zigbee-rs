//! Compile-time child-table capabilities.

/// Non-parent frontends have no child-table storage or lifecycle.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct NoChildren;

/// Concrete child-table storage owned by a parent router or coordinator.
#[derive(Debug)]
pub struct PersistentChildren<C> {
    store: C,
}

impl<C> PersistentChildren<C> {
    pub const fn new(store: C) -> Self {
        Self { store }
    }

    pub const fn store(&self) -> &C {
        &self.store
    }

    pub fn store_mut(&mut self) -> &mut C {
        &mut self.store
    }

    pub fn into_inner(self) -> C {
        self.store
    }
}
