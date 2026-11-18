//! Radio-owned suspend boundary, also exercised by source-linked host tests.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SuspendError {
    /// An application TX or hardware acknowledgement is still active.
    Busy,
    /// A received frame must be delivered before reconsidering sleep.
    PendingReceive,
    /// The interrupt handler has unprocessed events.
    PendingEvents,
}

pub(crate) trait Boundary {
    fn busy(&self) -> bool;
    fn queued(&self) -> bool;
    fn events_pending(&self) -> bool;
    fn stop(&mut self);
    /// Preserve completed RX after STOP and clear only handled stop events.
    fn finish_stop(&mut self) -> bool;
    fn resume_receive(&mut self);
    fn suspend(&mut self);
}

pub(crate) fn try_suspend(boundary: &mut impl Boundary) -> Result<(), SuspendError> {
    if boundary.busy() {
        return Err(SuspendError::Busy);
    }
    if boundary.queued() {
        return Err(SuspendError::PendingReceive);
    }
    if boundary.events_pending() {
        return Err(SuspendError::PendingEvents);
    }
    boundary.stop();
    if boundary.finish_stop() {
        boundary.resume_receive();
        return Err(SuspendError::PendingReceive);
    }
    boundary.suspend();
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Default)]
    struct Fake {
        busy: bool,
        queued: bool,
        pending: bool,
        raced_rx: bool,
        stopped: bool,
        resumed: bool,
        asleep: bool,
    }

    impl Boundary for Fake {
        fn busy(&self) -> bool {
            self.busy
        }
        fn queued(&self) -> bool {
            self.queued
        }
        fn events_pending(&self) -> bool {
            self.pending
        }
        fn stop(&mut self) {
            self.stopped = true;
        }
        fn finish_stop(&mut self) -> bool {
            assert!(self.stopped);
            self.queued |= self.raced_rx;
            self.raced_rx
        }
        fn resume_receive(&mut self) {
            self.resumed = true;
        }
        fn suspend(&mut self) {
            assert!(self.stopped && !self.queued);
            self.asleep = true;
        }
    }

    #[test]
    fn transmit_and_ack_are_never_stopped() {
        let mut hw = Fake {
            busy: true,
            ..Fake::default()
        };
        assert_eq!(try_suspend(&mut hw), Err(SuspendError::Busy));
        assert!(!hw.stopped && !hw.asleep);
    }

    #[test]
    fn queued_frame_vetoes_without_touching_hardware() {
        let mut hw = Fake {
            queued: true,
            ..Fake::default()
        };
        assert_eq!(try_suspend(&mut hw), Err(SuspendError::PendingReceive));
        assert!(!hw.stopped && hw.queued);
    }

    #[test]
    fn pending_interrupt_must_run_first() {
        let mut hw = Fake {
            pending: true,
            ..Fake::default()
        };
        assert_eq!(try_suspend(&mut hw), Err(SuspendError::PendingEvents));
        assert!(!hw.stopped);
    }

    #[test]
    fn receive_racing_stop_is_preserved_and_prevents_sleep() {
        let mut hw = Fake {
            raced_rx: true,
            ..Fake::default()
        };
        assert_eq!(try_suspend(&mut hw), Err(SuspendError::PendingReceive));
        assert!(hw.stopped && hw.queued && hw.resumed && !hw.asleep);
    }

    #[test]
    fn quiet_receiver_can_suspend() {
        let mut hw = Fake::default();
        assert_eq!(try_suspend(&mut hw), Ok(()));
        assert!(hw.stopped && hw.asleep && !hw.resumed);
    }
}
