//! The first panic of callbacks Jolt calls during a step, kept until the world resumes it.

use std::any::Any;
use std::mem;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Mutex, PoisonError};

/// The first panic of a callback, kept until the world resumes it.
#[derive(Default)]
pub(crate) struct PanicSlot {
    panicked: AtomicBool,
    payload: Mutex<Option<Box<dyn Any + Send>>>,
}

impl PanicSlot {
    /// Keeps `payload` when it is the first, and drops later ones without letting their drop
    /// unwind.
    pub(crate) fn record(&self, payload: Box<dyn Any + Send>) {
        let first = self
            .panicked
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .is_ok();
        if first {
            *self.payload.lock().unwrap_or_else(PoisonError::into_inner) = Some(payload);
        } else {
            drop_payload(payload);
        }
    }

    /// Whether a payload was recorded since the slot was last taken.
    pub(crate) fn has_panicked(&self) -> bool {
        self.panicked.load(Ordering::Acquire)
    }

    /// The kept payload, after which the slot is empty again.
    pub(crate) fn take(&self) -> Option<Box<dyn Any + Send>> {
        let payload = self
            .payload
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .take();
        self.panicked.store(false, Ordering::Release);
        payload
    }
}

/// Drops a panic payload whose own `Drop` may panic, without unwinding.
fn drop_payload(payload: Box<dyn Any + Send>) {
    if let Err(panic_in_drop) = catch_unwind(AssertUnwindSafe(|| drop(payload))) {
        mem::forget(panic_in_drop);
    }
}

/// Keeps the first of two payloads and drops the other.
pub(crate) fn first_payload(
    first: Option<Box<dyn Any + Send>>,
    second: Option<Box<dyn Any + Send>>,
) -> Option<Box<dyn Any + Send>> {
    match (first, second) {
        (Some(first), Some(second)) => {
            drop_payload(second);
            Some(first)
        }
        (first, second) => first.or(second),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A payload whose drop panics.
    struct PanicsOnDrop;

    impl Drop for PanicsOnDrop {
        fn drop(&mut self) {
            panic!("panic payload dropped");
        }
    }

    fn payload(value: u32) -> Box<dyn Any + Send> {
        Box::new(value)
    }

    #[test]
    fn a_later_payload_whose_drop_panics_is_dropped_without_unwinding() {
        let slot = PanicSlot::default();
        slot.record(payload(1));
        slot.record(Box::new(PanicsOnDrop));
        assert_eq!(slot.take().unwrap().downcast_ref::<u32>(), Some(&1));
        let kept = first_payload(Some(payload(2)), Some(Box::new(PanicsOnDrop)));
        assert_eq!(kept.unwrap().downcast_ref::<u32>(), Some(&2));
    }

    #[test]
    fn take_empties_the_slot() {
        let slot = PanicSlot::default();
        assert!(!slot.has_panicked());
        slot.record(payload(1));
        assert!(slot.has_panicked());
        assert!(slot.take().is_some());
        assert!(!slot.has_panicked());
        assert!(slot.take().is_none());
        slot.record(payload(2));
        assert_eq!(slot.take().unwrap().downcast_ref::<u32>(), Some(&2));
    }
}
