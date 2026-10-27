//! The tire max impulse callback of wheeled vehicles and motorcycles (Jolt
//! `WheeledVehicleController::SetTireMaxImpulseCallback`).

use std::any::Any;
use std::ffi::c_void;
use std::panic::{catch_unwind, AssertUnwindSafe};

use oxijolt_sys::{
    JPH_WheeledVehicleController, JPH_WheeledVehicleController_ResetTireMaxImpulseCallback,
    JPH_WheeledVehicleController_SetTireMaxImpulseCallback,
};

use super::{VehicleMut, VehicleRef, WheeledControllerKind};
use crate::panic_slot::PanicSlot;
use crate::{limits, VehicleError};

/// What Jolt hands a tire max impulse callback for one wheel with ground contact (see
/// [`VehicleMut::set_tire_max_impulse_callback`]).
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct TireMaxImpulseInput {
    /// Index of the wheel in the vehicle's wheel list.
    pub wheel: u32,
    /// Impulse of the suspension in this collision step, N·s.
    pub suspension_impulse: f32,
    /// Friction coefficient in the wheel's forward direction: the wheel's longitudinal friction
    /// curve at [`longitudinal_slip`](Self::longitudinal_slip), combined with the ground's
    /// friction.
    pub longitudinal_friction: f32,
    /// Friction coefficient in the wheel's sideways direction: the wheel's lateral friction
    /// curve at [`lateral_slip`](Self::lateral_slip), combined with the ground's friction.
    pub lateral_friction: f32,
    /// `|(ω r − v) / v|`, with `ω r` the speed of the wheel's rim and `v` the forward speed of
    /// the ground relative to the wheel, at least 0.001 m/s in magnitude. 0 is rolling without
    /// slip.
    pub longitudinal_slip: f32,
    /// Angle between the wheel's forward direction and the ground velocity relative to the
    /// wheel, radians in `[0, π/2]`; 0 below a relative speed of 0.001 m/s.
    pub lateral_slip: f32,
    /// Length of the collision step, s.
    pub delta_time: f32,
}

impl TireMaxImpulseInput {
    /// Jolt's own caps, which a vehicle without a callback uses: each friction coefficient times
    /// the suspension impulse.
    pub fn default_max_impulse(&self) -> TireMaxImpulse {
        TireMaxImpulse {
            longitudinal: self.longitudinal_friction * self.suspension_impulse,
            lateral: self.lateral_friction * self.suspension_impulse,
        }
    }
}

/// The caps a tire max impulse callback returns for one wheel, N·s. They bound the impulses the
/// tire may apply on the ground in this collision step; the impulses Jolt applies can be lower.
/// NaN and negative values count as 0; [`f32::INFINITY`] is no cap. A vehicle takes a callback
/// only while its wheels are light against its chassis ([`limits::MAX_WHEEL_MASS_RATIO`]).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TireMaxImpulse {
    /// Cap on the impulse along the wheel's forward direction, drive and brakes.
    pub longitudinal: f32,
    /// Cap on the impulse along the wheel's sideways direction.
    pub lateral: f32,
}

impl TireMaxImpulse {
    /// The caps with NaN and negative values replaced by 0.
    fn sanitized(self) -> Self {
        // `f32::max` returns the other operand for NaN.
        Self {
            longitudinal: self.longitudinal.max(0.0),
            lateral: self.lateral.max(0.0),
        }
    }
}

type TireMaxImpulseFn = dyn Fn(&TireMaxImpulseInput) -> TireMaxImpulse + Send + Sync;

/// A vehicle's tire max impulse callback, whose address is the `userData` Jolt passes to
/// [`tire_max_impulse`].
pub(super) struct TireCallback {
    callback: Box<TireMaxImpulseFn>,
    panic: PanicSlot,
}

impl TireCallback {
    /// The first panic of the callback since the last call, after which it runs again.
    pub(super) fn take_panic(&self) -> Option<Box<dyn Any + Send>> {
        self.panic.take()
    }
}

/// The `JPH_TireMaxImpulseCallback` of every vehicle with a Rust callback. Writes both caps on
/// every path: the callback's, sanitized, or Jolt's default once the callback panicked in this
/// step. A panic is kept for the world to resume and never unwinds into Jolt.
///
/// # Safety
/// `user_data` is the address of a [`TireCallback`] that lives for the call. The out pointers
/// point to Jolt's writable locals of the call (`WheeledVehicleController.cpp:670-676`).
#[allow(clippy::too_many_arguments)]
unsafe extern "C" fn tire_max_impulse(
    user_data: *mut c_void,
    wheel: u32,
    out_longitudinal: *mut f32,
    out_lateral: *mut f32,
    suspension_impulse: f32,
    longitudinal_friction: f32,
    lateral_friction: f32,
    longitudinal_slip: f32,
    lateral_slip: f32,
    delta_time: f32,
) {
    // SAFETY: the address of a live `TireCallback` (contract), which is only ever shared.
    let tire = unsafe { &*user_data.cast::<TireCallback>() };
    let input = TireMaxImpulseInput {
        wheel,
        suspension_impulse,
        longitudinal_friction,
        lateral_friction,
        longitudinal_slip,
        lateral_slip,
        delta_time,
    };
    let default = input.default_max_impulse();
    let caps = if tire.panic.has_panicked() {
        default
    } else {
        match catch_unwind(AssertUnwindSafe(|| (tire.callback)(&input))) {
            Ok(caps) => caps.sanitized(),
            Err(payload) => {
                tire.panic.record(payload);
                default
            }
        }
    };
    // SAFETY: both pointers are Jolt's writable locals of this call (contract).
    unsafe {
        out_longitudinal.write(caps.longitudinal);
        out_lateral.write(caps.lateral);
    }
}

impl<K: WheeledControllerKind> VehicleRef<'_, K> {
    /// Whether the vehicle has a tire max impulse callback (see
    /// [`VehicleMut::set_tire_max_impulse_callback`]).
    pub fn has_tire_max_impulse_callback(&self) -> bool {
        self.entry.tire_callback.is_some()
    }
}

impl<K: WheeledControllerKind> VehicleMut<'_, K> {
    fn wheeled_controller(&mut self) -> *mut JPH_WheeledVehicleController {
        // A wheeled vehicle's controller is a `WheeledVehicleController` and a motorcycle's a
        // `MotorcycleController` derived from it (`K: WheeledControllerKind`), both with single
        // inheritance from `VehicleController`.
        self.controller().cast()
    }

    /// Replaces how the vehicle caps its tire impulses (Jolt
    /// `WheeledVehicleController::SetTireMaxImpulseCallback`); without a callback the caps are
    /// [`TireMaxImpulseInput::default_max_impulse`]. A callback replaces the one set before.
    ///
    /// Jolt calls `callback` for each wheel with ground contact in every velocity iteration the
    /// solver runs, in every collision step. The number of calls is not fixed: the solver stops
    /// iterating early when no impulse changes. The calls run on the solver's threads,
    /// concurrently with other vehicles' callbacks, never with another call of the same
    /// vehicle. For results that do not depend on the thread count, the result must depend
    /// only on the input and on data that does not change during the step.
    ///
    /// A panic in `callback` makes the vehicle use Jolt's default caps for the rest of that
    /// step, and [`step`](crate::PhysicsWorld::step) resumes the panic afterwards. A panic
    /// that aborts the process is not caught.
    ///
    /// The callback and what it captures are not part of [`WorldState`](crate::WorldState):
    /// [`restore_state`](crate::PhysicsWorld::restore_state) keeps the callback and does not
    /// rewind its captured state. [`rebase`](crate::PhysicsWorld::rebase) keeps it too, since
    /// its input does not depend on the frame. The callback is dropped with the vehicle.
    ///
    /// # Errors
    /// [`VehicleError::InvalidValue`] when a wheel is heavy against the chassis
    /// ([`limits::MAX_WHEEL_MASS_RATIO`]); the vehicle keeps Jolt's default caps.
    ///
    /// # Example
    /// A tire with 80 % of Jolt's grip sideways:
    /// ```
    /// use oxijolt::*;
    ///
    /// # fn main() -> Result<(), Box<dyn std::error::Error>> {
    /// let mut world = PhysicsWorld::new(WorldSettings::default())?;
    /// let floor = Shape::new_box(Vec3::new(50.0, 1.0, 50.0))?;
    /// world.create_body(&floor, &BodySettings::new_static().position(RVec3::new(0.0, -1.0, 0.0)))?;
    /// let hull = Shape::new_box(Vec3::new(0.9, 0.3, 2.0))?;
    /// let chassis = world.create_body(
    ///     &hull,
    ///     &BodySettings::new_dynamic().position(RVec3::new(0.0, 1.0, 0.0)).mass(1500.0),
    /// )?;
    /// let wheel = |x: f32, z: f32| WheelSettings::new(Vec3::new(x, -0.1, z)).radius(0.35);
    /// let settings = WheeledVehicleSettings::new(
    ///     vec![wheel(0.9, 1.4), wheel(-0.9, 1.4), wheel(0.9, -1.4), wheel(-0.9, -1.4)],
    ///     vec![VehicleDifferentialSettings::new(Some(0), Some(1))],
    ///     VehicleCollisionTester::ray(ObjectLayer::MOVING),
    /// );
    /// let car = world.create_wheeled_vehicle(chassis, &settings)?;
    ///
    /// world.vehicle_mut(car)?.set_tire_max_impulse_callback(|input| {
    ///     let default = input.default_max_impulse();
    ///     TireMaxImpulse { lateral: 0.8 * default.lateral, ..default }
    /// })?;
    /// assert!(world.vehicle(car)?.has_tire_max_impulse_callback());
    /// assert!(world.step(1.0 / 60.0)?.is_complete());
    /// # Ok(())
    /// # }
    /// ```
    pub fn set_tire_max_impulse_callback(
        &mut self,
        callback: impl Fn(&TireMaxImpulseInput) -> TireMaxImpulse + Send + Sync + 'static,
    ) -> Result<(), VehicleError> {
        let ratio = self
            .entry
            .drivetrain
            .largest_wheel_mass_ratio(&self.entry.chassis);
        if !limits::is_wheel_mass_ratio(ratio) {
            return Err(VehicleError::InvalidValue(limits::WHEEL_MASS_RATIO_RULE));
        }
        let tire = Box::new(TireCallback {
            callback: Box::new(callback),
            panic: PanicSlot::default(),
        });
        // The replaced callback drops only after Jolt holds the new one.
        let replaced = self.entry.tire_callback.take();
        let tire: &TireCallback = self.entry.tire_callback.insert(tire);
        let user_data = std::ptr::from_ref(tire).cast_mut().cast::<c_void>();
        let controller = self.wheeled_controller();
        // SAFETY: the controller is this vehicle's (see `wheeled_controller`); the world is
        // borrowed mutably through this view, so no step runs. joltc copies the function and
        // `user_data` into the controller. `user_data` is the heap address of the entry's
        // `TireCallback`, which stays in place until the entry replaces or clears it (both
        // after installing another function) or drops it after its constraint is released.
        unsafe {
            JPH_WheeledVehicleController_SetTireMaxImpulseCallback(
                controller,
                Some(tire_max_impulse),
                user_data,
            )
        };
        drop(replaced);
        Ok(())
    }

    /// Removes the tire max impulse callback, so that the vehicle uses Jolt's default caps
    /// ([`TireMaxImpulseInput::default_max_impulse`]) from the next step on, and drops it. Does
    /// nothing without one.
    pub fn clear_tire_max_impulse_callback(&mut self) {
        if self.entry.tire_callback.is_none() {
            return;
        }
        let controller = self.wheeled_controller();
        // SAFETY: as in `set_tire_max_impulse_callback`; Jolt's default holds no pointer to the
        // callback dropped below.
        unsafe { JPH_WheeledVehicleController_ResetTireMaxImpulseCallback(controller) };
        self.entry.tire_callback = None;
    }
}
