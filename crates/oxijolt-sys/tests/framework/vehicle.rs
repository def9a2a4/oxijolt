//! Raw vehicle setup shared by the vehicle tests: wheel and controller settings, constraint
//! creation on a locked chassis, and one unregistered vehicle of each other kind.

use std::f32::consts::PI;
use std::ptr::null;

use super::*;

/// One wheel at the chassis' bottom centre, Jolt's other wheel defaults.
pub fn wheel_settings() -> *mut JPH_WheelSettingsWV {
    wheel_settings_at(vec3(0.0, -0.3, 0.0))
}

/// One wheel at `position` in the chassis' frame, Jolt's other wheel defaults.
pub fn wheel_settings_at(position: JPH_Vec3) -> *mut JPH_WheelSettingsWV {
    // SAFETY: Jolt is initialised (`TestWorld::new` ran). The settings are returned holding one
    // reference, which the caller releases; `position` is a live local that joltc copies.
    unsafe {
        let settings = JPH_WheelSettingsWV_Create();
        assert!(!settings.is_null());
        JPH_WheelSettings_SetPosition(settings.cast(), &position);
        settings
    }
}

/// A wheeled controller with one differential that drives wheel 0 alone.
pub fn controller_settings() -> *mut JPH_WheeledVehicleControllerSettings {
    wheeled_controller_settings(0, -1)
}

/// A wheeled controller with one differential between `left_wheel` and `right_wheel` (-1 for
/// none).
pub fn wheeled_controller_settings(
    left_wheel: i32,
    right_wheel: i32,
) -> *mut JPH_WheeledVehicleControllerSettings {
    let differential = JPH_VehicleDifferentialSettings {
        leftWheel: left_wheel,
        rightWheel: right_wheel,
        differentialRatio: 3.42,
        leftRightSplit: 0.5,
        limitedSlipRatio: 1.4,
        engineTorqueRatio: 1.0,
    };
    // SAFETY: Jolt is initialised. The settings are returned holding one reference, which the
    // caller releases; `differential` is a live local that joltc copies.
    unsafe {
        let settings = JPH_WheeledVehicleControllerSettings_Create();
        assert!(!settings.is_null());
        JPH_WheeledVehicleControllerSettings_SetDifferentials(settings, &differential, 1);
        settings
    }
}

/// Creates a vehicle constraint on `chassis` with `wheels` and `controller`, then releases our
/// references to the wheel and controller settings. The constraint is not registered with the
/// system; the caller releases its one reference.
pub fn create_constraint(
    world: &TestWorld,
    chassis: JPH_BodyID,
    wheels: &mut [*mut JPH_WheelSettings],
    controller: *mut JPH_VehicleControllerSettings,
) -> *mut JPH_VehicleConstraint {
    let settings = JPH_VehicleConstraintSettings {
        base: JPH_ConstraintSettings {
            enabled: true,
            constraintPriority: 0,
            numVelocityStepsOverride: 0,
            numPositionStepsOverride: 0,
            drawConstraintSize: 1.0,
            userData: 0,
        },
        up: vec3(0.0, 1.0, 0.0),
        forward: vec3(0.0, 0.0, 1.0),
        maxPitchRollAngle: PI,
        wheelsCount: wheels.len() as u32,
        wheels: wheels.as_mut_ptr(),
        antiRollBarsCount: 0,
        antiRollBars: null(),
        controller,
    };
    // SAFETY: the lock interface belongs to the live system; the multi-lock copies the one id.
    // The chassis exists, so the lock yields its body, which stays locked until the lock is
    // destroyed. The constructor stores the body pointer and reads the body; Jolt bodies do not
    // move in memory until destroyed. The constructor keeps its own references to the wheel
    // settings and builds its controller from the controller settings, so ours are released
    // afterwards.
    let constraint = unsafe {
        let lock_interface = JPH_PhysicsSystem_GetBodyLockInterface(world.system());
        let lock = JPH_BodyLockInterface_LockMultiWrite(lock_interface, &chassis, 1);
        let body = JPH_BodyLockMultiWrite_GetBody(lock, 0);
        assert!(!body.is_null());
        let constraint = JPH_VehicleConstraint_Create(body, &settings);
        JPH_BodyLockMultiWrite_Destroy(lock);
        for &wheel in wheels.iter() {
            JPH_WheelSettings_Destroy(wheel);
        }
        JPH_VehicleControllerSettings_Destroy(controller);
        constraint
    };
    assert!(!constraint.is_null());
    constraint
}

/// A small chassis box 1 m above the floor at `x`.
pub fn create_chassis(world: &TestWorld, x: Real) -> JPH_BodyID {
    create_box(
        world.body_interface(),
        vec3(0.2, 0.3, 0.4),
        rvec3(x, 1.0, 0.0),
        JPH_MotionType_Dynamic,
        OL_MOVING,
        JPH_Activation_Activate,
    )
}

/// A raw motorcycle on `chassis`, front and rear wheel 0.75 m from its centre, the rear wheel
/// (index 1) driven, never registered with the system.
pub fn create_motorcycle(world: &TestWorld, chassis: JPH_BodyID) -> *mut JPH_VehicleConstraint {
    let mut wheels = [
        wheel_settings_at(vec3(0.0, -0.3, 0.75)).cast(),
        wheel_settings_at(vec3(0.0, -0.3, -0.75)).cast(),
    ];
    let differential = JPH_VehicleDifferentialSettings {
        leftWheel: -1,
        rightWheel: 1,
        differentialRatio: 4.825,
        leftRightSplit: 0.5,
        limitedSlipRatio: 1.4,
        engineTorqueRatio: 1.0,
    };
    // SAFETY: Jolt is initialised. The settings are returned holding one reference, which
    // `create_constraint` releases. Motorcycle controller settings are wheeled controller
    // settings with single inheritance, joltc's cast convention; joltc copies `differential`.
    let controller = unsafe {
        let controller = JPH_MotorcycleControllerSettings_Create();
        assert!(!controller.is_null());
        JPH_WheeledVehicleControllerSettings_SetDifferentials(controller.cast(), &differential, 1);
        controller
    };
    create_constraint(world, chassis, &mut wheels, controller.cast())
}

/// A raw wheeled vehicle (one wheel) and a raw tracked vehicle (one wheel per track: wheel 0 on
/// the left track, wheel 1 on the right, each driven), never registered with the system, each
/// with its chassis.
pub fn other_vehicle_kinds(world: &TestWorld) -> [(*mut JPH_VehicleConstraint, JPH_BodyID); 2] {
    let wheeled_chassis = create_chassis(world, 5.0);
    let wheeled = create_constraint(
        world,
        wheeled_chassis,
        &mut [wheel_settings().cast()],
        controller_settings().cast(),
    );

    let tracked_chassis = create_chassis(world, -5.0);
    let left_wheels = [0_u32];
    let right_wheels = [1_u32];
    // SAFETY: Jolt is initialised. Each creator returns its object holding one reference, which
    // `create_constraint` releases. The track settings and their wheel lists are live locals
    // that `SetTrack` copies.
    let (mut tracked_wheels, tracked_controller) = unsafe {
        let wheels: [*mut JPH_WheelSettings; 2] = [
            JPH_WheelSettingsTV_Create().cast(),
            JPH_WheelSettingsTV_Create().cast(),
        ];
        let controller = JPH_TrackedVehicleControllerSettings_Create();
        assert!(!controller.is_null());
        let mut left: JPH_VehicleTrackSettings = std::mem::zeroed();
        JPH_VehicleTrackSettings_Init(&mut left);
        let mut right = left;
        left.drivenWheel = 0;
        left.wheels = left_wheels.as_ptr();
        left.wheelsCount = 1;
        right.drivenWheel = 1;
        right.wheels = right_wheels.as_ptr();
        right.wheelsCount = 1;
        JPH_TrackedVehicleControllerSettings_SetTrack(controller, 0, &left);
        JPH_TrackedVehicleControllerSettings_SetTrack(controller, 1, &right);
        (wheels, controller)
    };
    let tracked = create_constraint(
        world,
        tracked_chassis,
        &mut tracked_wheels,
        tracked_controller.cast(),
    );
    [(wheeled, wheeled_chassis), (tracked, tracked_chassis)]
}

/// Gives `constraint` a ray collision tester and registers it with the system as a constraint and
/// as a step listener, as Jolt expects a vehicle to be registered. [`unregister`] undoes it.
pub fn register(world: &TestWorld, constraint: *mut JPH_VehicleConstraint) {
    let up = vec3(0.0, 1.0, 0.0);
    // SAFETY: the system and constraint are live and no step runs. The tester is created holding
    // one reference; the constraint takes its own, so ours is released right after. The tester is
    // set before the first step.
    unsafe {
        let tester = JPH_VehicleCollisionTesterRay_Create(OL_MOVING, &up, 80.0_f32.to_radians());
        assert!(!tester.is_null());
        JPH_VehicleConstraint_SetVehicleCollisionTester(constraint, tester.cast());
        JPH_VehicleCollisionTester_Destroy(tester.cast());
        JPH_PhysicsSystem_AddConstraint(
            world.system(),
            JPH_VehicleConstraint_AsConstraint(constraint),
        );
        JPH_PhysicsSystem_AddStepListener(
            world.system(),
            JPH_VehicleConstraint_AsPhysicsStepListener(constraint),
        );
    }
}

/// Removes a vehicle that [`register`] registered from the step listeners and the constraints.
pub fn unregister(world: &TestWorld, constraint: *mut JPH_VehicleConstraint) {
    // SAFETY: the constraint is live and registered by `register`; no step runs.
    unsafe {
        JPH_PhysicsSystem_RemoveStepListener(
            world.system(),
            JPH_VehicleConstraint_AsPhysicsStepListener(constraint),
        );
        JPH_PhysicsSystem_RemoveConstraint(
            world.system(),
            JPH_VehicleConstraint_AsConstraint(constraint),
        );
    }
}
