//! Smoke tests for the vehicle constraint through the raw API: a box chassis with one driven
//! wheel, registered as a constraint (through the fork's `JPH_VehicleConstraint_AsConstraint`)
//! and as a step listener, steps on a floor and is torn down cleanly; the fork's vehicle step
//! listener, which applies a gravity override once per collision step; and the fork's checked
//! motorcycle target lean accessors.

mod framework;

use std::f32::consts::PI;
use std::ptr::null;

use framework::*;
use oxijolt_sys::*;

const DT: f32 = 1.0 / 60.0;

/// One wheel at the chassis' bottom centre, Jolt's other wheel defaults.
fn wheel_settings() -> *mut JPH_WheelSettingsWV {
    // SAFETY: Jolt is initialised (`TestWorld::new` ran). The settings are returned holding one
    // reference, which the caller releases; `position` is a live local.
    unsafe {
        let settings = JPH_WheelSettingsWV_Create();
        assert!(!settings.is_null());
        let position = vec3(0.0, -0.3, 0.0);
        JPH_WheelSettings_SetPosition(settings.cast(), &position);
        settings
    }
}

/// A wheeled controller with one differential that drives wheel 0 alone.
fn controller_settings() -> *mut JPH_WheeledVehicleControllerSettings {
    let differential = JPH_VehicleDifferentialSettings {
        leftWheel: 0,
        rightWheel: -1,
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

#[test]
fn vehicle_constraint_steps_and_tears_down() {
    let world = TestWorld::new(1);
    let floor = create_box(
        world.body_interface(),
        vec3(100.0, 1.0, 100.0),
        rvec3(0.0, -1.0, 0.0),
        JPH_MotionType_Static,
        OL_NON_MOVING,
        JPH_Activation_DontActivate,
    );
    let chassis = create_box(
        world.body_interface(),
        vec3(1.0, 0.3, 2.0),
        rvec3(0.0, 1.0, 0.0),
        JPH_MotionType_Dynamic,
        OL_MOVING,
        JPH_Activation_Activate,
    );

    // SAFETY: an all-zero `JPH_PhysicsSettings` is valid: integers, floats and `false`.
    let mut physics_settings: JPH_PhysicsSettings = unsafe { std::mem::zeroed() };
    // SAFETY: the system is live; `physics_settings` is a live local that joltc fills.
    unsafe { JPH_PhysicsSystem_GetPhysicsSettings(world.system(), &mut physics_settings) };
    // Jolt's defaults, which decide how many jobs run the step listeners.
    assert_eq!(physics_settings.stepListenersBatchSize, 8);
    assert_eq!(physics_settings.stepListenerBatchesPerJob, 1);

    let wheel = wheel_settings();
    let mut wheels = [wheel.cast::<JPH_WheelSettings>()];
    let controller = controller_settings();
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
        wheelsCount: 1,
        wheels: wheels.as_mut_ptr(),
        antiRollBarsCount: 0,
        antiRollBars: null(),
        controller: controller.cast(),
    };

    // SAFETY: the lock interface belongs to the live system; the multi-lock copies the one id.
    // The chassis exists, so the lock yields its body, which stays locked until the lock is
    // destroyed. The constructor stores the body pointer and reads the body; Jolt bodies do not
    // move in memory until destroyed. The settings, wheel and controller settings are live, and
    // the constraint keeps its own references to the wheel settings and copies the controller
    // settings. The new constraint holds one reference, ours.
    let constraint = unsafe {
        let lock_interface = JPH_PhysicsSystem_GetBodyLockInterface(world.system());
        let lock = JPH_BodyLockInterface_LockMultiWrite(lock_interface, &chassis, 1);
        let body = JPH_BodyLockMultiWrite_GetBody(lock, 0);
        assert!(!body.is_null());
        let constraint = JPH_VehicleConstraint_Create(body, &settings);
        JPH_BodyLockMultiWrite_Destroy(lock);
        JPH_WheelSettings_Destroy(wheel.cast());
        JPH_VehicleControllerSettings_Destroy(controller.cast());
        constraint
    };
    assert!(!constraint.is_null());

    // SAFETY: the constraint is live and holds our reference. `GetSubType` is a virtual call
    // through the returned pointer, so a wrong base address would not return the vehicle
    // subtype.
    let as_constraint = unsafe { JPH_VehicleConstraint_AsConstraint(constraint) };
    // SAFETY: as above.
    let sub_type = unsafe { JPH_Constraint_GetSubType(as_constraint) };
    assert_eq!(sub_type, JPH_ConstraintSubType_Vehicle);

    let up = vec3(0.0, 1.0, 0.0);
    // SAFETY: the system and constraint are live. The tester is created holding one reference;
    // the constraint takes its own, so ours is released right after. The constraint is added
    // both as a constraint and as a step listener, with the tester set before the first step,
    // which is how Jolt expects a vehicle to be registered.
    unsafe {
        let tester = JPH_VehicleCollisionTesterRay_Create(OL_MOVING, &up, 80.0_f32.to_radians());
        assert!(!tester.is_null());
        JPH_VehicleConstraint_SetVehicleCollisionTester(constraint, tester.cast());
        JPH_VehicleCollisionTester_Destroy(tester.cast());
        JPH_PhysicsSystem_AddConstraint(world.system(), as_constraint);
        JPH_PhysicsSystem_AddStepListener(
            world.system(),
            JPH_VehicleConstraint_AsPhysicsStepListener(constraint),
        );
    }

    for _ in 0..10 {
        world.step(DT);
    }

    // SAFETY: the constraint is live and has one wheel; the wheel is owned by the constraint
    // and read only. No step runs meanwhile.
    let has_contact = unsafe {
        assert_eq!(JPH_VehicleConstraint_GetWheelsCount(constraint), 1);
        let wheel = JPH_VehicleConstraint_GetWheel(constraint, 0);
        JPH_Wheel_HasContact(wheel)
    };
    // The chassis bottom is 0.7 above the floor and the wheel reaches 0.5 + 0.3 below it.
    assert!(has_contact);

    // SAFETY: the vehicle is removed from the listeners and the constraints before our
    // reference, the last one, is released, and before its body is destroyed. The body ids
    // name live bodies.
    unsafe {
        JPH_PhysicsSystem_RemoveStepListener(
            world.system(),
            JPH_VehicleConstraint_AsPhysicsStepListener(constraint),
        );
        JPH_PhysicsSystem_RemoveConstraint(world.system(), as_constraint);
        JPH_Constraint_Destroy(as_constraint);
        JPH_BodyInterface_RemoveAndDestroyBody(world.body_interface(), chassis);
        JPH_BodyInterface_RemoveAndDestroyBody(world.body_interface(), floor);
    }
}

/// Creates a vehicle constraint on `chassis` with `wheels` and `controller`, then releases our
/// references to the wheel and controller settings. The constraint is not registered with the
/// system; the caller releases its one reference.
fn create_constraint(
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
    // SAFETY: as in `vehicle_constraint_steps_and_tears_down`: the chassis is locked for the
    // constructor, which keeps its own references to the wheel settings and builds its
    // controller from the controller settings, so ours are released afterwards.
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
fn create_chassis(world: &TestWorld, x: Real) -> JPH_BodyID {
    create_box(
        world.body_interface(),
        vec3(0.2, 0.3, 0.4),
        rvec3(x, 1.0, 0.0),
        JPH_MotionType_Dynamic,
        OL_MOVING,
        JPH_Activation_Activate,
    )
}

/// A raw wheeled vehicle (one wheel) and a raw tracked vehicle (one wheel per track), never
/// registered with the system, each with its chassis.
fn other_vehicle_kinds(world: &TestWorld) -> [(*mut JPH_VehicleConstraint, JPH_BodyID); 2] {
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

/// Which step listener runs a vehicle.
#[derive(Clone, Copy)]
enum VehicleListener {
    /// The constraint's own `PhysicsStepListener` base.
    Jolt,
    /// The fork's `JPH_VehicleStepListener`.
    Fork,
}

/// Lets a one-wheel vehicle with its gravity override at the world's gravity fall freely next to a
/// plain box of the same shape for one update of `collision_steps` collision steps, run by
/// `listener`; returns the vertical velocities of the chassis and the box.
fn fall_with_gravity_override(listener: VehicleListener, collision_steps: i32) -> [f32; 2] {
    let world = TestWorld::new(1);
    let size = vec3(0.2, 0.3, 0.4);
    let chassis = create_box(
        world.body_interface(),
        size,
        rvec3(0.0, 10.0, 0.0),
        JPH_MotionType_Dynamic,
        OL_MOVING,
        JPH_Activation_Activate,
    );
    let free_box = create_box(
        world.body_interface(),
        size,
        rvec3(10.0, 10.0, 0.0),
        JPH_MotionType_Dynamic,
        OL_MOVING,
        JPH_Activation_Activate,
    );
    let constraint = create_constraint(
        &world,
        chassis,
        &mut [wheel_settings().cast()],
        controller_settings().cast(),
    );
    let mut gravity = vec3(0.0, 0.0, 0.0);
    let up = vec3(0.0, 1.0, 0.0);
    // SAFETY: the system and the constraint are live and no step runs. The tester's reference is
    // ours until the constraint takes its own. The fork's listener is created for the live
    // constraint, which outlives its registration; either listener is registered with the tester
    // set and removed below before the listener and the constraint are released.
    let fork_listener = unsafe {
        JPH_PhysicsSystem_GetGravity(world.system(), &mut gravity);
        JPH_VehicleConstraint_OverrideGravity(constraint, &gravity);
        let tester = JPH_VehicleCollisionTesterRay_Create(OL_MOVING, &up, 80.0_f32.to_radians());
        assert!(!tester.is_null());
        JPH_VehicleConstraint_SetVehicleCollisionTester(constraint, tester.cast());
        JPH_VehicleCollisionTester_Destroy(tester.cast());
        JPH_PhysicsSystem_AddConstraint(
            world.system(),
            JPH_VehicleConstraint_AsConstraint(constraint),
        );
        let fork_listener = JPH_VehicleStepListener_Create(constraint);
        assert!(!fork_listener.is_null());
        fork_listener
    };
    // SAFETY: both pointers are live; the cast picks the step listener base of each.
    let step_listener = unsafe {
        match listener {
            VehicleListener::Jolt => JPH_VehicleConstraint_AsPhysicsStepListener(constraint),
            VehicleListener::Fork => JPH_VehicleStepListener_AsPhysicsStepListener(fork_listener),
        }
    };
    // SAFETY: the listener is live and stays registered until it is removed below.
    unsafe { JPH_PhysicsSystem_AddStepListener(world.system(), step_listener) };
    assert_eq!(gravity.y, -9.81);

    world.step_with_collision_steps(DT, collision_steps);

    let mut chassis_velocity = vec3(0.0, 0.0, 0.0);
    let mut box_velocity = vec3(0.0, 0.0, 0.0);
    // SAFETY: the ids name live bodies and the outputs are live locals. The listener is removed
    // before it is destroyed, the vehicle from the constraints before our reference, the last
    // one, is released, and both before the chassis goes.
    unsafe {
        JPH_BodyInterface_GetLinearVelocity(world.body_interface(), chassis, &mut chassis_velocity);
        JPH_BodyInterface_GetLinearVelocity(world.body_interface(), free_box, &mut box_velocity);
        JPH_PhysicsSystem_RemoveStepListener(world.system(), step_listener);
        JPH_VehicleStepListener_Destroy(fork_listener);
        JPH_PhysicsSystem_RemoveConstraint(
            world.system(),
            JPH_VehicleConstraint_AsConstraint(constraint),
        );
        JPH_Constraint_Destroy(JPH_VehicleConstraint_AsConstraint(constraint));
        JPH_BodyInterface_RemoveAndDestroyBody(world.body_interface(), chassis);
        JPH_BodyInterface_RemoveAndDestroyBody(world.body_interface(), free_box);
    }
    [chassis_velocity.y, box_velocity.y]
}

#[test]
fn vehicle_step_listener_applies_the_gravity_override_once_per_collision_step() {
    for collision_steps in 1..=4 {
        let [chassis, free_box] =
            fall_with_gravity_override(VehicleListener::Fork, collision_steps);
        assert!(free_box < -0.1, "{free_box}");
        assert!(
            (chassis - free_box).abs() <= 1e-6 * free_box.abs(),
            "{collision_steps} collision steps: chassis {chassis}, box {free_box}"
        );
    }
    // Jolt's own listener adds the override again in every collision step: collision step k of
    // four gets k times it, (1 + 2 + 3 + 4) / 4 = 2.5 times the box's velocity in all.
    let [chassis, free_box] = fall_with_gravity_override(VehicleListener::Jolt, 4);
    assert!(
        (chassis / free_box - 2.5).abs() < 1e-3,
        "chassis {chassis}, box {free_box}"
    );
}

/// Reads a controller's target lean through the extension, starting from a sentinel.
fn target_lean(controller: *mut JPH_VehicleController) -> (bool, [f32; 3]) {
    let mut lean = vec3(7.0, 8.0, 9.0);
    // SAFETY: the caller passes a live controller (its constraint holds it); `lean` is a live
    // local.
    let found = unsafe { JPH_MotorcycleController_GetTargetLean(controller, &mut lean) };
    (found, [lean.x, lean.y, lean.z])
}

#[test]
fn motorcycle_target_lean_is_checked_read_and_written() {
    let world = TestWorld::new(1);
    let floor = create_box(
        world.body_interface(),
        vec3(100.0, 1.0, 100.0),
        rvec3(0.0, -1.0, 0.0),
        JPH_MotionType_Static,
        OL_NON_MOVING,
        JPH_Activation_DontActivate,
    );
    let chassis = create_chassis(&world, 0.0);
    let wheel = |z: f32| {
        let position = vec3(0.0, -0.3, z);
        // SAFETY: Jolt is initialised. The settings are returned holding one reference, which
        // `create_constraint` releases; `position` is a live local that joltc copies.
        unsafe {
            let settings = JPH_WheelSettingsWV_Create();
            assert!(!settings.is_null());
            JPH_WheelSettings_SetPosition(settings.cast(), &position);
            settings.cast::<JPH_WheelSettings>()
        }
    };
    let mut wheels = [wheel(0.75), wheel(-0.75)];
    // The rear wheel alone is driven.
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
    let motorcycle = create_constraint(&world, chassis, &mut wheels, controller.cast());
    // SAFETY: the constraint is live and owns its controller.
    let motorcycle_controller = unsafe { JPH_VehicleConstraint_GetController(motorcycle) };

    // Jolt starts from the zero vector.
    assert_eq!(target_lean(motorcycle_controller), (true, [0.0; 3]));

    let up = vec3(0.0, 1.0, 0.0);
    // SAFETY: as in `vehicle_constraint_steps_and_tears_down`: the tester's reference is ours
    // until the constraint takes its own; the vehicle is registered as a constraint and a step
    // listener with its tester set.
    unsafe {
        let tester = JPH_VehicleCollisionTesterRay_Create(OL_MOVING, &up, 80.0_f32.to_radians());
        assert!(!tester.is_null());
        JPH_VehicleConstraint_SetVehicleCollisionTester(motorcycle, tester.cast());
        JPH_VehicleCollisionTester_Destroy(tester.cast());
        JPH_PhysicsSystem_AddConstraint(
            world.system(),
            JPH_VehicleConstraint_AsConstraint(motorcycle),
        );
        JPH_PhysicsSystem_AddStepListener(
            world.system(),
            JPH_VehicleConstraint_AsPhysicsStepListener(motorcycle),
        );
    }
    for _ in 0..10 {
        world.step(DT);
    }
    let (found, lean) = target_lean(motorcycle_controller);
    assert!(found);
    assert!(lean.iter().all(|value| value.is_finite()), "{lean:?}");
    let distance_to_up = (lean[0].powi(2) + (lean[1] - 1.0).powi(2) + lean[2].powi(2)).sqrt();
    assert!(distance_to_up < 0.1, "{lean:?}");

    let written = vec3(0.6, 0.8, 0.0);
    // SAFETY: the controller is live, no step runs, and `written` is a live local.
    let stored = unsafe { JPH_MotorcycleController_SetTargetLean(motorcycle_controller, &written) };
    assert!(stored);
    let (found, lean) = target_lean(motorcycle_controller);
    assert!(found);
    assert_eq!(
        lean.map(f32::to_bits),
        [0.6_f32, 0.8, 0.0].map(f32::to_bits)
    );

    let others = other_vehicle_kinds(&world);
    for &(constraint, _) in &others {
        // SAFETY: the constraint is live and owns its controller.
        let controller = unsafe { JPH_VehicleConstraint_GetController(constraint) };
        assert_eq!(target_lean(controller), (false, [7.0, 8.0, 9.0]));
        let other = vec3(1.0, 0.0, 0.0);
        // SAFETY: as above; `other` is a live local.
        assert!(!unsafe { JPH_MotorcycleController_SetTargetLean(controller, &other) });
    }

    // SAFETY: the motorcycle is removed from the listeners and constraints before our reference,
    // the last one, is released; the other two were never registered. Each constraint goes
    // before its body; the ids name live bodies.
    unsafe {
        JPH_PhysicsSystem_RemoveStepListener(
            world.system(),
            JPH_VehicleConstraint_AsPhysicsStepListener(motorcycle),
        );
        JPH_PhysicsSystem_RemoveConstraint(
            world.system(),
            JPH_VehicleConstraint_AsConstraint(motorcycle),
        );
        JPH_Constraint_Destroy(JPH_VehicleConstraint_AsConstraint(motorcycle));
        for (constraint, body) in others {
            JPH_Constraint_Destroy(JPH_VehicleConstraint_AsConstraint(constraint));
            JPH_BodyInterface_RemoveAndDestroyBody(world.body_interface(), body);
        }
        JPH_BodyInterface_RemoveAndDestroyBody(world.body_interface(), chassis);
        JPH_BodyInterface_RemoveAndDestroyBody(world.body_interface(), floor);
    }
}
