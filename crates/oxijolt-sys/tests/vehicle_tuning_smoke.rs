//! Smoke tests for the fork's live drivetrain setters through the raw API: engine, transmission
//! and differentials of wheeled controllers (motorcycles through the wheeled cast), engine,
//! transmission and tracks of tracked controllers, and the reset of the tire max impulse callback.

mod framework;

use std::ffi::c_void;
use std::sync::atomic::{AtomicU32, Ordering};

use framework::vehicle::*;
use framework::*;
use oxijolt_sys::*;

const DT: f32 = 1.0 / 60.0;

/// The controller of a live constraint, as the handle type `T` of its kind.
fn controller_of<T>(constraint: *mut JPH_VehicleConstraint) -> *mut T {
    // SAFETY: the caller passes a live constraint, which owns its controller.
    unsafe { JPH_VehicleConstraint_GetController(constraint) }.cast()
}

/// Destroys unregistered constraints and then their chassis.
fn destroy_vehicles(world: &TestWorld, vehicles: &[(*mut JPH_VehicleConstraint, JPH_BodyID)]) {
    for &(constraint, body) in vehicles {
        // SAFETY: the constraint is live, not registered, and our reference is its last one; it
        // goes before its chassis, whose id names a live body.
        unsafe {
            JPH_Constraint_Destroy(JPH_VehicleConstraint_AsConstraint(constraint));
            JPH_BodyInterface_RemoveAndDestroyBody(world.body_interface(), body);
        }
    }
}

/// A static floor whose top is at y = 0.
fn create_floor(world: &TestWorld) -> JPH_BodyID {
    create_box(
        world.body_interface(),
        vec3(100.0, 1.0, 100.0),
        rvec3(0.0, -1.0, 0.0),
        JPH_MotionType_Static,
        OL_NON_MOVING,
        JPH_Activation_DontActivate,
    )
}

/// A raw car at height `y`: a 2 x 0.6 x 4 m chassis with four wheels, front pair 0 and 1, rear
/// pair 2 and 3, the front pair driven. Not registered with the system.
fn create_car(world: &TestWorld, y: Real) -> (*mut JPH_VehicleConstraint, JPH_BodyID) {
    let chassis = create_box(
        world.body_interface(),
        vec3(1.0, 0.3, 2.0),
        rvec3(0.0, y, 0.0),
        JPH_MotionType_Dynamic,
        OL_MOVING,
        JPH_Activation_Activate,
    );
    let mut wheels = [(0.9, 1.5), (-0.9, 1.5), (0.9, -1.5), (-0.9, -1.5)]
        .map(|(x, z)| wheel_settings_at(vec3(x, -0.3, z)).cast::<JPH_WheelSettings>());
    let constraint = create_constraint(
        world,
        chassis,
        &mut wheels,
        wheeled_controller_settings(0, 1).cast(),
    );
    (constraint, chassis)
}

/// The absolute angular velocities of the car's four wheels.
fn wheel_speeds(constraint: *mut JPH_VehicleConstraint) -> [f32; 4] {
    // SAFETY: the constraint is live and has four wheels, which it owns; no step runs.
    std::array::from_fn(|index| unsafe {
        JPH_Wheel_GetAngularVelocity(JPH_VehicleConstraint_GetWheel(constraint, index as u32)).abs()
    })
}

#[test]
fn live_engine_reaches_wheeled_and_tracked_controllers() {
    let world = TestWorld::new(1);
    let [car, tank] = other_vehicle_kinds(&world);
    let motorcycle_chassis = create_chassis(&world, 0.0);
    let motorcycle = (
        create_motorcycle(&world, motorcycle_chassis),
        motorcycle_chassis,
    );

    // SAFETY: Jolt is initialised; the curve is ours until destroyed below.
    let curve = unsafe {
        let curve = JPH_LinearCurve_Create();
        assert!(!curve.is_null());
        JPH_LinearCurve_AddPoint(curve, 0.0, 1.0);
        JPH_LinearCurve_AddPoint(curve, 1.0, 1.0);
        curve
    };
    let settings = JPH_VehicleEngineSettings {
        maxTorque: 1234.0,
        minRPM: 800.0,
        maxRPM: 7000.0,
        normalizedTorque: curve,
        inertia: 0.7,
        angularDamping: 0.3,
    };

    let wheeled = [car, motorcycle].map(|(constraint, _)| {
        let controller = controller_of::<JPH_WheeledVehicleController>(constraint);
        // SAFETY: the controller is live and no step runs; a motorcycle controller is a wheeled
        // controller by single inheritance. The engine is owned by the controller.
        unsafe { JPH_WheeledVehicleController_GetEngine(controller) }
    });
    let tracked = controller_of::<JPH_TrackedVehicleController>(tank.0);
    // SAFETY: as above, for the tracked controller.
    let tracked_engine = unsafe { JPH_TrackedVehicleController_GetEngine(tracked) };
    let engines = [wheeled[0], wheeled[1], tracked_engine];
    // SAFETY: the engines are owned by live controllers.
    let rpm_before = engines.map(|engine| unsafe { JPH_VehicleEngine_GetCurrentRPM(engine) });

    // SAFETY: the controllers are live and no step runs; `settings` and its curve are live.
    unsafe {
        for (constraint, _) in [car, motorcycle] {
            JPH_WheeledVehicleController_SetEngine(controller_of(constraint), &settings);
        }
        JPH_TrackedVehicleController_SetEngine(tracked, &settings);
        // The controllers hold deep copies of the curve.
        JPH_LinearCurve_Destroy(curve);
    }

    for (engine, before) in engines.into_iter().zip(rpm_before) {
        // SAFETY: the engine is owned by a live controller; no step runs.
        let (torque, rpm) = unsafe {
            (
                JPH_VehicleEngine_GetTorque(engine, 1.0),
                JPH_VehicleEngine_GetCurrentRPM(engine),
            )
        };
        assert_eq!(torque, 1234.0);
        assert_eq!(rpm.to_bits(), before.to_bits());
    }

    destroy_vehicles(&world, &[car, tank, motorcycle]);
}

/// Transmission settings with these gear ratios and clutch release time, Jolt's defaults
/// otherwise. The caller destroys them.
fn transmission_settings(
    forward: &[f32],
    reverse: &[f32],
    clutch_release_time: f32,
) -> *mut JPH_VehicleTransmissionSettings {
    // SAFETY: Jolt is initialised; the slices are live and joltc copies them.
    unsafe {
        let settings = JPH_VehicleTransmissionSettings_Create();
        assert!(!settings.is_null());
        JPH_VehicleTransmissionSettings_SetGearRatios(
            settings,
            forward.as_ptr(),
            forward.len() as u32,
        );
        JPH_VehicleTransmissionSettings_SetReverseGearRatios(
            settings,
            reverse.as_ptr(),
            reverse.len() as u32,
        );
        JPH_VehicleTransmissionSettings_SetClutchReleaseTime(settings, clutch_release_time);
        settings
    }
}

/// Puts `transmission` in third gear at half clutch friction, then checks that `set` accepts new
/// ratios with the controller's gear counts (`forward`, `reverse`) and clutch release time 0.3 s,
/// keeping gear and friction, and refuses other gear counts or another release time.
fn check_live_transmission(
    transmission: *mut JPH_VehicleTransmission,
    forward: &[f32],
    reverse: &[f32],
    set: impl Fn(*const JPH_VehicleTransmissionSettings) -> bool,
) {
    let apply = |forward: &[f32], reverse: &[f32], clutch_release_time: f32| {
        let settings = transmission_settings(forward, reverse, clutch_release_time);
        let applied = set(settings);
        // SAFETY: the settings are ours; the controller copied what it took.
        unsafe { JPH_VehicleTransmissionSettings_Destroy(settings) };
        applied
    };
    // SAFETY: the transmission is owned by a live controller and no step runs.
    let state = || unsafe {
        (
            JPH_VehicleTransmission_GetCurrentGear(transmission),
            JPH_VehicleTransmission_GetClutchFriction(transmission),
            JPH_VehicleTransmission_GetCurrentRatio(transmission),
        )
    };
    // SAFETY: as above.
    unsafe { JPH_VehicleTransmission_Set(transmission, 3, 0.5) };

    assert!(apply(forward, reverse, 0.3));
    assert_eq!(state(), (3, 0.5, forward[2]));

    // Every refused change carries other ratios, so a partial copy would show in the ratio.
    let doubled = |ratios: &[f32]| ratios.iter().map(|ratio| 2.0 * ratio).collect::<Vec<f32>>();
    let (other_forward, other_reverse) = (doubled(forward), doubled(reverse));
    assert!(
        !apply(&other_forward[1..], &other_reverse, 0.3),
        "fewer forward gears"
    );
    let mut more_reverse = other_reverse.clone();
    more_reverse.push(-1.0);
    assert!(
        !apply(&other_forward, &more_reverse, 0.3),
        "more reverse gears"
    );
    assert!(
        !apply(&other_forward, &other_reverse[1..], 0.3),
        "fewer reverse gears"
    );
    assert!(
        !apply(&other_forward, &other_reverse, 0.4),
        "another clutch release time"
    );
    assert_eq!(state(), (3, 0.5, forward[2]));
}

#[test]
fn live_transmission_keeps_gear_and_refuses_fixed_changes() {
    let world = TestWorld::new(1);
    let [car, tank] = other_vehicle_kinds(&world);

    let wheeled = controller_of::<JPH_WheeledVehicleController>(car.0);
    // SAFETY: the controller is live; the transmission it owns is not const in Jolt, joltc's
    // getter only hands it out as const.
    let transmission = unsafe { JPH_WheeledVehicleController_GetTransmission(wheeled) }.cast_mut();
    // Jolt's wheeled default: five forward gears, one reverse.
    check_live_transmission(
        transmission,
        &[3.0, 2.5, 2.0, 1.5, 1.0],
        &[-3.0],
        |settings| {
            // SAFETY: the controller is live, no step runs, and the settings are live.
            unsafe { JPH_WheeledVehicleController_SetTransmission(wheeled, settings) }
        },
    );

    let tracked = controller_of::<JPH_TrackedVehicleController>(tank.0);
    // SAFETY: as for the wheeled controller.
    let transmission = unsafe { JPH_TrackedVehicleController_GetTransmission(tracked) }.cast_mut();
    // Jolt's tracked default: four forward gears, two reverse.
    check_live_transmission(
        transmission,
        &[4.5, 3.5, 2.5, 1.5],
        &[-4.5, -3.5],
        |settings| {
            // SAFETY: the controller is live, no step runs, and the settings are live.
            unsafe { JPH_TrackedVehicleController_SetTransmission(tracked, settings) }
        },
    );

    destroy_vehicles(&world, &[car, tank]);
}

#[test]
fn live_differentials_change_which_wheels_are_driven() {
    let world = TestWorld::new(1);
    let zero = vec3(0.0, 0.0, 0.0);
    // SAFETY: the system is live and no step runs; `zero` is a live local.
    unsafe { JPH_PhysicsSystem_SetGravity(world.system(), &zero) };
    // Airborne: no floor, no gravity, so the wheels spin freely.
    let (car, chassis) = create_car(&world, 10.0);
    register(&world, car);
    let controller = controller_of::<JPH_WheeledVehicleController>(car);
    // SAFETY: the controller is live and no step runs.
    unsafe { JPH_WheeledVehicleController_SetDriverInput(controller, 1.0, 0.0, 0.0, 0.0) };

    for _ in 0..90 {
        world.step(DT);
    }
    let front_driven = wheel_speeds(car);
    assert!(
        front_driven[0] > 1.0 && front_driven[1] > 1.0,
        "{front_driven:?}"
    );
    assert_eq!(&front_driven[2..], &[0.0, 0.0]);

    let rear = JPH_VehicleDifferentialSettings {
        leftWheel: 2,
        rightWheel: 3,
        differentialRatio: 3.42,
        leftRightSplit: 0.5,
        limitedSlipRatio: 1.4,
        engineTorqueRatio: 1.0,
    };
    // SAFETY: the controller is live, no step runs, and `rear` is a live local; wheels 2 and 3
    // exist.
    unsafe {
        JPH_WheeledVehicleController_SetDifferentials(controller, &rear, 1);
        JPH_WheeledVehicleController_SetDifferentialLimitedSlipRatio(controller, 2.0);
    }
    for _ in 0..90 {
        world.step(DT);
    }
    let rear_driven = wheel_speeds(car);
    assert!(
        rear_driven.iter().all(|speed| speed.is_finite()),
        "{rear_driven:?}"
    );
    assert!(
        rear_driven[2] > 1.0 && rear_driven[3] > 1.0,
        "{rear_driven:?}"
    );
    assert!(
        rear_driven[0] <= front_driven[0] && rear_driven[1] <= front_driven[1],
        "{front_driven:?} -> {rear_driven:?}"
    );

    unregister(&world, car);
    destroy_vehicles(&world, &[(car, chassis)]);
}

/// The track on `side` of a live tracked controller: inertia, angular damping, max brake torque
/// and differential ratio.
fn track_values(controller: *mut JPH_TrackedVehicleController, side: JPH_TrackSide) -> [f32; 4] {
    // SAFETY: the controller is live and owns its tracks; no step runs.
    unsafe {
        let track = JPH_TrackedVehicleController_GetTrack(controller, side);
        [
            JPH_VehicleTrack_GetInertia(track),
            JPH_VehicleTrack_GetAngularDamping(track),
            JPH_VehicleTrack_GetMaxBrakeTorque(track),
            JPH_VehicleTrack_GetDifferentialRatio(track),
        ]
    }
}

#[test]
fn live_track_settings_reach_the_track_and_refuse_other_wheels() {
    let world = TestWorld::new(1);
    let [car, tank] = other_vehicle_kinds(&world);
    let controller = controller_of::<JPH_TrackedVehicleController>(tank.0);
    let right_before = track_values(controller, JPH_TrackSide_Right);

    let set = |side: JPH_TrackSide, driven_wheel: u32, wheels: &[u32], inertia: f32| {
        // SAFETY: an all-zero track settings value is valid (integers, floats, a null list),
        // and `_Init` fills it.
        let mut track: JPH_VehicleTrackSettings = unsafe { std::mem::zeroed() };
        // SAFETY: `track` is a live local.
        unsafe { JPH_VehicleTrackSettings_Init(&mut track) };
        track.drivenWheel = driven_wheel;
        track.wheels = wheels.as_ptr();
        track.wheelsCount = wheels.len() as u32;
        track.inertia = inertia;
        track.angularDamping = 0.7;
        track.maxBrakeTorque = 9000.0;
        track.differentialRatio = 5.0;
        // SAFETY: the controller is live, no step runs, and `track` and its wheel list are live.
        unsafe { JPH_TrackedVehicleController_SetTrack(controller, side, &track) }
    };

    // The left track drives wheel 0 and holds only wheel 0 (`other_vehicle_kinds`).
    assert!(set(JPH_TrackSide_Left, 0, &[0], 25.0));
    let expected = [25.0, 0.7, 9000.0, 5.0];
    assert_eq!(track_values(controller, JPH_TrackSide_Left), expected);
    assert_eq!(track_values(controller, JPH_TrackSide_Right), right_before);

    assert!(
        !set(JPH_TrackSide_Left, 1, &[0], 99.0),
        "another driven wheel"
    );
    assert!(!set(JPH_TrackSide_Left, 0, &[1], 99.0), "another wheel");
    assert!(!set(JPH_TrackSide_Left, 0, &[0, 1], 99.0), "more wheels");
    assert!(!set(JPH_TrackSide_Left, 0, &[], 99.0), "no wheels");
    let invalid_side: JPH_TrackSide = 2;
    assert!(!set(invalid_side, 0, &[0], 99.0), "no such side");
    assert_eq!(track_values(controller, JPH_TrackSide_Left), expected);
    assert_eq!(track_values(controller, JPH_TrackSide_Right), right_before);

    destroy_vehicles(&world, &[car, tank]);
}

/// Counts its calls in the `AtomicU32` behind `user_data` and returns Jolt's default impulses.
///
/// # Safety
/// `user_data` points to a live `AtomicU32`; the output pointers are valid for writes, as Jolt
/// passes them.
unsafe extern "C" fn counting_tire_callback(
    user_data: *mut c_void,
    _wheel_index: u32,
    out_longitudinal_impulse: *mut f32,
    out_lateral_impulse: *mut f32,
    suspension_impulse: f32,
    longitudinal_friction: f32,
    lateral_friction: f32,
    _longitudinal_slip: f32,
    _lateral_slip: f32,
    _delta_time: f32,
) {
    // SAFETY: the caller's contract; nothing here can panic.
    unsafe {
        (*user_data.cast::<AtomicU32>()).fetch_add(1, Ordering::Relaxed);
        *out_longitudinal_impulse = longitudinal_friction * suspension_impulse;
        *out_lateral_impulse = lateral_friction * suspension_impulse;
    }
}

#[test]
fn tire_callback_reset_stops_calling_the_old_callback() {
    let world = TestWorld::new(1);
    let floor = create_floor(&world);
    let (car, chassis) = create_car(&world, 1.0);
    register(&world, car);
    let controller = controller_of::<JPH_WheeledVehicleController>(car);
    let calls = AtomicU32::new(0);
    // SAFETY: the controller is live and no step runs. `calls` outlives every step that can call
    // the callback: the reset below comes before it goes out of scope.
    unsafe {
        JPH_WheeledVehicleController_SetDriverInput(controller, 1.0, 0.0, 0.0, 0.0);
        JPH_WheeledVehicleController_SetTireMaxImpulseCallback(
            controller,
            Some(counting_tire_callback),
            std::ptr::from_ref(&calls).cast_mut().cast(),
        );
    }

    let has_contact = || {
        // SAFETY: the constraint is live and owns its wheels; no step runs.
        (0..4).all(|index| unsafe {
            JPH_Wheel_HasContact(JPH_VehicleConstraint_GetWheel(car, index))
        })
    };
    for _ in 0..10 {
        world.step(DT);
    }
    let after_ten = calls.load(Ordering::Relaxed);
    for _ in 0..10 {
        world.step(DT);
    }
    let after_twenty = calls.load(Ordering::Relaxed);
    assert!(has_contact());
    assert!(
        after_ten > 0 && after_twenty > after_ten,
        "{after_ten}, {after_twenty}"
    );

    // SAFETY: the controller is live and no step runs.
    unsafe { JPH_WheeledVehicleController_ResetTireMaxImpulseCallback(controller) };
    for _ in 0..10 {
        world.step(DT);
    }
    assert!(has_contact());
    assert_eq!(calls.load(Ordering::Relaxed), after_twenty);

    unregister(&world, car);
    destroy_vehicles(&world, &[(car, chassis)]);
    // SAFETY: the id names the live floor.
    unsafe { JPH_BodyInterface_RemoveAndDestroyBody(world.body_interface(), floor) };
}
