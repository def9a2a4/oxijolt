//! Tire max impulse callbacks of a car and a motorcycle: the caps they return change the drive,
//! Jolt's default formula through a callback matches no callback bit for bit, invalid caps
//! count as 0, unbounded caps step finitely, wheels heavy against their chassis refuse a callback,
//! panics are resumed by `step`, and callbacks live as long as their vehicle.

mod common;

use std::any::Any;
use std::panic::{catch_unwind, panic_any, AssertUnwindSafe};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};

use common::vehicle::{
    add_car_with, car_world, chassis_settings, record_vehicle, CarLayers, GRAVITY,
};
use common::vehicle_kinds::{add_bike, ride};
use common::*;
use oxijolt::*;

/// A vehicle on flat ground under gravity, with its chassis.
struct Scene<K: VehicleKind> {
    world: PhysicsWorld,
    layers: CarLayers,
    chassis: BodyId,
    id: VehicleId<K>,
}

/// The vehicle kinds with a tire callback: how to build a scene and set the input.
trait Rig: WheeledControllerKind + Sized {
    fn scene() -> Scene<Self>;
    fn input(world: &mut PhysicsWorld, id: VehicleId<Self>, input: DriverInput);
}

/// A static box ground whose top face is the plane y = 0.
fn add_ground(world: &mut PhysicsWorld, layers: &CarLayers) {
    world
        .create_body(
            &Shape::new_box(Vec3::new(100.0, 1.0, 100.0)).unwrap(),
            &BodySettings::new_static()
                .position(RVec3::new(0.0, -1.0, 0.0))
                .object_layer(layers.ground),
        )
        .unwrap();
}

/// The shared test car at `x`, under its gravity override.
fn add_test_car(world: &mut PhysicsWorld, layers: &CarLayers, x: Real) -> (BodyId, VehicleId) {
    let body = chassis_settings(layers, RVec3::new(x, 1.0, 0.0), Quat::IDENTITY);
    let (chassis, id) = add_car_with(world, &body, VehicleCollisionTester::ray(layers.probe));
    world.vehicle_mut(id).unwrap().set_gravity(GRAVITY).unwrap();
    (chassis, id)
}

impl Rig for WheeledVehicle {
    fn scene() -> Scene<Self> {
        let (mut world, layers) = car_world(GRAVITY, 1);
        add_ground(&mut world, &layers);
        let (chassis, id) = add_test_car(&mut world, &layers, 0.0);
        Scene {
            world,
            layers,
            chassis,
            id,
        }
    }

    fn input(world: &mut PhysicsWorld, id: VehicleId<Self>, input: DriverInput) {
        world
            .vehicle_mut(id)
            .unwrap()
            .set_driver_input(input)
            .unwrap();
    }
}

impl Rig for Motorcycle {
    fn scene() -> Scene<Self> {
        let (mut world, layers) = car_world(GRAVITY, 1);
        add_ground(&mut world, &layers);
        let (chassis, id) = add_bike(
            &mut world,
            &layers,
            RVec3::new(0.0, 1.0, 0.0),
            Quat::IDENTITY,
        );
        Scene {
            world,
            layers,
            chassis,
            id,
        }
    }

    fn input(world: &mut PhysicsWorld, id: VehicleId<Self>, input: DriverInput) {
        world
            .vehicle_mut(id)
            .unwrap()
            .set_driver_input(input)
            .unwrap();
    }
}

/// Full throttle, straight ahead.
fn throttle() -> DriverInput {
    ride(1.0, 0.0)
}

/// Accelerate, turn right, then left.
fn drive_and_steer(tick: usize) -> DriverInput {
    match tick {
        0..40 => ride(1.0, 0.0),
        40..80 => ride(0.6, 0.3),
        _ => ride(0.4, -0.3),
    }
}

/// Runs `ticks` ticks of `input` from `first` on and returns the vehicle's state of each tick.
fn run<K: Rig>(
    scene: &mut Scene<K>,
    first: usize,
    ticks: usize,
    input: impl Fn(usize) -> DriverInput,
) -> Vec<Vec<u8>> {
    (first..first + ticks)
        .map(|tick| {
            K::input(&mut scene.world, scene.id, input(tick));
            step(&mut scene.world, 1);
            let mut state = Vec::new();
            record_vehicle(&scene.world, scene.id, &mut state);
            state
        })
        .collect()
}

/// The first tick at which two runs differ.
fn first_difference(a: &[Vec<u8>], b: &[Vec<u8>]) -> Option<usize> {
    assert_eq!(a.len(), b.len());
    a.iter().zip(b).position(|(a, b)| a != b)
}

/// Horizontal speed of the chassis, m/s.
fn horizontal_speed(world: &PhysicsWorld, chassis: BodyId) -> f32 {
    let velocity = world.body(chassis).unwrap().linear_velocity();
    velocity.x.hypot(velocity.z)
}

/// Horizontal distance of the chassis from `start`, m.
fn travelled(world: &PhysicsWorld, chassis: BodyId, start: RVec3) -> Real {
    let position = world.body(chassis).unwrap().position();
    (position.x - start.x).hypot(position.z - start.z)
}

/// Caps of 0 in both directions.
fn no_grip(_: &TireMaxImpulseInput) -> TireMaxImpulse {
    TireMaxImpulse::default()
}

#[test]
fn zero_tire_impulse_gives_no_drive() {
    let mut control = WheeledVehicle::scene();
    let start = control.world.body(control.chassis).unwrap().position();
    run(&mut control, 0, 120, |_| throttle());
    let moved = travelled(&control.world, control.chassis, start);
    assert!(moved > 1.0, "{moved}");

    let mut scene = WheeledVehicle::scene();
    scene
        .world
        .vehicle_mut(scene.id)
        .unwrap()
        .set_tire_max_impulse_callback(no_grip)
        .unwrap();
    run(&mut scene, 0, 120, |_| throttle());
    let speed = horizontal_speed(&scene.world, scene.chassis);
    assert!(speed < 1.0e-3, "{speed}");
    let spin = |scene: &Scene<WheeledVehicle>| {
        let vehicle = scene.world.vehicle(scene.id).unwrap();
        let wheels = vehicle.wheels();
        assert!(wheels.iter().all(|wheel| wheel.contact.is_some()));
        // The front wheels are driven.
        [wheels[0].angular_velocity, wheels[1].angular_velocity]
    };
    let (free, gripping) = (spin(&scene), spin(&control));
    eprintln!("driven wheels: {free:?} rad/s without grip, {gripping:?} with; moved {moved} m");
    for (free, gripping) in free.iter().zip(gripping) {
        assert!(free.abs() > gripping.abs(), "{free} vs {gripping}");
    }
}

fn default_formula_matches<K: Rig>() {
    let mut plain = K::scene();
    let mut through = K::scene();
    through
        .world
        .vehicle_mut(through.id)
        .unwrap()
        .set_tire_max_impulse_callback(|input| input.default_max_impulse())
        .unwrap();
    let a = run(&mut plain, 0, 120, drive_and_steer);
    let b = run(&mut through, 0, 120, drive_and_steer);
    assert_eq!(first_difference(&a, &b), None);
    assert!(through
        .world
        .vehicle(through.id)
        .unwrap()
        .has_tire_max_impulse_callback());
    assert!(!plain
        .world
        .vehicle(plain.id)
        .unwrap()
        .has_tire_max_impulse_callback());
}

#[test]
fn default_formula_matches_no_callback() {
    default_formula_matches::<WheeledVehicle>();
    default_formula_matches::<Motorcycle>();
}

/// Direction of the chassis' horizontal velocity, radians about +Y.
fn heading(world: &PhysicsWorld, chassis: BodyId) -> f32 {
    let velocity = world.body(chassis).unwrap().linear_velocity();
    velocity.x.atan2(velocity.z)
}

#[test]
fn reduced_lateral_grip_widens_the_turn() {
    let turn = |tick: usize| {
        if tick < 60 {
            ride(1.0, 0.0)
        } else {
            ride(0.5, 0.5)
        }
    };
    let radius = |slippery: bool| {
        let mut scene = WheeledVehicle::scene();
        run(&mut scene, 0, 120, turn);
        if slippery {
            scene
                .world
                .vehicle_mut(scene.id)
                .unwrap()
                .set_tire_max_impulse_callback(|input| {
                    let default = input.default_max_impulse();
                    TireMaxImpulse {
                        lateral: 0.2 * default.lateral,
                        ..default
                    }
                })
                .unwrap();
        }
        run(&mut scene, 120, 60, turn);
        // Distance over turned angle of the path in the last 20 ticks.
        let mut distance = 0.0;
        let mut previous = heading(&scene.world, scene.chassis);
        let mut turned = 0.0;
        for tick in 180..200 {
            run(&mut scene, tick, 1, turn);
            distance += horizontal_speed(&scene.world, scene.chassis) * DT;
            let now = heading(&scene.world, scene.chassis);
            let delta = (now - previous + 3.0 * std::f32::consts::PI)
                .rem_euclid(2.0 * std::f32::consts::PI)
                - std::f32::consts::PI;
            turned += delta;
            previous = now;
        }
        distance / turned.abs()
    };
    let (gripping, slippery) = (radius(false), radius(true));
    eprintln!("path radius: {gripping} m with Jolt's grip, {slippery} m with 0.2 of it sideways");
    assert!(slippery > 1.2 * gripping, "{slippery} vs {gripping}");
}

#[test]
fn invalid_outputs_count_as_zero() {
    let with = |longitudinal: bool, value: f32| {
        move |input: &TireMaxImpulseInput| {
            let mut caps = input.default_max_impulse();
            if longitudinal {
                caps.longitudinal = value;
            } else {
                caps.lateral = value;
            }
            caps
        }
    };
    for longitudinal in [true, false] {
        let mut zero = WheeledVehicle::scene();
        zero.world
            .vehicle_mut(zero.id)
            .unwrap()
            .set_tire_max_impulse_callback(with(longitudinal, 0.0))
            .unwrap();
        let expected = run(&mut zero, 0, 60, drive_and_steer);
        for value in [f32::NAN, -1.0, f32::NEG_INFINITY] {
            let mut scene = WheeledVehicle::scene();
            scene
                .world
                .vehicle_mut(scene.id)
                .unwrap()
                .set_tire_max_impulse_callback(with(longitudinal, value))
                .unwrap();
            let actual = run(&mut scene, 0, 60, drive_and_steer);
            assert_eq!(
                first_difference(&expected, &actual),
                None,
                "longitudinal {longitudinal}, value {value}"
            );
        }
    }
}

/// Whether the chassis and every wheel of the vehicle hold finite values.
fn is_finite<K: VehicleKind>(world: &PhysicsWorld, id: VehicleId<K>) -> bool {
    let vehicle = world.vehicle(id).unwrap();
    let body = world.body(vehicle.body()).unwrap();
    let position = body.position();
    let finite3 = |v: Vec3| v.x.is_finite() && v.y.is_finite() && v.z.is_finite();
    position.x.is_finite()
        && position.y.is_finite()
        && position.z.is_finite()
        && finite3(body.linear_velocity())
        && finite3(body.angular_velocity())
        && vehicle.wheels().iter().all(|wheel| {
            wheel.angular_velocity.is_finite()
                && wheel.suspension_length.is_finite()
                && wheel.suspension_lambda.is_finite()
        })
        && vehicle.engine_rpm().is_finite()
}

/// Drive, full steering, brake, hand brake while steering, then reverse, in collision steps of
/// 1/60 s, with every cap at `cap`; a sideways push starts the hand brake phase.
fn unbounded_caps<K: Rig>(cap: f32) {
    let mut scene = K::scene();
    scene
        .world
        .vehicle_mut(scene.id)
        .unwrap()
        .set_tire_max_impulse_callback(move |_| TireMaxImpulse {
            longitudinal: cap,
            lateral: cap,
        })
        .unwrap();
    for tick in 0..150 {
        let input = match tick {
            0..40 => ride(1.0, 0.0),
            40..60 => ride(1.0, 1.0),
            60..80 => DriverInput {
                brake: 1.0,
                ..DriverInput::default()
            },
            80..120 => DriverInput {
                forward: 0.5,
                right: -1.0,
                hand_brake: 1.0,
                ..DriverInput::default()
            },
            _ => ride(-1.0, 0.5),
        };
        if tick == 80 {
            let mut body = scene.world.body_mut(scene.chassis).unwrap();
            let velocity = body.linear_velocity();
            body.set_linear_velocity(Vec3::new(velocity.x + 8.0, velocity.y, velocity.z))
                .unwrap();
        }
        K::input(&mut scene.world, scene.id, input);
        let report = scene.world.step_with_collision_steps(2.0 * DT, 2).unwrap();
        assert!(report.is_complete());
        assert!(is_finite(&scene.world, scene.id), "cap {cap}, tick {tick}");
    }
}

#[test]
fn unbounded_caps_stay_finite() {
    for cap in [f32::INFINITY, f32::MAX] {
        unbounded_caps::<WheeledVehicle>(cap);
        unbounded_caps::<Motorcycle>(cap);
    }
}

/// A box chassis for the wheel mass ratio tests: its half extents, and where its front right
/// wheel is attached; the other three mirror it.
#[derive(Clone, Copy, Debug)]
struct Hull {
    half_extents: Vec3,
    front_right_wheel: Vec3,
}

/// A car-shaped box with its wheels under its corners.
const BOX_CAR: Hull = Hull {
    half_extents: Vec3::new(0.9, 0.3, 2.0),
    front_right_wheel: Vec3::new(0.9, -0.1, 1.4),
};

/// A tall narrow box with its wheels 3 m ahead and behind: its yaw inertia is small against
/// its pitch and roll inertia, so a steered wheel pushes it much harder sideways than forward.
const POLE: Hull = Hull {
    half_extents: Vec3::new(0.3, 2.0, 0.3),
    front_right_wheel: Vec3::new(0.5, -1.9, 3.0),
};

/// A `hull` chassis of `mass` (centre of mass in its middle) on four wheels of radius 0.35 and
/// moment of inertia `inertia`, the front ones steering up to `steer` radians, settled on flat
/// ground for a second.
fn heavy_wheel_car(
    hull: Hull,
    mass: f32,
    inertia: f32,
    steer: f32,
) -> (PhysicsWorld, BodyId, VehicleId) {
    let mut world = PhysicsWorld::new(WorldSettings::default()).unwrap();
    let floor = Shape::new_box(Vec3::new(500.0, 1.0, 500.0)).unwrap();
    let ground = BodySettings::new_static().position(RVec3::new(0.0, -1.0, 0.0));
    world.create_body(&floor, &ground).unwrap();
    let Vec3 { x, y, z } = hull.front_right_wheel;
    // The longest suspension of 0.5 m plus the wheel's radius just reaches the ground.
    let height = 0.9 - y;
    let body = BodySettings::new_dynamic()
        .mass(mass)
        .allow_sleeping(false)
        .position(RVec3::new(0.0, Real::from(height), 0.0));
    let chassis = world
        .create_body(&Shape::new_box(hull.half_extents).unwrap(), &body)
        .unwrap();
    let wheel = |x: f32, z: f32, steer: f32| {
        WheelSettings::new(Vec3::new(x, y, z))
            .radius(0.35)
            .inertia(inertia)
            .max_steer_angle(steer)
    };
    let settings = WheeledVehicleSettings::new(
        vec![
            wheel(x, z, steer),
            wheel(-x, z, steer),
            wheel(x, -z, 0.0),
            wheel(-x, -z, 0.0),
        ],
        vec![VehicleDifferentialSettings::new(Some(0), Some(1))],
        VehicleCollisionTester::ray(ObjectLayer::MOVING),
    );
    let id = world.create_wheeled_vehicle(chassis, &settings).unwrap();
    step(&mut world, 60);
    (world, chassis, id)
}

/// Caps of `cap` in both directions.
fn caps_of(cap: f32) -> impl Fn(&TireMaxImpulseInput) -> TireMaxImpulse {
    move |_| TireMaxImpulse {
        longitudinal: cap,
        lateral: cap,
    }
}

/// Whether the vehicle is finite and its chassis below Jolt's velocity clamps (500 m/s,
/// 15π rad/s), which an overflowing clamp would leave at exactly zero.
fn is_unclamped(world: &PhysicsWorld, id: VehicleId) -> bool {
    let chassis = world.body(world.vehicle(id).unwrap().body()).unwrap();
    let length = |v: Vec3| (v.x * v.x + v.y * v.y + v.z * v.z).sqrt();
    let speed = length(chassis.linear_velocity());
    let spin = length(chassis.angular_velocity());
    is_finite(world, id) && speed < 400.0 && spin < 40.0 && speed > 0.0
}

/// Drives the car from 10 m/s through throttle, full steering, brakes and reverse in steps of
/// `dt`, and asserts after every step that it stays finite and below the clamps.
fn drive_from_10_m_s(world: &mut PhysicsWorld, chassis: BodyId, id: VehicleId, dt: f32) {
    let mut body = world.body_mut(chassis).unwrap();
    body.set_linear_velocity(Vec3::new(0.0, 0.0, 10.0)).unwrap();
    let steps = (4.0 / dt).round() as usize;
    for tick in 0..steps {
        let input = match 4 * tick / steps {
            0 => ride(1.0, 0.0),
            1 => ride(1.0, 1.0),
            2 => DriverInput {
                brake: 1.0,
                right: -1.0,
                ..DriverInput::default()
            },
            _ => ride(-1.0, -0.5),
        };
        world
            .vehicle_mut(id)
            .unwrap()
            .set_driver_input(input)
            .unwrap();
        assert!(world.step(dt).unwrap().is_complete());
        assert!(is_unclamped(world, id), "dt {dt}, tick {tick}");
    }
}

/// Jolt's default largest steering angle of a wheel, 70°.
const DEFAULT_STEER: f32 = 70.0 * std::f32::consts::PI / 180.0;

/// Whether `result` is the wheel mass ratio's refusal.
fn is_mass_ratio_refusal(result: &Result<(), VehicleError>) -> bool {
    matches!(result, Err(VehicleError::InvalidValue(what)) if what.contains("MAX_WHEEL_MASS_RATIO"))
}

#[test]
fn wheels_heavy_against_the_chassis_refuse_a_callback() {
    for inertia in [1000.0, 1.0e25] {
        let (mut world, chassis, id) = heavy_wheel_car(BOX_CAR, 1500.0, inertia, DEFAULT_STEER);
        for cap in [f32::INFINITY, f32::MAX] {
            let refused = world
                .vehicle_mut(id)
                .unwrap()
                .set_tire_max_impulse_callback(caps_of(cap));
            assert!(is_mass_ratio_refusal(&refused), "{refused:?}");
            assert!(!world.vehicle(id).unwrap().has_tire_max_impulse_callback());
        }
        // Jolt's own caps keep such wheels finite.
        drive_from_10_m_s(&mut world, chassis, id, DT);
    }
}

#[test]
fn steered_wheels_count_their_sideways_push() {
    // Wheels of 3.84 kg m^2 on the 1500 kg pole pass a ratio taken over the unsteered
    // forward/up plane only; steering 70° with unbounded caps, they threw the chassis to Jolt's
    // angular velocity clamp within three steps.
    let inertia = 3.84;
    let (mut world, chassis, id) = heavy_wheel_car(POLE, 1500.0, inertia, DEFAULT_STEER);
    let refused = world
        .vehicle_mut(id)
        .unwrap()
        .set_tire_max_impulse_callback(caps_of(f32::INFINITY));
    assert!(is_mass_ratio_refusal(&refused), "{refused:?}");
    drive_from_10_m_s(&mut world, chassis, id, DT);
    // Without steering they push the pole only forward and up, and take the callback.
    let (mut world, _, id) = heavy_wheel_car(POLE, 1500.0, inertia, 0.0);
    world
        .vehicle_mut(id)
        .unwrap()
        .set_tire_max_impulse_callback(caps_of(f32::INFINITY))
        .unwrap();
}

#[test]
fn a_tester_that_reaches_farther_is_refused_while_a_callback_is_installed() {
    let inertia = heaviest_accepted_inertia(BOX_CAR, 1500.0, DEFAULT_STEER);
    let (mut world, _, id) = heavy_wheel_car(BOX_CAR, 1500.0, inertia, DEFAULT_STEER);
    let mut vehicle = world.vehicle_mut(id).unwrap();
    // A sphere wider than the wheel puts the contact farther from the suspension than the ray,
    // so the same wheels are heavier against the chassis.
    let sphere = VehicleCollisionTester::cast_sphere(ObjectLayer::MOVING, 0.8);
    vehicle.set_collision_tester(sphere).unwrap();
    let refused = vehicle.set_tire_max_impulse_callback(caps_of(f32::INFINITY));
    assert!(is_mass_ratio_refusal(&refused), "{refused:?}");
    let ray = VehicleCollisionTester::ray(ObjectLayer::MOVING);
    vehicle.set_collision_tester(ray).unwrap();
    vehicle
        .set_tire_max_impulse_callback(caps_of(f32::INFINITY))
        .unwrap();
    let refused = vehicle.set_collision_tester(sphere);
    assert!(is_mass_ratio_refusal(&refused), "{refused:?}");
    assert_eq!(world.vehicle(id).unwrap().collision_tester(), ray);
    // Without a callback the tester is free again.
    let mut vehicle = world.vehicle_mut(id).unwrap();
    vehicle.clear_tire_max_impulse_callback();
    vehicle.set_collision_tester(sphere).unwrap();
}

/// The heaviest wheel inertia the `hull` car of `mass` with front wheels steering up to
/// `steer` takes a callback with, within 0.1 %.
fn heaviest_accepted_inertia(hull: Hull, mass: f32, steer: f32) -> f32 {
    let accepts = |inertia: f32| {
        let (mut world, _, id) = heavy_wheel_car(hull, mass, inertia, steer);
        let accepted = world
            .vehicle_mut(id)
            .unwrap()
            .set_tire_max_impulse_callback(caps_of(f32::INFINITY))
            .is_ok();
        accepted
    };
    let (mut light, mut heavy) = (0.01, 1000.0);
    assert!(accepts(light) && !accepts(heavy), "{hull:?}, mass {mass}");
    while heavy > 1.001 * light {
        let middle = (light * heavy).sqrt();
        if accepts(middle) {
            light = middle;
        } else {
            heavy = middle;
        }
    }
    light
}

#[test]
fn the_heaviest_accepted_wheels_with_unbounded_caps_stay_finite() {
    let chassis = [(BOX_CAR, 1500.0), (BOX_CAR, 300.0), (POLE, 1500.0)];
    for (hull, mass) in chassis {
        for steer_degrees in [0.0_f32, 30.0, 70.0, 89.0] {
            let steer = steer_degrees.to_radians();
            let inertia = heaviest_accepted_inertia(hull, mass, steer);
            eprintln!(
                "{hull:?}, {mass} kg, steering {steer_degrees}°: wheels up to {inertia} kg m^2 take a callback"
            );
            for cap in [f32::INFINITY, f32::MAX] {
                for dt in [DT, 0.1] {
                    let (mut world, chassis, id) = heavy_wheel_car(hull, mass, inertia, steer);
                    world
                        .vehicle_mut(id)
                        .unwrap()
                        .set_tire_max_impulse_callback(caps_of(cap))
                        .unwrap();
                    drive_from_10_m_s(&mut world, chassis, id, dt);
                }
            }
        }
    }
}

#[test]
fn the_callback_sees_contacting_wheels() {
    let mut scene = WheeledVehicle::scene();
    run(&mut scene, 0, 30, |_| ride(0.5, 0.3));
    let seen = Arc::new(Mutex::new(Vec::new()));
    let record = Arc::clone(&seen);
    scene
        .world
        .vehicle_mut(scene.id)
        .unwrap()
        .set_tire_max_impulse_callback(move |input| {
            record.lock().unwrap().push((input.wheel, input.delta_time));
            input.default_max_impulse()
        })
        .unwrap();
    for _ in 0..10 {
        let report = scene
            .world
            .step_with_collision_steps(1.0 / 30.0, 2)
            .unwrap();
        assert!(report.is_complete());
    }
    let seen = seen.lock().unwrap();
    assert!(!seen.is_empty());
    for &(wheel, delta_time) in seen.iter() {
        assert!(wheel < 4, "{wheel}");
        assert_eq!(delta_time.to_bits(), (1.0_f32 / 60.0).to_bits());
    }
    for wheel in 0..4 {
        assert!(seen.iter().any(|&(seen, _)| seen == wheel), "wheel {wheel}");
    }

    // Without ground and gravity no wheel touches anything.
    let (mut world, layers) = car_world(Vec3::ZERO, 1);
    let body = chassis_settings(&layers, RVec3::new(0.0, 1.0, 0.0), Quat::IDENTITY);
    let (_, id) = add_car_with(&mut world, &body, VehicleCollisionTester::ray(layers.probe));
    let calls = Arc::new(AtomicUsize::new(0));
    let count = Arc::clone(&calls);
    let mut vehicle = world.vehicle_mut(id).unwrap();
    vehicle.set_driver_input(throttle()).unwrap();
    vehicle
        .set_tire_max_impulse_callback(move |input| {
            count.fetch_add(1, Ordering::Relaxed);
            input.default_max_impulse()
        })
        .unwrap();
    step(&mut world, 30);
    assert_eq!(calls.load(Ordering::Relaxed), 0);
}

#[test]
fn clear_restores_jolts_default() {
    let mut scene = WheeledVehicle::scene();
    run(&mut scene, 0, 30, drive_and_steer);
    let saved = scene.world.save_state();
    let untouched = run(&mut scene, 30, 60, drive_and_steer);

    scene.world.restore_state(&saved).unwrap();
    let mut vehicle = scene.world.vehicle_mut(scene.id).unwrap();
    vehicle.set_tire_max_impulse_callback(no_grip).unwrap();
    vehicle.clear_tire_max_impulse_callback();
    // Clearing twice does nothing.
    vehicle.clear_tire_max_impulse_callback();
    assert!(!scene
        .world
        .vehicle(scene.id)
        .unwrap()
        .has_tire_max_impulse_callback());
    let cleared = run(&mut scene, 30, 60, drive_and_steer);
    assert_eq!(first_difference(&untouched, &cleared), None);
}

/// A panic payload whose drop panics again.
struct PanicsOnDrop;

impl Drop for PanicsOnDrop {
    fn drop(&mut self) {
        panic!("tire panic payload dropped");
    }
}

/// A contact listener that panics for every added or persisted contact.
struct PanickingListener;

impl ContactListener for PanickingListener {
    fn contact_added(&self, _: &ContactManifold, _: &mut ContactSettings) {
        panic!("contact listener panic");
    }

    fn contact_persisted(&self, _: &ContactManifold, _: &mut ContactSettings) {
        panic!("contact listener panic");
    }
}

/// The message of a `panic!` payload.
fn message(payload: &(dyn Any + Send)) -> &str {
    payload
        .downcast_ref::<&str>()
        .copied()
        .or_else(|| payload.downcast_ref::<String>().map(String::as_str))
        .unwrap_or("<not a message>")
}

#[test]
fn a_panicking_callback_panics_out_of_step_and_the_world_stays_usable() {
    let mut scene = WheeledVehicle::scene();
    run(&mut scene, 0, 20, |_| throttle());
    scene
        .world
        .vehicle_mut(scene.id)
        .unwrap()
        .set_tire_max_impulse_callback(|_| panic!("tire callback panic"))
        .unwrap();
    for _ in 0..3 {
        let payload = catch_unwind(AssertUnwindSafe(|| scene.world.step(DT)))
            .expect_err("the callback panics in every step");
        assert_eq!(message(&*payload), "tire callback panic");
        assert!(is_finite(&scene.world, scene.id));
    }

    // Two cars and a contact listener panic in the same step: one payload resumes, every slot
    // is drained, and the payload whose drop panics is dropped without unwinding.
    let (second_chassis, second) = add_test_car(&mut scene.world, &scene.layers, 6.0);
    let cube = BodySettings::new_dynamic()
        .position(RVec3::new(-6.0, 0.5, 0.0))
        .object_layer(scene.layers.moving)
        .allow_sleeping(false);
    let cube_shape = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    scene.world.create_body(&cube_shape, &cube).unwrap();
    scene
        .world
        .vehicle_mut(scene.id)
        .unwrap()
        .set_tire_max_impulse_callback(|input| input.default_max_impulse())
        .unwrap();
    step(&mut scene.world, 30);
    scene
        .world
        .set_contact_listener(Some(Arc::new(PanickingListener)));
    scene
        .world
        .vehicle_mut(scene.id)
        .unwrap()
        .set_tire_max_impulse_callback(|_| panic!("tire callback panic"))
        .unwrap();
    scene
        .world
        .vehicle_mut(second)
        .unwrap()
        .set_tire_max_impulse_callback(|_| panic_any(PanicsOnDrop))
        .unwrap();
    let payload = catch_unwind(AssertUnwindSafe(|| scene.world.step(DT)))
        .expect_err("the listener and both callbacks panic");
    assert_eq!(message(&*payload), "contact listener panic");

    scene.world.set_contact_listener(None);
    for id in [scene.id, second] {
        scene
            .world
            .vehicle_mut(id)
            .unwrap()
            .set_tire_max_impulse_callback(|input| input.default_max_impulse())
            .unwrap();
    }
    for _ in 0..60 {
        for id in [scene.id, second] {
            WheeledVehicle::input(&mut scene.world, id, throttle());
        }
        step(&mut scene.world, 1);
        assert!(is_finite(&scene.world, scene.id));
        assert!(is_finite(&scene.world, second));
    }
    assert!(horizontal_speed(&scene.world, second_chassis) > 1.0);
}

#[test]
fn after_a_panic_the_step_uses_jolts_default() {
    let mut plain = WheeledVehicle::scene();
    let mut panicking = WheeledVehicle::scene();
    let before = run(&mut plain, 0, 20, |_| throttle());
    assert_eq!(before, run(&mut panicking, 0, 20, |_| throttle()));
    // Panics on its first call; every later call counts and asks for no grip.
    let panicked = Arc::new(AtomicUsize::new(0));
    let later_calls = Arc::new(AtomicUsize::new(0));
    let (first, later) = (Arc::clone(&panicked), Arc::clone(&later_calls));
    panicking
        .world
        .vehicle_mut(panicking.id)
        .unwrap()
        .set_tire_max_impulse_callback(move |_| {
            if first.fetch_add(1, Ordering::Relaxed) == 0 {
                panic!("first call");
            }
            later.fetch_add(1, Ordering::Relaxed);
            TireMaxImpulse::default()
        })
        .unwrap();
    WheeledVehicle::input(&mut panicking.world, panicking.id, throttle());
    let payload = catch_unwind(AssertUnwindSafe(|| panicking.world.step(DT)))
        .expect_err("the first call panics");
    assert_eq!(message(&*payload), "first call");
    assert_eq!(later_calls.load(Ordering::Relaxed), 0);
    let mut previous = Vec::new();
    record_vehicle(&plain.world, plain.id, &mut previous);
    run(&mut plain, 20, 1, |_| throttle());
    let (mut with_default, mut actual) = (Vec::new(), Vec::new());
    record_vehicle(&plain.world, plain.id, &mut with_default);
    record_vehicle(&panicking.world, panicking.id, &mut actual);
    assert_ne!(previous, with_default);
    assert_eq!(
        with_default, actual,
        "the panicking step used Jolt's default caps"
    );

    // The next step calls the callback again.
    step(&mut panicking.world, 1);
    assert!(later_calls.load(Ordering::Relaxed) > 0);
}

#[test]
fn callbacks_are_dropped_with_replace_clear_remove_and_world_drop() {
    let token = Arc::new(());
    let holding = |token: &Arc<()>| {
        let token = Arc::clone(token);
        move |input: &TireMaxImpulseInput| {
            let _held = &token;
            input.default_max_impulse()
        }
    };
    let mut scene = WheeledVehicle::scene();
    let mut vehicle = scene.world.vehicle_mut(scene.id).unwrap();
    vehicle
        .set_tire_max_impulse_callback(holding(&token))
        .unwrap();
    assert_eq!(Arc::strong_count(&token), 2);
    run(&mut scene, 0, 10, |_| throttle());
    let mut vehicle = scene.world.vehicle_mut(scene.id).unwrap();
    vehicle.set_tire_max_impulse_callback(no_grip).unwrap();
    assert_eq!(Arc::strong_count(&token), 1, "replace");

    vehicle
        .set_tire_max_impulse_callback(holding(&token))
        .unwrap();
    vehicle.clear_tire_max_impulse_callback();
    assert_eq!(Arc::strong_count(&token), 1, "clear");

    let mut bike = Motorcycle::scene();
    bike.world
        .vehicle_mut(bike.id)
        .unwrap()
        .set_tire_max_impulse_callback(holding(&token))
        .unwrap();
    run(&mut bike, 0, 10, |_| throttle());
    assert_eq!(Arc::strong_count(&token), 2);
    bike.world.remove_vehicle(bike.id).unwrap();
    assert_eq!(Arc::strong_count(&token), 1, "remove");

    scene
        .world
        .vehicle_mut(scene.id)
        .unwrap()
        .set_tire_max_impulse_callback(holding(&token))
        .unwrap();
    run(&mut scene, 10, 10, |_| throttle());
    drop(scene);
    assert_eq!(Arc::strong_count(&token), 1, "world drop");
}

/// A callback that counts its calls and checks the wheel index, with Jolt's default caps.
fn counting(
    wheels: u32,
) -> (
    Arc<AtomicUsize>,
    impl Fn(&TireMaxImpulseInput) -> TireMaxImpulse,
) {
    let calls = Arc::new(AtomicUsize::new(0));
    let count = Arc::clone(&calls);
    let callback = move |input: &TireMaxImpulseInput| {
        assert!(input.wheel < wheels);
        count.fetch_add(1, Ordering::Relaxed);
        input.default_max_impulse()
    };
    (calls, callback)
}

#[test]
fn distinct_callbacks_per_vehicle() {
    let (mut world, layers) = car_world(GRAVITY, 1);
    add_ground(&mut world, &layers);
    let (stuck_chassis, stuck) = add_test_car(&mut world, &layers, -6.0);
    let (free_chassis, free) = add_test_car(&mut world, &layers, 6.0);
    let (_, bike) = add_bike(
        &mut world,
        &layers,
        RVec3::new(0.0, 1.0, 10.0),
        Quat::IDENTITY,
    );
    let (stuck_calls, stuck_counter) = counting(4);
    let (free_calls, free_counter) = counting(4);
    let (bike_calls, bike_counter) = counting(2);
    world
        .vehicle_mut(stuck)
        .unwrap()
        .set_tire_max_impulse_callback(move |input| {
            stuck_counter(input);
            TireMaxImpulse::default()
        })
        .unwrap();
    world
        .vehicle_mut(free)
        .unwrap()
        .set_tire_max_impulse_callback(move |input| free_counter(input))
        .unwrap();
    world
        .vehicle_mut(bike)
        .unwrap()
        .set_tire_max_impulse_callback(move |input| bike_counter(input))
        .unwrap();
    for _ in 0..90 {
        for id in [stuck, free] {
            WheeledVehicle::input(&mut world, id, throttle());
        }
        Motorcycle::input(&mut world, bike, ride(0.3, 0.0));
        step(&mut world, 1);
    }
    for calls in [&stuck_calls, &free_calls, &bike_calls] {
        assert!(calls.load(Ordering::Relaxed) > 0);
    }
    assert!(horizontal_speed(&world, stuck_chassis) < 1.0e-3);
    assert!(horizontal_speed(&world, free_chassis) > 1.0);
}

#[test]
fn callback_survives_restore_and_rebase() {
    let mut scene = WheeledVehicle::scene();
    scene
        .world
        .vehicle_mut(scene.id)
        .unwrap()
        .set_tire_max_impulse_callback(no_grip)
        .unwrap();
    run(&mut scene, 0, 10, |_| throttle());
    let saved = scene.world.save_state();
    run(&mut scene, 10, 30, |_| throttle());
    scene.world.restore_state(&saved).unwrap();
    assert!(scene
        .world
        .vehicle(scene.id)
        .unwrap()
        .has_tire_max_impulse_callback());
    run(&mut scene, 10, 60, |_| throttle());
    let speed = horizontal_speed(&scene.world, scene.chassis);
    assert!(speed < 1.0e-3, "after restore: {speed}");

    let bodies: Vec<BodyId> = scene.world.body_ids().collect();
    let half = 0.25 * std::f32::consts::PI;
    let rotation = Quat::from_xyzw(0.0, half.sin(), 0.0, half.cos());
    scene
        .world
        .rebase(&bodies, rotation, RVec3::new(100.0, 0.0, -50.0))
        .unwrap();
    assert!(scene
        .world
        .vehicle(scene.id)
        .unwrap()
        .has_tire_max_impulse_callback());
    run(&mut scene, 70, 60, |_| throttle());
    let speed = horizontal_speed(&scene.world, scene.chassis);
    assert!(speed < 1.0e-3, "after rebase: {speed}");
}
