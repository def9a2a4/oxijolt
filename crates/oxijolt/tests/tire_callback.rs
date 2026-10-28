//! Tire max impulse callbacks of a car and a motorcycle: the friction they return changes the
//! drive, Jolt's own friction through a callback matches no callback bit for bit, coefficients
//! are clamped to `0..=MAX_FRICTION`, the largest friction of a callback matches friction curves
//! of the largest friction, panics are resumed by `step`, and callbacks live as long as their
//! vehicle.

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

/// No friction in either direction.
fn no_grip(_: &TireMaxImpulseInput) -> TireFriction {
    TireFriction::default()
}

/// Jolt's friction with the longitudinal coefficient scaled by `factor`.
fn longitudinal_grip(factor: f32) -> impl Fn(&TireMaxImpulseInput) -> TireFriction {
    move |input| {
        let friction = input.default_friction();
        TireFriction {
            longitudinal: factor * friction.longitudinal,
            ..friction
        }
    }
}

/// The angular velocities of the car's driven front wheels, which all touch the ground.
fn driven_wheel_spin(scene: &Scene<WheeledVehicle>) -> [f32; 2] {
    let vehicle = scene.world.vehicle(scene.id).unwrap();
    let wheels = vehicle.wheels();
    assert!(wheels.iter().all(|wheel| wheel.contact.is_some()));
    [wheels[0].angular_velocity, wheels[1].angular_velocity]
}

#[test]
fn zero_friction_gives_no_drive() {
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
        .set_tire_max_impulse_callback(no_grip);
    run(&mut scene, 0, 120, |_| throttle());
    let speed = horizontal_speed(&scene.world, scene.chassis);
    assert!(speed < 1.0e-3, "{speed}");
    let (free, gripping) = (driven_wheel_spin(&scene), driven_wheel_spin(&control));
    eprintln!("driven wheels: {free:?} rad/s without grip, {gripping:?} with; moved {moved} m");
    for (free, gripping) in free.iter().zip(gripping) {
        assert!(free.abs() > gripping.abs(), "{free} vs {gripping}");
    }
}

#[test]
fn less_longitudinal_friction_spins_the_driven_wheels() {
    let drive = |factor: f32| {
        let mut scene = WheeledVehicle::scene();
        let start = scene.world.body(scene.chassis).unwrap().position();
        scene
            .world
            .vehicle_mut(scene.id)
            .unwrap()
            .set_tire_max_impulse_callback(longitudinal_grip(factor));
        run(&mut scene, 0, 60, |_| throttle());
        let moved = travelled(&scene.world, scene.chassis, start);
        (moved, driven_wheel_spin(&scene))
    };
    let (gripping, gripping_spin) = drive(1.0);
    let (slipping, slipping_spin) = drive(0.2);
    eprintln!(
        "Jolt's grip: {gripping} m, wheels {gripping_spin:?} rad/s; \
         0.2 of it: {slipping} m, wheels {slipping_spin:?} rad/s"
    );
    assert!(slipping < 0.9 * gripping, "{slipping} vs {gripping}");
    for (slipping, gripping) in slipping_spin.iter().zip(gripping_spin) {
        assert!(slipping.abs() > gripping.abs(), "{slipping} vs {gripping}");
    }
}

fn default_friction_matches<K: Rig>() {
    let mut plain = K::scene();
    let mut through = K::scene();
    through
        .world
        .vehicle_mut(through.id)
        .unwrap()
        .set_tire_max_impulse_callback(|input| input.default_friction());
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
fn default_friction_matches_no_callback() {
    default_friction_matches::<WheeledVehicle>();
    default_friction_matches::<Motorcycle>();
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
                    let default = input.default_friction();
                    TireFriction {
                        lateral: 0.2 * default.lateral,
                        ..default
                    }
                });
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
fn outputs_are_clamped_to_the_friction_bounds() {
    let with = |longitudinal: bool, value: f32| {
        move |input: &TireMaxImpulseInput| {
            let mut friction = input.default_friction();
            if longitudinal {
                friction.longitudinal = value;
            } else {
                friction.lateral = value;
            }
            friction
        }
    };
    let mut plain = WheeledVehicle::scene();
    let jolt = run(&mut plain, 0, 60, drive_and_steer);
    let below = [f32::NAN, -1.0, f32::NEG_INFINITY];
    let above = [limits::MAX_FRICTION.next_up(), f32::MAX, f32::INFINITY];
    for longitudinal in [true, false] {
        for (bound, values) in [(0.0, below), (limits::MAX_FRICTION, above)] {
            let mut at_bound = WheeledVehicle::scene();
            at_bound
                .world
                .vehicle_mut(at_bound.id)
                .unwrap()
                .set_tire_max_impulse_callback(with(longitudinal, bound));
            let expected = run(&mut at_bound, 0, 60, drive_and_steer);
            assert!(
                first_difference(&jolt, &expected).is_some(),
                "longitudinal {longitudinal}, bound {bound}"
            );
            for value in values {
                let mut scene = WheeledVehicle::scene();
                scene
                    .world
                    .vehicle_mut(scene.id)
                    .unwrap()
                    .set_tire_max_impulse_callback(with(longitudinal, value));
                let actual = run(&mut scene, 0, 60, drive_and_steer);
                assert_eq!(
                    first_difference(&expected, &actual),
                    None,
                    "longitudinal {longitudinal}, value {value}"
                );
            }
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

/// The largest friction in both directions.
fn max_friction(_: &TireMaxImpulseInput) -> TireFriction {
    TireFriction {
        longitudinal: limits::MAX_FRICTION,
        lateral: limits::MAX_FRICTION,
    }
}

/// Infinite friction in both directions.
fn infinite_friction(_: &TireMaxImpulseInput) -> TireFriction {
    TireFriction {
        longitudinal: f32::INFINITY,
        lateral: f32::INFINITY,
    }
}

/// Drive, full steering, brake, hand brake while steering, then reverse, in collision steps of
/// 1/60 s, with the largest friction; a sideways push starts the hand brake phase.
fn largest_friction<K: Rig>() {
    let mut scene = K::scene();
    scene
        .world
        .vehicle_mut(scene.id)
        .unwrap()
        .set_tire_max_impulse_callback(max_friction);
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
        assert!(is_finite(&scene.world, scene.id), "tick {tick}");
    }
}

#[test]
fn the_largest_friction_stays_finite() {
    largest_friction::<WheeledVehicle>();
    largest_friction::<Motorcycle>();
}

/// A 1500 kg box chassis (centre of mass in its middle) on four wheels of radius 0.35 whose
/// tire impulses diverged under caps far above friction times the suspension impulse.
#[derive(Clone, Copy, Debug)]
struct HeavyWheels {
    half_extents: Vec3,
    /// Where the front right wheel is attached; the other three mirror it.
    front_right_wheel: Vec3,
    /// The wheels' moment of inertia, kg m^2.
    inertia: f32,
    /// The front wheels' largest steering angle, radians.
    steer: f32,
    /// The height of each wheel's suspension force point in the chassis, if it has one.
    force_point_height: Option<f32>,
}

/// Jolt's default largest steering angle of a wheel, 70°.
const DEFAULT_STEER: f32 = 70.0 * std::f32::consts::PI / 180.0;

const HEAVY_WHEELS: [HeavyWheels; 3] = [
    // A car-shaped box on wheels of 1000 kg m^2.
    HeavyWheels {
        half_extents: Vec3::new(0.9, 0.3, 2.0),
        front_right_wheel: Vec3::new(0.9, -0.1, 1.4),
        inertia: 1000.0,
        steer: DEFAULT_STEER,
        force_point_height: None,
    },
    // A tall narrow box with its wheels 3 m ahead and behind: its yaw inertia is small against
    // its pitch and roll inertia, so a steered wheel pushes it much harder sideways than forward.
    HeavyWheels {
        half_extents: Vec3::new(0.3, 2.0, 0.3),
        front_right_wheel: Vec3::new(0.5, -1.9, 3.0),
        inertia: 3.84,
        steer: DEFAULT_STEER,
        force_point_height: None,
    },
    // A compact box whose suspension pushes above the centre of mass, the other side from the
    // contact.
    HeavyWheels {
        half_extents: Vec3::new(0.5, 0.3, 0.5),
        front_right_wheel: Vec3::new(0.5, -0.3, 0.75),
        inertia: 8.4,
        steer: 0.0,
        force_point_height: Some(1.15),
    },
];

/// The `heavy` car on flat ground of friction `MAX_FRICTION`, settled for a second, with
/// `callback` installed before it settles. Without a callback its friction curves are
/// `MAX_FRICTION` at every slip, which Jolt combines with the ground's to `MAX_FRICTION`.
fn heavy_wheel_car(
    heavy: HeavyWheels,
    callback: Option<fn(&TireMaxImpulseInput) -> TireFriction>,
) -> (PhysicsWorld, BodyId, VehicleId) {
    let mut world = PhysicsWorld::new(WorldSettings::default()).unwrap();
    let floor = Shape::new_box(Vec3::new(500.0, 1.0, 500.0)).unwrap();
    let ground = BodySettings::new_static()
        .position(RVec3::new(0.0, -1.0, 0.0))
        .friction(limits::MAX_FRICTION);
    world.create_body(&floor, &ground).unwrap();
    let Vec3 { x, y, z } = heavy.front_right_wheel;
    // The longest suspension of 0.5 m plus the wheel's radius just reaches the ground.
    let height = 0.9 - y;
    let body = BodySettings::new_dynamic()
        .mass(1500.0)
        .allow_sleeping(false)
        .position(RVec3::new(0.0, Real::from(height), 0.0));
    let chassis = world
        .create_body(&Shape::new_box(heavy.half_extents).unwrap(), &body)
        .unwrap();
    let wheel = |x: f32, z: f32, steer: f32| {
        let wheel = WheelSettings::new(Vec3::new(x, y, z))
            .radius(0.35)
            .inertia(heavy.inertia)
            .max_steer_angle(steer)
            .suspension_force_point(heavy.force_point_height.map(|h| Vec3::new(x, h, z)));
        if callback.is_some() {
            return wheel;
        }
        let max = limits::MAX_FRICTION;
        wheel
            .longitudinal_friction(vec![(0.0, max), (1.0, max)])
            .lateral_friction(vec![(0.0, max), (90.0, max)])
    };
    let settings = WheeledVehicleSettings::new(
        vec![
            wheel(x, z, heavy.steer),
            wheel(-x, z, heavy.steer),
            wheel(x, -z, 0.0),
            wheel(-x, -z, 0.0),
        ],
        vec![VehicleDifferentialSettings::new(Some(0), Some(1))],
        VehicleCollisionTester::ray(ObjectLayer::MOVING),
    );
    let id = world.create_wheeled_vehicle(chassis, &settings).unwrap();
    if let Some(callback) = callback {
        world
            .vehicle_mut(id)
            .unwrap()
            .set_tire_max_impulse_callback(callback);
    }
    step(&mut world, 60);
    (world, chassis, id)
}

/// Drives the car from 10 m/s through throttle, full steering, brakes and reverse for 4 s in
/// steps of `dt` and returns its state after every step, and its largest chassis speed and
/// spin.
fn drive_from_10_m_s(
    world: &mut PhysicsWorld,
    chassis: BodyId,
    id: VehicleId,
    dt: f32,
) -> (Vec<Vec<u8>>, f32, f32) {
    let mut body = world.body_mut(chassis).unwrap();
    body.set_linear_velocity(Vec3::new(0.0, 0.0, 10.0)).unwrap();
    let steps = (4.0 / dt).round() as usize;
    let length = |v: Vec3| (v.x * v.x + v.y * v.y + v.z * v.z).sqrt();
    let (mut states, mut speed, mut spin) = (Vec::new(), 0.0_f32, 0.0_f32);
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
        let body = world.body(chassis).unwrap();
        speed = speed.max(length(body.linear_velocity()));
        spin = spin.max(length(body.angular_velocity()));
        let mut state = Vec::new();
        record_vehicle(world, id, &mut state);
        states.push(state);
    }
    (states, speed, spin)
}

#[test]
fn the_largest_friction_of_a_callback_matches_the_largest_friction_curves() {
    for heavy in HEAVY_WHEELS {
        for dt in [DT, 0.1] {
            let drive = |callback| {
                let (mut world, chassis, id) = heavy_wheel_car(heavy, callback);
                drive_from_10_m_s(&mut world, chassis, id, dt)
            };
            let (curves, speed, spin) = drive(None);
            eprintln!("{heavy:?}, dt {dt}: up to {speed} m/s and {spin} rad/s");
            // An infinite coefficient is clamped to the largest friction.
            for callback in [max_friction, infinite_friction] {
                let (callback, ..) = drive(Some(callback));
                assert_eq!(
                    first_difference(&curves, &callback),
                    None,
                    "{heavy:?}, dt {dt}"
                );
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
            input.default_friction()
        });
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
    vehicle.set_tire_max_impulse_callback(move |input| {
        count.fetch_add(1, Ordering::Relaxed);
        input.default_friction()
    });
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
    vehicle.set_tire_max_impulse_callback(no_grip);
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
        .set_tire_max_impulse_callback(|_| panic!("tire callback panic"));
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
        .set_tire_max_impulse_callback(|input| input.default_friction());
    step(&mut scene.world, 30);
    scene
        .world
        .set_contact_listener(Some(Arc::new(PanickingListener)));
    scene
        .world
        .vehicle_mut(scene.id)
        .unwrap()
        .set_tire_max_impulse_callback(|_| panic!("tire callback panic"));
    scene
        .world
        .vehicle_mut(second)
        .unwrap()
        .set_tire_max_impulse_callback(|_| panic_any(PanicsOnDrop));
    let payload = catch_unwind(AssertUnwindSafe(|| scene.world.step(DT)))
        .expect_err("the listener and both callbacks panic");
    assert_eq!(message(&*payload), "contact listener panic");

    scene.world.set_contact_listener(None);
    for id in [scene.id, second] {
        scene
            .world
            .vehicle_mut(id)
            .unwrap()
            .set_tire_max_impulse_callback(|input| input.default_friction());
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
            TireFriction::default()
        });
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
            input.default_friction()
        }
    };
    let mut scene = WheeledVehicle::scene();
    let mut vehicle = scene.world.vehicle_mut(scene.id).unwrap();
    vehicle.set_tire_max_impulse_callback(holding(&token));
    assert_eq!(Arc::strong_count(&token), 2);
    run(&mut scene, 0, 10, |_| throttle());
    let mut vehicle = scene.world.vehicle_mut(scene.id).unwrap();
    vehicle.set_tire_max_impulse_callback(no_grip);
    assert_eq!(Arc::strong_count(&token), 1, "replace");

    vehicle.set_tire_max_impulse_callback(holding(&token));
    vehicle.clear_tire_max_impulse_callback();
    assert_eq!(Arc::strong_count(&token), 1, "clear");

    let mut bike = Motorcycle::scene();
    bike.world
        .vehicle_mut(bike.id)
        .unwrap()
        .set_tire_max_impulse_callback(holding(&token));
    run(&mut bike, 0, 10, |_| throttle());
    assert_eq!(Arc::strong_count(&token), 2);
    bike.world.remove_vehicle(bike.id).unwrap();
    assert_eq!(Arc::strong_count(&token), 1, "remove");

    scene
        .world
        .vehicle_mut(scene.id)
        .unwrap()
        .set_tire_max_impulse_callback(holding(&token));
    run(&mut scene, 10, 10, |_| throttle());
    drop(scene);
    assert_eq!(Arc::strong_count(&token), 1, "world drop");
}

/// A callback that counts its calls and checks the wheel index, with Jolt's default friction.
fn counting(
    wheels: u32,
) -> (
    Arc<AtomicUsize>,
    impl Fn(&TireMaxImpulseInput) -> TireFriction,
) {
    let calls = Arc::new(AtomicUsize::new(0));
    let count = Arc::clone(&calls);
    let callback = move |input: &TireMaxImpulseInput| {
        assert!(input.wheel < wheels);
        count.fetch_add(1, Ordering::Relaxed);
        input.default_friction()
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
            TireFriction::default()
        });
    world
        .vehicle_mut(free)
        .unwrap()
        .set_tire_max_impulse_callback(move |input| free_counter(input));
    world
        .vehicle_mut(bike)
        .unwrap()
        .set_tire_max_impulse_callback(move |input| bike_counter(input));
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
        .set_tire_max_impulse_callback(no_grip);
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
