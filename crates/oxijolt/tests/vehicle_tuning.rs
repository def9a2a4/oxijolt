//! Live drivetrains: the engine, transmission and differentials of a car, a motorcycle and a
//! tank read back as set, change the drive from the next step on, keep the gear counts and the
//! clutch release time of creation, survive a restore, and refuse invalid values without
//! changing anything. The tank's tracks are in `tracked_tuning.rs`.

mod common;

use common::vehicle::{
    add_car, car_wheels, car_world, chassis_settings, chassis_shape, record_vehicle, CarLayers,
    GRAVITY, SUSPENSION_MIN, WHEEL_RADIUS, WHEEL_WIDTH,
};
use common::vehicle_kinds::*;
use common::*;
use oxijolt::*;

/// The car's forward gear ratios, Jolt's wheeled default.
const CAR_GEAR_RATIOS: [f32; 5] = [2.66, 1.78, 1.3, 1.0, 0.74];

/// The car's transmission: Jolt's wheeled default, shifting between 1100 and 1700 rpm so that
/// the car reaches third gear within a few seconds.
fn car_transmission() -> VehicleTransmissionSettings {
    VehicleTransmissionSettings::default()
        .shift_up_rpm(1700.0)
        .shift_down_rpm(1100.0)
}

/// A vehicle on flat ground under gravity, with its chassis.
struct Scene<K: VehicleKind> {
    world: PhysicsWorld,
    ground: BodyId,
    chassis: BodyId,
    id: VehicleId<K>,
}

/// The vehicle kinds these tests drive: how to build a scene and give full throttle.
trait Drive: VehicleKind + Sized {
    fn scene() -> Scene<Self>;
    fn throttle(world: &mut PhysicsWorld, id: VehicleId<Self>);
    /// The engine and the transmission the fixture is created with.
    fn drivetrain() -> (VehicleEngineSettings, VehicleTransmissionSettings);
}

/// A static box ground whose top face is the plane y = 0.
fn add_ground(world: &mut PhysicsWorld, layers: &CarLayers) -> BodyId {
    world
        .create_body(
            &Shape::new_box(Vec3::new(100.0, 1.0, 100.0)).unwrap(),
            &BodySettings::new_static()
                .position(RVec3::new(0.0, -1.0, 0.0))
                .object_layer(layers.ground),
        )
        .unwrap()
}

impl Drive for WheeledVehicle {
    /// The test car's body and wheels with four-wheel drive and [`car_transmission`], under its
    /// gravity override. The front-wheel drive of the shared fixture slips at any throttle that
    /// moves it briskly, and Jolt does not shift up while a driven wheel slips.
    fn scene() -> Scene<Self> {
        let (mut world, layers) = car_world(GRAVITY, 1);
        let ground = add_ground(&mut world, &layers);
        let body = chassis_settings(&layers, RVec3::new(0.0, 1.0, 0.0), Quat::IDENTITY);
        let chassis = world.create_body(&chassis_shape(), &body).unwrap();
        let half = |left, right| {
            VehicleDifferentialSettings::new(Some(left), Some(right)).engine_torque_ratio(0.5)
        };
        let settings = WheeledVehicleSettings::new(
            car_wheels(),
            vec![half(0, 1), half(2, 3)],
            VehicleCollisionTester::ray(layers.probe),
        )
        .transmission(car_transmission());
        let id = world.create_wheeled_vehicle(chassis, &settings).unwrap();
        world.vehicle_mut(id).unwrap().set_gravity(GRAVITY).unwrap();
        Scene {
            world,
            ground,
            chassis,
            id,
        }
    }

    fn throttle(world: &mut PhysicsWorld, id: VehicleId<Self>) {
        let input = DriverInput {
            forward: 0.6,
            ..DriverInput::default()
        };
        world
            .vehicle_mut(id)
            .unwrap()
            .set_driver_input(input)
            .unwrap();
    }

    fn drivetrain() -> (VehicleEngineSettings, VehicleTransmissionSettings) {
        (VehicleEngineSettings::default(), car_transmission())
    }
}

impl Drive for Motorcycle {
    fn scene() -> Scene<Self> {
        let (mut world, layers) = car_world(GRAVITY, 1);
        let ground = add_ground(&mut world, &layers);
        let (chassis, id) = add_bike(
            &mut world,
            &layers,
            RVec3::new(0.0, 1.0, 0.0),
            Quat::IDENTITY,
        );
        Scene {
            world,
            ground,
            chassis,
            id,
        }
    }

    fn throttle(world: &mut PhysicsWorld, id: VehicleId<Self>) {
        world
            .vehicle_mut(id)
            .unwrap()
            .set_driver_input(ride(0.6, 0.0))
            .unwrap();
    }

    fn drivetrain() -> (VehicleEngineSettings, VehicleTransmissionSettings) {
        (bike_engine(), bike_transmission())
    }
}

impl Drive for TrackedVehicle {
    fn scene() -> Scene<Self> {
        let (mut world, layers) = car_world(GRAVITY, 1);
        let ground = add_ground(&mut world, &layers);
        let (chassis, id) = add_tank(
            &mut world,
            &layers,
            RVec3::new(0.0, 1.5, 0.0),
            Quat::IDENTITY,
        );
        Scene {
            world,
            ground,
            chassis,
            id,
        }
    }

    fn throttle(world: &mut PhysicsWorld, id: VehicleId<Self>) {
        world
            .vehicle_mut(id)
            .unwrap()
            .set_driver_input(tracks(1.0, 1.0, 1.0))
            .unwrap();
    }

    fn drivetrain() -> (VehicleEngineSettings, VehicleTransmissionSettings) {
        (
            TrackedVehicleSettings::default_engine(),
            TrackedVehicleSettings::default_transmission(),
        )
    }
}

impl<K: Drive> Scene<K> {
    /// Full throttle, then one step.
    fn tick(&mut self) {
        K::throttle(&mut self.world, self.id);
        step(&mut self.world, 1);
    }

    fn ticks(&mut self, count: usize) {
        for _ in 0..count {
            self.tick();
        }
    }

    fn vehicle(&self) -> VehicleRef<'_, K> {
        self.world.vehicle(self.id).unwrap()
    }

    fn vehicle_mut(&mut self) -> VehicleMut<'_, K> {
        self.world.vehicle_mut(self.id).unwrap()
    }

    /// The vehicle and its chassis as exact bits.
    fn bits(&self) -> Vec<u8> {
        let mut digest = Vec::new();
        record_vehicle(&self.world, self.id, &mut digest);
        digest
    }

    /// Speed of the chassis along +Z, the vehicle's forward, m/s.
    fn forward_speed(&self) -> f32 {
        self.world.body(self.chassis).unwrap().linear_velocity().z
    }

    fn gear(&self) -> i32 {
        self.vehicle().current_gear()
    }

    /// Asserts that the chassis, every wheel and the engine hold finite values.
    #[track_caller]
    fn assert_finite(&self) {
        let body = self.world.body(self.chassis).unwrap();
        let position: [Real; 3] = body.position().into();
        assert!(
            position.iter().all(|value| value.is_finite()),
            "{position:?}"
        );
        let velocities: [[f32; 3]; 2] = [
            body.linear_velocity().into(),
            body.angular_velocity().into(),
        ];
        assert!(velocities
            .as_flattened()
            .iter()
            .all(|value| value.is_finite()));
        let vehicle = self.vehicle();
        assert!(vehicle.engine_rpm().is_finite());
        for wheel in vehicle.wheels() {
            assert!(wheel.angular_velocity.is_finite() && wheel.rotation_angle.is_finite());
        }
    }
}

/// `ratios`, each scaled by `factor`.
fn scaled(ratios: &[f32], factor: f32) -> Vec<f32> {
    ratios.iter().map(|ratio| ratio * factor).collect()
}

#[track_caller]
fn assert_invalid(result: Result<(), VehicleError>, rule: &str) {
    match result {
        Err(VehicleError::InvalidValue(what)) => {
            assert!(
                what.contains(rule),
                "refused for {what:?}, expected {rule:?}"
            )
        }
        other => panic!("expected a refusal for {rule:?}, got {other:?}"),
    }
}

fn reads_back_as_set<K: Drive>(engine: VehicleEngineSettings, gear_ratios: &[f32]) {
    let mut scene = K::scene();
    let (created_engine, created_transmission) = K::drivetrain();
    assert_eq!(scene.vehicle().engine_settings(), &created_engine);
    assert_eq!(
        scene.vehicle().transmission_settings(),
        &created_transmission
    );
    let transmission = created_transmission
        .clone()
        .gear_ratios(scaled(gear_ratios, 1.25))
        .switch_time(0.25);
    scene.vehicle_mut().set_engine(engine.clone()).unwrap();
    assert_eq!(scene.vehicle().engine_settings(), &engine);
    assert_eq!(
        scene.vehicle().transmission_settings(),
        &created_transmission
    );
    scene
        .vehicle_mut()
        .set_transmission(transmission.clone())
        .unwrap();
    assert_eq!(scene.vehicle().engine_settings(), &engine);
    assert_eq!(scene.vehicle().transmission_settings(), &transmission);
    scene.ticks(30);
    scene.assert_finite();
}

#[test]
fn drivetrain_reads_back_as_set() {
    reads_back_as_set::<WheeledVehicle>(
        VehicleEngineSettings::default()
            .max_torque(650.0)
            .inertia(0.6),
        &CAR_GEAR_RATIOS,
    );
    reads_back_as_set::<Motorcycle>(
        bike_engine().max_torque(120.0).angular_damping(0.3),
        &BIKE_GEAR_RATIOS,
    );
    reads_back_as_set::<TrackedVehicle>(
        TrackedVehicleSettings::default_engine()
            .max_torque(700.0)
            .normalized_torque(vec![(0.0, 1.0), (1.0, 0.9)]),
        &[4.0, 3.0, 2.0, 1.0],
    );
}

/// Twins driven at full throttle, one of which gets `change` at tick 10: their bits until then
/// and their forward speeds at tick 60, changed twin second.
fn twins<K: Drive>(change: impl Fn(&mut VehicleMut<'_, K>)) -> [(Vec<Vec<u8>>, f32); 2] {
    [false, true].map(|changed| {
        let mut scene = K::scene();
        let mut before = Vec::new();
        for _ in 0..10 {
            scene.tick();
            before.push(scene.bits());
        }
        if changed {
            change(&mut scene.vehicle_mut());
        }
        scene.ticks(50);
        scene.assert_finite();
        (before, scene.forward_speed())
    })
}

#[test]
fn more_engine_torque_accelerates_harder() {
    let [(car_before, car_speed), (stronger_before, stronger_speed)] =
        twins::<WheeledVehicle>(|car| {
            car.set_engine(VehicleEngineSettings::default().max_torque(1500.0))
                .unwrap()
        });
    assert!(
        car_before == stronger_before,
        "the twins differ before the change"
    );
    eprintln!("car at tick 60: {car_speed} m/s, stronger engine {stronger_speed} m/s");
    assert!(stronger_speed > car_speed + 0.1);

    let [(tank_before, tank_speed), (stronger_before, stronger_speed)] =
        twins::<TrackedVehicle>(|tank| {
            let engine = TrackedVehicleSettings::default_engine().max_torque(2000.0);
            tank.set_engine(engine).unwrap()
        });
    assert!(
        tank_before == stronger_before,
        "the twins differ before the change"
    );
    eprintln!("tank at tick 60: {tank_speed} m/s, stronger engine {stronger_speed} m/s");
    assert!(stronger_speed > tank_speed + 0.1);
}

#[test]
fn a_longer_gear_ratio_changes_the_drive() {
    let run = |factor: Option<f32>| {
        let mut scene = WheeledVehicle::scene();
        let mut before = Vec::new();
        for _ in 0..10 {
            scene.tick();
            before.push(scene.bits());
        }
        if let Some(factor) = factor {
            let transmission = car_transmission().gear_ratios(scaled(&CAR_GEAR_RATIOS, factor));
            scene.vehicle_mut().set_transmission(transmission).unwrap();
        }
        scene.ticks(30);
        scene.assert_finite();
        (before, scene.vehicle().engine_rpm(), scene.forward_speed())
    };
    let (before, rpm, speed) = run(None);
    let (longer_before, longer_rpm, longer_speed) = run(Some(1.5));
    assert!(
        before == longer_before,
        "the twins differ before the change"
    );
    eprintln!("rpm {rpm} vs {longer_rpm}, speed {speed} vs {longer_speed} m/s");
    assert_ne!(rpm.to_bits(), longer_rpm.to_bits());
    assert_ne!(speed.to_bits(), longer_speed.to_bits());
}

/// Drives `K`, created with the gear ratios `forward` and `reverse`, until it is in gear 3 or
/// above, saves, and checks that changed gear counts and a changed clutch release time are
/// refused, that a restore still steps finitely, and that a transmission with the same counts
/// and new ratios is accepted and steps finitely from the saved gear.
fn gear_counts_and_clutch_release_time_are_fixed<K: Drive>(forward: &[f32], reverse: &[f32]) {
    let mut scene = K::scene();
    let (_, created) = K::drivetrain();
    let mut ticks = 0;
    while scene.gear() < 3 && ticks < 600 {
        scene.tick();
        ticks += 1;
    }
    assert!(
        scene.gear() >= 3,
        "gear {} after {ticks} ticks",
        scene.gear()
    );
    let saved = scene.world.save_state();
    let other_reverse = if reverse.len() > 1 {
        reverse[1..].to_vec()
    } else {
        vec![reverse[0], 0.5 * reverse[0]]
    };
    let refusals = [
        // Two forward gears while the transmission is in a higher one.
        (
            created.clone().gear_ratios(forward[..2].to_vec()),
            "gear counts",
        ),
        (
            created.clone().reverse_gear_ratios(other_reverse),
            "gear counts",
        ),
        (
            created.clone().clutch_release_time(0.4),
            "clutch release time",
        ),
    ];
    for (transmission, what) in refusals {
        assert_eq!(
            scene.vehicle_mut().set_transmission(transmission),
            Err(VehicleError::FixedAtCreation(what))
        );
        assert_eq!(scene.vehicle().transmission_settings(), &created);
    }
    scene.world.restore_state(&saved).unwrap();
    scene.ticks(120);
    scene.assert_finite();

    let longer = created.clone().gear_ratios(scaled(forward, 1.2));
    scene
        .vehicle_mut()
        .set_transmission(longer.clone())
        .unwrap();
    assert_eq!(scene.vehicle().transmission_settings(), &longer);
    scene.world.restore_state(&saved).unwrap();
    assert!(scene.gear() >= 3);
    for _ in 0..120 {
        scene.tick();
        let gear = scene.gear();
        assert!((-(reverse.len() as i32)..=forward.len() as i32).contains(&gear));
    }
    scene.assert_finite();
}

#[test]
fn gear_counts_and_clutch_release_time_are_fixed_at_creation() {
    gear_counts_and_clutch_release_time_are_fixed::<WheeledVehicle>(&CAR_GEAR_RATIOS, &[-2.9]);
    gear_counts_and_clutch_release_time_are_fixed::<TrackedVehicle>(
        &[4.0, 3.0, 2.0, 1.0],
        &[-4.0, -3.0],
    );
}

#[test]
fn clutch_release_time_refusal_during_a_countdown() {
    let mut scene = WheeledVehicle::scene();
    // The first throttled step puts the car into first gear from neutral, which needs no switch
    // time: Jolt starts releasing the clutch over the 0.3 s release time at once. Five steps
    // later 0.3 − 5/60 s are left, a clutch friction of 5/18, strictly between 0 and 1.
    scene.tick();
    assert_eq!(scene.gear(), 1);
    scene.ticks(5);
    assert_eq!(scene.gear(), 1);
    let saved = scene.world.save_state();
    let created = car_transmission();
    for release_time in [0.0, 0.1, 0.6] {
        assert_eq!(
            scene
                .vehicle_mut()
                .set_transmission(created.clone().clutch_release_time(release_time)),
            Err(VehicleError::FixedAtCreation("clutch release time"))
        );
    }
    scene.ticks(10);
    scene.world.restore_state(&saved).unwrap();
    scene.ticks(120);
    scene.assert_finite();
    assert_eq!(scene.vehicle().transmission_settings(), &created);
}

#[test]
fn restore_keeps_live_drivetrain_settings() {
    let stronger = VehicleEngineSettings::default().max_torque(900.0);
    let run = |detour: bool| {
        let mut scene = WheeledVehicle::scene();
        scene.ticks(60);
        let saved = scene.world.save_state();
        scene.vehicle_mut().set_engine(stronger.clone()).unwrap();
        if detour {
            let swerve = DriverInput {
                forward: -0.5,
                right: 0.8,
                brake: 0.3,
                hand_brake: 0.0,
            };
            for _ in 0..30 {
                scene.vehicle_mut().set_driver_input(swerve).unwrap();
                step(&mut scene.world, 1);
            }
        }
        scene.world.restore_state(&saved).unwrap();
        assert_eq!(scene.vehicle().engine_settings(), &stronger);
        (0..60)
            .map(|_| {
                scene.tick();
                scene.bits()
            })
            .collect::<Vec<_>>()
    };
    assert!(
        run(true) == run(false),
        "the replay after a detour differs from the one without"
    );
}

#[test]
fn lower_rpm_limits_clamp_a_restored_rpm() {
    const NEW_MAX_RPM: f32 = 1300.0;
    let mut scene = WheeledVehicle::scene();
    let mut ticks = 0;
    while scene.vehicle().engine_rpm() <= 1600.0 && ticks < 600 {
        scene.tick();
        ticks += 1;
    }
    let saved_rpm = scene.vehicle().engine_rpm();
    assert!(saved_rpm > 1600.0, "{saved_rpm} rpm after {ticks} ticks");
    let saved = scene.world.save_state();
    // The shift up rpm must stay below the max rpm, so it goes down first.
    let transmission = car_transmission()
        .shift_up_rpm(1250.0)
        .shift_down_rpm(1100.0);
    scene.vehicle_mut().set_transmission(transmission).unwrap();
    let engine = VehicleEngineSettings::default().max_rpm(NEW_MAX_RPM);
    scene.vehicle_mut().set_engine(engine).unwrap();
    scene.world.restore_state(&saved).unwrap();
    assert_eq!(scene.vehicle().engine_rpm(), saved_rpm);
    scene.tick();
    let rpm = scene.vehicle().engine_rpm();
    assert!(rpm <= NEW_MAX_RPM, "{rpm} rpm");
    scene.ticks(60);
    scene.assert_finite();
}

/// Steps an untouched twin and one whose `refused` change failed with `rule`: their getters
/// and their bits for 60 ticks are the same.
fn refused_change_changes_nothing<K: Drive>(
    rule: &str,
    refused: impl Fn(&mut VehicleMut<'_, K>) -> Result<(), VehicleError>,
) {
    let run = |refuse: bool| {
        let mut scene = K::scene();
        scene.ticks(20);
        if refuse {
            assert_invalid(refused(&mut scene.vehicle_mut()), rule);
        }
        let (engine, transmission) = K::drivetrain();
        assert_eq!(scene.vehicle().engine_settings(), &engine);
        assert_eq!(scene.vehicle().transmission_settings(), &transmission);
        (0..60)
            .map(|_| {
                scene.tick();
                scene.bits()
            })
            .collect::<Vec<_>>()
    };
    assert!(
        run(false) == run(true),
        "a refused change changed the drive"
    );
}

#[test]
fn invalid_live_drivetrains_change_nothing() {
    // Cross-part: the transmission shifts up above the new max rpm.
    refused_change_changes_nothing::<WheeledVehicle>("below the engine's max rpm", |car| {
        car.set_engine(VehicleEngineSettings::default().max_rpm(1600.0))
    });
    refused_change_changes_nothing::<WheeledVehicle>("below the engine's max rpm", |car| {
        car.set_transmission(car_transmission().shift_up_rpm(7000.0))
    });
    // A subnormal engine inertia makes the engine's step coefficient infinite.
    refused_change_changes_nothing::<Motorcycle>(
        "engine inertia and torque give a non-finite step coefficient",
        |bike| bike.set_engine(bike_engine().inertia(1.0e-40)),
    );
    // The track drive envelope refuses a tank engine far beyond its tracks.
    refused_change_changes_nothing::<TrackedVehicle>("track drive envelope", |tank| {
        tank.set_engine(TrackedVehicleSettings::default_engine().max_torque(1.0e25))
    });
}

#[test]
fn drivetrain_setters_use_the_current_collision_tester() {
    let mut scene = WheeledVehicle::scene();
    scene.ticks(10);
    let layer = scene.vehicle().collision_tester().object_layer();
    let sphere = VehicleCollisionTester::cast_sphere(layer, 0.2);
    scene.vehicle_mut().set_collision_tester(sphere).unwrap();
    scene
        .vehicle_mut()
        .set_engine(VehicleEngineSettings::default().max_torque(600.0))
        .unwrap();
    assert_eq!(scene.vehicle().collision_tester(), sphere);

    let (half_sin, half_cos) = (0.5 * 45.0_f32.to_radians()).sin_cos();
    let eighth_turn = Quat::from_xyzw(0.0, 0.0, half_sin, half_cos);
    scene
        .world
        .rebase(&[scene.ground, scene.chassis], eighth_turn, RVec3::ZERO)
        .unwrap();
    let rebased = scene.vehicle().collision_tester();
    assert_ne!(rebased, sphere);
    let transmission = car_transmission().clutch_strength(12.0);
    scene.vehicle_mut().set_transmission(transmission).unwrap();
    assert_eq!(scene.vehicle().collision_tester(), rebased);
    step(&mut scene.world, 30);
    scene.assert_finite();
}

/// The angular velocities of the vehicle's wheels, rad/s, in wheel order.
fn wheel_spins<K: VehicleKind>(vehicle: &VehicleRef<'_, K>) -> Vec<f32> {
    vehicle
        .wheels()
        .iter()
        .map(|wheel| wheel.angular_velocity)
        .collect()
}

#[test]
fn moving_drive_to_the_rear_differential_changes_which_wheels_spin() {
    // The test car, front-wheel drive, floating without gravity or ground at full throttle.
    let (mut world, layers) = car_world(Vec3::ZERO, 1);
    let (_, car) = add_car(&mut world, &layers, RVec3::ZERO, Quat::IDENTITY);
    let full = DriverInput {
        forward: 1.0,
        ..DriverInput::default()
    };
    let drive = |world: &mut PhysicsWorld, ticks: usize| {
        for _ in 0..ticks {
            world
                .vehicle_mut(car)
                .unwrap()
                .set_driver_input(full)
                .unwrap();
            step(world, 1);
        }
    };
    drive(&mut world, 90);
    let front_driven = wheel_spins(&world.vehicle(car).unwrap());
    eprintln!("front-wheel drive: {front_driven:?} rad/s");
    assert!(front_driven[0] > 1.0 && front_driven[1] > 1.0);
    assert_eq!(&front_driven[2..], [0.0, 0.0]);

    let rear = vec![VehicleDifferentialSettings::new(Some(2), Some(3))];
    world
        .vehicle_mut(car)
        .unwrap()
        .set_differentials(rear.clone(), 2.0)
        .unwrap();
    let vehicle = world.vehicle(car).unwrap();
    assert_eq!(vehicle.differentials(), rear);
    assert_eq!(vehicle.differential_limited_slip_ratio(), 2.0);
    drive(&mut world, 90);
    let rear_driven = wheel_spins(&world.vehicle(car).unwrap());
    eprintln!("rear-wheel drive: {rear_driven:?} rad/s");
    assert!(rear_driven.iter().all(|spin| spin.is_finite()));
    assert!(rear_driven[2] > 1.0 && rear_driven[3] > 1.0);
    assert!(rear_driven[0] <= front_driven[0] && rear_driven[1] <= front_driven[1]);
}

/// A car on flat ground whose engine drives, through two differentials of half the torque
/// each, its rear pair and a pair mounted too high to reach the ground; the front pair rolls.
/// The differentials share the torque openly (limited slip ratio `f32::MAX`).
fn lifted_pair_car() -> Scene<WheeledVehicle> {
    let (mut world, layers) = car_world(GRAVITY, 1);
    let ground = add_ground(&mut world, &layers);
    let body = chassis_settings(&layers, RVec3::new(0.0, 1.0, 0.0), Quat::IDENTITY);
    let chassis = world.create_body(&chassis_shape(), &body).unwrap();
    let lifted = |x: f32| {
        WheelSettings::new(Vec3::new(x, 0.3, 0.0))
            .radius(WHEEL_RADIUS)
            .width(WHEEL_WIDTH)
            .suspension_min_length(SUSPENSION_MIN)
            .suspension_max_length(0.1)
    };
    let mut wheels = car_wheels();
    wheels.extend([lifted(0.9), lifted(-0.9)]);
    let half = |left, right| {
        VehicleDifferentialSettings::new(Some(left), Some(right)).engine_torque_ratio(0.5)
    };
    let settings = WheeledVehicleSettings::new(
        wheels,
        vec![half(4, 5), half(2, 3)],
        VehicleCollisionTester::ray(layers.probe),
    )
    .transmission(car_transmission())
    .differential_limited_slip_ratio(f32::MAX);
    let id = world.create_wheeled_vehicle(chassis, &settings).unwrap();
    world.vehicle_mut(id).unwrap().set_gravity(GRAVITY).unwrap();
    Scene {
        world,
        ground,
        chassis,
        id,
    }
}

#[test]
fn limited_slip_ratio_changes_torque_split() {
    let run = |limited_slip_ratio: Option<f32>| {
        let mut scene = lifted_pair_car();
        let mut before = Vec::new();
        for _ in 0..10 {
            scene.tick();
            before.push(scene.bits());
        }
        let wheels = scene.vehicle().wheels();
        assert!(wheels[4].contact.is_none() && wheels[2].contact.is_some());
        if let Some(ratio) = limited_slip_ratio {
            let differentials = scene.vehicle().differentials().to_vec();
            scene
                .vehicle_mut()
                .set_differentials(differentials, ratio)
                .unwrap();
        }
        scene.ticks(50);
        scene.assert_finite();
        let spins = wheel_spins(&scene.vehicle());
        (before, spins[4], spins[2], scene.forward_speed())
    };
    let (open_before, open_lifted, open_rear, open_speed) = run(None);
    let (tight_before, tight_lifted, tight_rear, tight_speed) = run(Some(1.05));
    assert!(
        open_before == tight_before,
        "the twins differ before the change"
    );
    eprintln!(
        "open: lifted {open_lifted}, rear {open_rear} rad/s, {open_speed} m/s; \
         limited 1.05: lifted {tight_lifted}, rear {tight_rear} rad/s, {tight_speed} m/s"
    );
    assert!(tight_lifted < open_lifted);
    assert!(tight_speed > open_speed);
}

#[test]
fn a_motorcycle_differential_change_changes_its_drive() {
    const BIKE_DIFFERENTIAL_RATIO: f32 = 1.93 * 40.0 / 16.0;
    let scene = Motorcycle::scene();
    let created =
        [VehicleDifferentialSettings::new(None, Some(1))
            .differential_ratio(BIKE_DIFFERENTIAL_RATIO)];
    assert_eq!(scene.vehicle().differentials(), created);
    assert_eq!(scene.vehicle().differential_limited_slip_ratio(), 1.4);

    // The rear wheel slips at this throttle. Four times the ratio turns it at the engine's
    // pace without slipping, and the bike falls behind.
    let shorter = vec![created[0].differential_ratio(4.0 * BIKE_DIFFERENTIAL_RATIO)];
    let [(before, speed), (changed_before, changed_speed)] = twins::<Motorcycle>(|bike| {
        bike.set_differentials(shorter.clone(), 1.4).unwrap();
    });
    assert!(
        before == changed_before,
        "the twins differ before the change"
    );
    eprintln!("motorcycle at tick 60: {speed} m/s, shorter differential {changed_speed} m/s");
    assert!(changed_speed < speed - 0.5);
}

#[test]
fn invalid_live_differentials_change_nothing() {
    let half = |left, right| {
        VehicleDifferentialSettings::new(Some(left), Some(right)).engine_torque_ratio(0.5)
    };
    let cases = [
        (
            "differential wheel index out of range",
            vec![half(0, 1), half(2, 4)],
            1.4,
        ),
        (
            "differential engine torque ratios must add up to 1",
            vec![half(0, 1), half(2, 3).engine_torque_ratio(0.6)],
            1.4,
        ),
        (
            "differential limited slip ratio must be finite and above 1",
            vec![half(0, 1), half(2, 3)],
            1.0,
        ),
        ("a vehicle needs at least one differential", vec![], 1.4),
        // 1e38 times the largest gear ratio times the clutch strength overflows `f32`.
        (
            "clutch strength, ratios and wheel inertia give a non-finite step coefficient",
            vec![half(0, 1), half(2, 3).differential_ratio(1.0e38)],
            1.4,
        ),
    ];
    for (rule, differentials, limited_slip_ratio) in cases {
        refused_change_changes_nothing::<WheeledVehicle>(rule, |car| {
            car.set_differentials(differentials.clone(), limited_slip_ratio)
        });
        let mut scene = WheeledVehicle::scene();
        let created = scene.vehicle().differentials().to_vec();
        let created_ratio = scene.vehicle().differential_limited_slip_ratio();
        assert_invalid(
            scene
                .vehicle_mut()
                .set_differentials(differentials, limited_slip_ratio),
            rule,
        );
        assert_eq!(scene.vehicle().differentials(), created);
        assert_eq!(
            scene.vehicle().differential_limited_slip_ratio(),
            created_ratio
        );
    }
}
