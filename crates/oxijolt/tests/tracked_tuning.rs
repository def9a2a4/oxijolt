//! Live tracks: a tank's track settings read back as set, change braking and driving from the
//! next step on, keep each track's wheels of creation, stay within the chassis' track mass
//! ratio, and step finitely after a change that follows fast driving and a restore.

mod common;

use common::vehicle::{car_world, record_vehicle, CarLayers, GRAVITY};
use common::vehicle_kinds::*;
use common::*;
use oxijolt::*;

/// A tank on flat ground under world gravity.
struct Tank {
    world: PhysicsWorld,
    chassis: BodyId,
    id: VehicleId<TrackedVehicle>,
}

impl Tank {
    /// The shared test tank, or the tank with `left` and `right` as its tracks.
    fn new(tracks: Option<[VehicleTrackSettings; 2]>) -> Self {
        let (mut world, layers) = car_world(GRAVITY, 1);
        add_ground(&mut world, &layers);
        let body = behaviour_chassis(&layers, 4000.0, RVec3::new(0.0, 1.5, 0.0), Quat::IDENTITY);
        let chassis = world.create_body(&tank_chassis_shape(), &body).unwrap();
        let [left, right] = tracks.unwrap_or_else(|| [tank_track(1.7), tank_track(-1.7)]);
        let settings =
            TrackedVehicleSettings::new(left, right, VehicleCollisionTester::ray(layers.probe));
        let id = world.create_tracked_vehicle(chassis, &settings).unwrap();
        Self { world, chassis, id }
    }

    fn drive(&mut self, input: TrackedDriverInput, ticks: usize) {
        self.vehicle_mut().set_driver_input(input).unwrap();
        step(&mut self.world, ticks);
    }

    fn vehicle(&self) -> VehicleRef<'_, TrackedVehicle> {
        self.world.vehicle(self.id).unwrap()
    }

    fn vehicle_mut(&mut self) -> VehicleMut<'_, TrackedVehicle> {
        self.world.vehicle_mut(self.id).unwrap()
    }

    /// Both tracks with `change` applied to the shared test tank's tracks.
    fn set_both(
        &mut self,
        change: impl Fn(VehicleTrackSettings) -> VehicleTrackSettings,
    ) -> Result<(), VehicleError> {
        self.vehicle_mut()
            .set_tracks(change(tank_track(1.7)), change(tank_track(-1.7)))
    }

    /// The tank, its chassis and its tracks as exact bits.
    fn bits(&self) -> Vec<u8> {
        let mut digest = Vec::new();
        record_vehicle(&self.world, self.id, &mut digest);
        for track in self.vehicle().tracks() {
            digest.extend(track.angular_velocity.to_bits().to_le_bytes());
        }
        digest
    }

    /// Speed of the chassis along +Z, the tank's forward, m/s.
    fn forward_speed(&self) -> f32 {
        self.world.body(self.chassis).unwrap().linear_velocity().z
    }

    /// Asserts that the chassis, the tracks, the wheels and the engine hold finite values and
    /// that no track stands exactly still, which an overflow in Jolt's step shows as.
    #[track_caller]
    fn assert_finite_and_moving(&self) {
        let body = self.world.body(self.chassis).unwrap();
        let position: [Real; 3] = body.position().into();
        assert!(position.iter().all(|value| value.is_finite()));
        let velocities: [[f32; 3]; 2] = [
            body.linear_velocity().into(),
            body.angular_velocity().into(),
        ];
        assert!(velocities.as_flattened().iter().all(|v| v.is_finite()));
        let vehicle = self.vehicle();
        assert!(vehicle.engine_rpm().is_finite());
        for track in vehicle.tracks() {
            assert!(
                track.angular_velocity.is_finite() && track.angular_velocity != 0.0,
                "{track:?}"
            );
        }
        for wheel in vehicle.wheels() {
            assert!(wheel.angular_velocity.is_finite() && wheel.rotation_angle.is_finite());
        }
    }
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

const FORWARD: TrackedDriverInput = TrackedDriverInput {
    forward: 1.0,
    left_ratio: 1.0,
    right_ratio: 1.0,
    brake: 0.0,
};

const BRAKE: TrackedDriverInput = TrackedDriverInput {
    forward: 0.0,
    left_ratio: 1.0,
    right_ratio: 1.0,
    brake: 1.0,
};

/// Twin tanks driven forward, each given its own tracks at tick 10 by `change`: their bits until
/// then, and the twins after `ticks` more ticks of `input`.
fn twins(
    changes: [&dyn Fn(VehicleTrackSettings) -> VehicleTrackSettings; 2],
    input: TrackedDriverInput,
    ticks: usize,
) -> [(Vec<Vec<u8>>, Tank); 2] {
    changes.map(|change| {
        let mut tank = Tank::new(None);
        let before = (0..10)
            .map(|_| {
                tank.drive(FORWARD, 1);
                tank.bits()
            })
            .collect();
        tank.set_both(change).unwrap();
        tank.drive(input, ticks);
        (before, tank)
    })
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

#[test]
fn track_settings_read_back_as_set() {
    let mut tank = Tank::new(None);
    assert_eq!(
        tank.vehicle().track_settings(TrackSide::Left),
        &tank_track(1.7)
    );
    assert_eq!(
        tank.vehicle().track_settings(TrackSide::Right),
        &tank_track(-1.7)
    );
    let left = tank_track(1.7)
        .inertia(10.8)
        .angular_damping(0.8)
        .max_brake_torque(20000.0)
        .differential_ratio(7.0);
    let right = tank_track(-1.7).inertia(9.0).differential_ratio(5.0);
    tank.vehicle_mut()
        .set_tracks(left.clone(), right.clone())
        .unwrap();
    assert_eq!(tank.vehicle().track_settings(TrackSide::Left), &left);
    assert_eq!(tank.vehicle().track_settings(TrackSide::Right), &right);
    tank.drive(FORWARD, 30);
    tank.assert_finite_and_moving();
}

#[test]
fn heavier_brakes_stop_the_tank_sooner() {
    let light = |track: VehicleTrackSettings| track.max_brake_torque(1000.0);
    let heavy = |track: VehicleTrackSettings| track.max_brake_torque(15000.0);
    let [(light_before, mut light), (heavy_before, mut heavy)] =
        twins([&light, &heavy], FORWARD, 110);
    assert!(
        light_before == heavy_before,
        "the twins differ before the change"
    );
    let moving = [light.forward_speed(), heavy.forward_speed()];
    light.drive(BRAKE, 30);
    heavy.drive(BRAKE, 30);
    let (light_speed, heavy_speed) = (light.forward_speed(), heavy.forward_speed());
    eprintln!("{moving:?} m/s, after 30 braked ticks: light {light_speed}, heavy {heavy_speed}");
    assert!(heavy_speed < light_speed - 0.5);
}

#[test]
fn a_higher_track_differential_ratio_changes_track_speed() {
    let created = |track: VehicleTrackSettings| track;
    let higher = |track: VehicleTrackSettings| track.differential_ratio(9.0);
    let [(before, created), (higher_before, higher)] = twins([&created, &higher], FORWARD, 50);
    assert!(
        before == higher_before,
        "the twins differ before the change"
    );
    let [left, _] = created.vehicle().tracks();
    let [higher_left, _] = higher.vehicle().tracks();
    eprintln!(
        "left track at tick 60: {} rad/s, differential ratio 9: {} rad/s",
        left.angular_velocity, higher_left.angular_velocity
    );
    assert_ne!(
        left.angular_velocity.to_bits(),
        higher_left.angular_velocity.to_bits()
    );
    higher.assert_finite_and_moving();
}

#[test]
fn more_track_damping_slows_the_tracks() {
    let created = |track: VehicleTrackSettings| track;
    let damped = |track: VehicleTrackSettings| track.angular_damping(20.0);
    let [(before, created), (damped_before, damped)] = twins([&created, &damped], FORWARD, 50);
    assert!(
        before == damped_before,
        "the twins differ before the change"
    );
    let [left, right] = created.vehicle().tracks();
    let [damped_left, damped_right] = damped.vehicle().tracks();
    eprintln!(
        "tracks at tick 60: {} and {} rad/s, damped {} and {} rad/s",
        left.angular_velocity,
        right.angular_velocity,
        damped_left.angular_velocity,
        damped_right.angular_velocity
    );
    assert!(damped_left.angular_velocity < left.angular_velocity);
    assert!(damped_right.angular_velocity < right.angular_velocity);
    assert!(damped.forward_speed() < created.forward_speed());
}

#[test]
fn track_wheels_are_fixed_at_creation() {
    let mut tank = Tank::new(None);
    tank.drive(FORWARD, 10);
    let mut moved_wheels = tank_track_wheels(1.7);
    moved_wheels[4] = moved_wheels[4].clone().radius(0.25);
    let refusals = [
        VehicleTrackSettings::new(tank_track_wheels(1.7), 0),
        VehicleTrackSettings::new(moved_wheels, TANK_DRIVEN_WHEEL),
        VehicleTrackSettings::new(tank_track_wheels(1.7)[1..].to_vec(), TANK_DRIVEN_WHEEL - 1),
    ];
    for left in refusals {
        assert_eq!(
            tank.vehicle_mut().set_tracks(left, tank_track(-1.7)),
            Err(VehicleError::FixedAtCreation("track wheels"))
        );
        assert_eq!(
            tank.vehicle_mut()
                .set_tracks(tank_track(1.7), tank_track(-1.7)),
            Ok(())
        );
    }
    assert_eq!(
        tank.vehicle_mut().set_tracks(
            tank_track(1.7),
            VehicleTrackSettings::new(tank_track_wheels(-1.7), 0)
        ),
        Err(VehicleError::FixedAtCreation("track wheels"))
    );
    assert_eq!(
        tank.vehicle().track_settings(TrackSide::Right),
        &tank_track(-1.7)
    );
    tank.drive(FORWARD, 30);
    tank.assert_finite_and_moving();
}

#[test]
fn track_inertia_bounded_by_the_chassis() {
    let mut tank = Tank::new(None);
    tank.drive(FORWARD, 10);
    // Well inside `MAX_TRACK_INERTIA` and the inertia ratio, but heavier than this 4000 kg
    // chassis can push: it takes tracks up to about 11.05 kg·m².
    assert_invalid(
        tank.set_both(|track| track.inertia(11.2)),
        "a track is too heavy for its chassis",
    );
    assert_invalid(
        tank.vehicle_mut()
            .set_tracks(tank_track(1.7), tank_track(-1.7).inertia(0.05)),
        "track inertia ratio must be at most limits::MAX_TRACK_INERTIA_RATIO",
    );
    assert_eq!(
        tank.vehicle().track_settings(TrackSide::Left),
        &tank_track(1.7)
    );
    assert_eq!(
        tank.vehicle().track_settings(TrackSide::Right),
        &tank_track(-1.7)
    );
    // Slightly lighter tracks are accepted.
    tank.set_both(|track| track.inertia(10.9)).unwrap();
    tank.drive(FORWARD, 30);
    tank.assert_finite_and_moving();
}

#[test]
fn live_tracks_after_fast_driving_step_finitely() {
    let light = |x: f32| tank_track(x).inertia(limits::MIN_TRACK_INERTIA);
    let mut tank = Tank::new(Some([light(1.7), light(-1.7)]));
    tank.drive(FORWARD, 120);
    let fast = tank.world.save_state();
    let pivot = TrackedDriverInput {
        left_ratio: -1.0,
        ..FORWARD
    };
    tank.drive(pivot, 60);
    let heavier = |x: f32| light(x).inertia(100.0 * limits::MIN_TRACK_INERTIA);
    tank.vehicle_mut()
        .set_tracks(heavier(1.7), heavier(-1.7))
        .unwrap();
    tank.world.restore_state(&fast).unwrap();
    for tick in 0..300 {
        let input = if tick % 60 < 30 { FORWARD } else { pivot };
        tank.drive(input, 1);
        tank.assert_finite_and_moving();
    }
}
