use super::super::tests::{bits3, curve_points, jolt_vec};
use super::*;
use crate::world::ensure_initialized;
use crate::{ObjectLayer, SpringSettings};

const LAYERS: u32 = 2;

/// A valid tank: three wheels per track, each track driven at its first wheel, a ray tester on
/// layer 1.
fn tank() -> TrackedVehicleSettings {
    let track = |x: f32| {
        let wheels = [1.0, 0.0, -1.0]
            .map(|z| TrackedWheelSettings::new(Vec3::new(x, -0.3, z)))
            .to_vec();
        VehicleTrackSettings::new(wheels, 0)
    };
    TrackedVehicleSettings::new(
        track(1.0),
        track(-1.0),
        VehicleCollisionTester::ray(ObjectLayer::new(1)),
    )
}

fn with_left(
    edit: impl Fn(VehicleTrackSettings) -> VehicleTrackSettings,
) -> TrackedVehicleSettings {
    let mut settings = tank();
    settings.left = edit(settings.left.clone());
    settings
}

fn with_inertias(left: f32, right: f32) -> TrackedVehicleSettings {
    let mut settings = with_left(|t| t.inertia(left));
    settings.right = settings.right.clone().inertia(right);
    settings
}

fn with_wheel(
    edit: impl Fn(TrackedWheelSettings) -> TrackedWheelSettings,
) -> TrackedVehicleSettings {
    let mut settings = tank();
    settings.right.wheels[1] = edit(settings.right.wheels[1].clone());
    settings
}

#[track_caller]
fn assert_refused(settings: TrackedVehicleSettings, rule: &str) {
    match settings.validate(LAYERS) {
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
fn tracked_wheel_defaults_are_jolts() {
    assert!(ensure_initialized());
    let ours = TrackedWheelSettings::new(Vec3::ZERO);
    // SAFETY: Jolt is initialised; the guard owns the one reference joltc returns.
    let jolt = unsafe { Owned::from_raw(JPH_WheelSettingsTV_Create()) }.unwrap();
    let tv = jolt.as_ptr();
    let base: *mut JPH_WheelSettings = tv.cast();
    // SAFETY: the settings are live and only read; every output is a live local.
    unsafe {
        let direction = jolt_vec(|v| JPH_WheelSettings_GetSuspensionDirection(base, v));
        assert_eq!(bits3(direction), bits3(ours.base.suspension_direction));
        let axis = jolt_vec(|v| JPH_WheelSettings_GetSteeringAxis(base, v));
        assert_eq!(bits3(axis), bits3(ours.base.steering_axis));
        let up = jolt_vec(|v| JPH_WheelSettings_GetWheelUp(base, v));
        assert_eq!(bits3(up), bits3(ours.base.wheel_up));
        let forward = jolt_vec(|v| JPH_WheelSettings_GetWheelForward(base, v));
        assert_eq!(bits3(forward), bits3(ours.base.wheel_forward));
        assert!(!JPH_WheelSettings_GetEnableSuspensionForcePoint(base));
        assert_eq!(
            JPH_WheelSettings_GetSuspensionMinLength(base),
            ours.base.suspension_min_length
        );
        assert_eq!(
            JPH_WheelSettings_GetSuspensionMaxLength(base),
            ours.base.suspension_max_length
        );
        assert_eq!(
            JPH_WheelSettings_GetSuspensionPreloadLength(base),
            ours.base.suspension_preload_length
        );
        let mut spring = JPH_SpringSettings {
            mode: JPH_SpringMode_StiffnessAndDamping,
            frequencyOrStiffness: 0.0,
            damping: 0.0,
        };
        JPH_WheelSettings_GetSuspensionSpring(base, &mut spring);
        let expected = ours.base.suspension_spring.to_jph();
        assert_eq!(spring.mode, expected.mode);
        assert_eq!(spring.frequencyOrStiffness, expected.frequencyOrStiffness);
        assert_eq!(spring.damping, expected.damping);
        assert_eq!(JPH_WheelSettings_GetRadius(base), ours.base.radius);
        assert_eq!(JPH_WheelSettings_GetWidth(base), ours.base.width);
        assert_eq!(
            JPH_WheelSettingsTV_GetLongitudinalFriction(tv),
            ours.longitudinal_friction
        );
        assert_eq!(
            JPH_WheelSettingsTV_GetLateralFriction(tv),
            ours.lateral_friction
        );
    }
}

#[test]
fn track_defaults_are_jolts() {
    let ours = VehicleTrackSettings::new(Vec::new(), 0);
    // SAFETY: an all-zero `JPH_VehicleTrackSettings` is valid (integers, floats and a null
    // pointer); `_Init` fills it with Jolt's defaults and allocates nothing.
    let jolt = unsafe {
        let mut jolt: JPH_VehicleTrackSettings = std::mem::zeroed();
        JPH_VehicleTrackSettings_Init(&mut jolt);
        jolt
    };
    assert_eq!(jolt.drivenWheel, ours.driven_wheel);
    assert_eq!(jolt.inertia, ours.inertia);
    assert_eq!(jolt.angularDamping, ours.angular_damping);
    assert_eq!(jolt.maxBrakeTorque, ours.max_brake_torque);
    assert_eq!(jolt.differentialRatio, ours.differential_ratio);
}

#[test]
fn tracked_controller_defaults_are_jolts() {
    assert!(ensure_initialized());
    let engine = TrackedVehicleSettings::default_engine();
    let transmission = TrackedVehicleSettings::default_transmission();
    // SAFETY: Jolt is initialised; the guard owns the one reference joltc returns.
    let jolt = unsafe { Owned::from_raw(JPH_TrackedVehicleControllerSettings_Create()) }.unwrap();
    let ptr = jolt.as_ptr();
    // SAFETY: the settings are live and only read. `GetEngine` lends the settings' own torque
    // curve and `GetTransmission` the settings' own transmission; both are read while the guard
    // keeps the settings alive and never destroyed here.
    unsafe {
        let mut jolt_engine: JPH_VehicleEngineSettings = std::mem::zeroed();
        JPH_TrackedVehicleControllerSettings_GetEngine(ptr, &mut jolt_engine);
        assert_eq!(jolt_engine.maxTorque, engine.max_torque);
        assert_eq!(jolt_engine.minRPM, engine.min_rpm);
        assert_eq!(jolt_engine.maxRPM, engine.max_rpm);
        assert_eq!(jolt_engine.inertia, engine.inertia);
        assert_eq!(jolt_engine.angularDamping, engine.angular_damping);
        assert_eq!(
            curve_points(jolt_engine.normalizedTorque),
            engine.normalized_torque
        );

        let jolt_transmission = JPH_TrackedVehicleControllerSettings_GetTransmission(ptr);
        assert_eq!(
            JPH_VehicleTransmissionSettings_GetMode(jolt_transmission),
            JPH_TransmissionMode_Auto
        );
        let forward: Vec<f32> =
            (0..JPH_VehicleTransmissionSettings_GetGearRatioCount(jolt_transmission))
                .map(|i| JPH_VehicleTransmissionSettings_GetGearRatio(jolt_transmission, i))
                .collect();
        assert_eq!(forward, transmission.gear_ratios);
        let reverse: Vec<f32> =
            (0..JPH_VehicleTransmissionSettings_GetReverseGearRatioCount(jolt_transmission))
                .map(|i| JPH_VehicleTransmissionSettings_GetReverseGearRatio(jolt_transmission, i))
                .collect();
        assert_eq!(reverse, transmission.reverse_gear_ratios);
        assert_eq!(
            JPH_VehicleTransmissionSettings_GetSwitchTime(jolt_transmission),
            transmission.switch_time
        );
        assert_eq!(
            JPH_VehicleTransmissionSettings_GetClutchReleaseTime(jolt_transmission),
            transmission.clutch_release_time
        );
        assert_eq!(
            JPH_VehicleTransmissionSettings_GetSwitchLatency(jolt_transmission),
            transmission.switch_latency
        );
        assert_eq!(
            JPH_VehicleTransmissionSettings_GetShiftUpRPM(jolt_transmission),
            transmission.shift_up_rpm
        );
        assert_eq!(
            JPH_VehicleTransmissionSettings_GetShiftDownRPM(jolt_transmission),
            transmission.shift_down_rpm
        );
        assert_eq!(
            JPH_VehicleTransmissionSettings_GetClutchStrength(jolt_transmission),
            transmission.clutch_strength
        );
    }
}

#[test]
fn built_tracked_wheels_reach_jolt() {
    assert!(ensure_initialized());
    let wheel = TrackedWheelSettings::new(Vec3::new(1.0, -0.3, 0.5))
        .radius(0.4)
        .longitudinal_friction(3.0)
        .lateral_friction(1.5);
    let jolt = wheel.create();
    let tv = jolt.as_ptr();
    // SAFETY: the settings are live and only read; outputs are live locals.
    unsafe {
        let position = jolt_vec(|v| JPH_WheelSettings_GetPosition(tv.cast(), v));
        assert_eq!(bits3(position), bits3(wheel.base.position));
        assert_eq!(JPH_WheelSettings_GetRadius(tv.cast()), 0.4);
        assert_eq!(JPH_WheelSettingsTV_GetLongitudinalFriction(tv), 3.0);
        assert_eq!(JPH_WheelSettingsTV_GetLateralFriction(tv), 1.5);
    }
    let built = tank().build();
    assert_eq!(built.wheels.len(), 6);
    assert_eq!(built.tracks, Some([0..3, 3..6]));
    assert!(matches!(built.controller, ControllerGuard::Tracked(_)));
}

#[test]
fn tracked_settings_are_validated() {
    assert!(ensure_initialized());
    assert_eq!(tank().validate(LAYERS), Ok(()));

    assert_refused(
        with_left(|t| VehicleTrackSettings::new(Vec::new(), 0).inertia(t.inertia)),
        "at least one wheel",
    );
    assert_refused(
        with_left(|t| VehicleTrackSettings {
            driven_wheel: 3,
            ..t
        }),
        "driven wheel",
    );
    assert_refused(
        with_left(|t| VehicleTrackSettings {
            driven_wheel: u32::MAX,
            ..t
        }),
        "driven wheel",
    );
    let below_min = f32::from_bits(limits::MIN_TRACK_INERTIA.to_bits() - 1);
    let above_max = f32::from_bits(limits::MAX_TRACK_INERTIA.to_bits() + 1);
    for inertia in [0.0, -1.0, f32::NAN, f32::INFINITY, below_min, above_max] {
        assert_refused(with_left(|t| t.inertia(inertia)), "track inertia");
    }
    for inertia in [limits::MIN_TRACK_INERTIA, limits::MAX_TRACK_INERTIA] {
        assert_eq!(with_inertias(inertia, inertia).validate(LAYERS), Ok(()));
    }
    assert_refused(with_left(|t| t.angular_damping(-0.1)), "angular damping");
    assert_refused(
        with_left(|t| t.max_brake_torque(f32::NAN)),
        "max brake torque",
    );
    for ratio in [0.0, -6.0, f32::INFINITY] {
        assert_refused(
            with_left(|t| t.differential_ratio(ratio)),
            "differential ratio",
        );
    }
    for friction in [-0.1, limits::MAX_FRICTION * 1.001, f32::NAN] {
        assert_refused(
            with_wheel(|w| w.longitudinal_friction(friction)),
            "friction",
        );
        assert_refused(with_wheel(|w| w.lateral_friction(friction)), "friction");
    }
    assert_eq!(
        with_wheel(|w| w
            .longitudinal_friction(limits::MAX_FRICTION)
            .lateral_friction(0.0))
        .validate(LAYERS),
        Ok(())
    );
    assert_refused(tank().up(Vec3::new(0.0, 2.0, 0.0)), "unit");
    assert_refused(tank().forward(Vec3::new(0.0, 0.6, 0.8)), "perpendicular");
    assert_refused(tank().max_pitch_roll_angle(-0.1), "pitch roll");
    assert_refused(
        tank().engine(TrackedVehicleSettings::default_engine().max_rpm(3000.0)),
        "shift up rpm",
    );
    assert_refused(
        tank().transmission(TrackedVehicleSettings::default_transmission().gear_ratios(vec![])),
        "gear ratios",
    );
    let mut far_layer = tank();
    far_layer.collision_tester = VehicleCollisionTester::ray(ObjectLayer::new(LAYERS));
    assert_refused(far_layer, "object layer");
}

#[test]
fn tracked_wheels_share_the_wheel_rules() {
    assert_refused(with_wheel(|w| w.radius(0.0)), "wheel radius");
    assert_refused(with_wheel(|w| w.width(-0.1)), "wheel width");
    assert_refused(
        with_wheel(|w| w.suspension_direction(Vec3::new(0.0, -1.1, 0.0))),
        "wheel directions",
    );
    assert_refused(
        with_wheel(|w| w.wheel_up(Vec3::new(0.0, 0.0, 1.0))),
        "perpendicular",
    );
    assert_refused(with_wheel(|w| w.suspension_min_length(-0.1)), "min length");
    assert_refused(with_wheel(|w| w.suspension_max_length(0.2)), "max length");
    assert_refused(with_wheel(|w| w.suspension_preload_length(-0.1)), "preload");
    assert_refused(
        with_wheel(|w| w.suspension_force_point(Some(Vec3::new(f32::NAN, 0.0, 0.0)))),
        "force point",
    );
    assert_refused(
        with_wheel(|w| {
            w.suspension_spring(SpringSettings::FrequencyAndDamping {
                frequency: 0.0,
                damping: 0.5,
            })
        }),
        "suspension spring",
    );
    let extent = limits::MAX_SHAPE_EXTENT;
    assert_refused(
        with_wheel(|_| TrackedWheelSettings::new(Vec3::new(extent * 1.001, 0.0, 0.0))),
        "wheel position",
    );
    // The cylinder tester needs positive widths, as for wheeled vehicles.
    let mut settings = with_wheel(|w| w.width(0.0));
    assert_eq!(settings.validate(LAYERS), Ok(()));
    settings.collision_tester = VehicleCollisionTester::cast_cylinder(ObjectLayer::new(1));
    assert_refused(settings, "cylinder tester");
}

#[test]
fn track_wheel_radii_stay_within_max_ratio_of_the_driven_wheel() {
    const RULE: &str = "wheel radii";
    let radii = |driven: f32, other: f32| {
        let mut settings = tank();
        settings.left.wheels[0] = settings.left.wheels[0].clone().radius(driven);
        settings.left.wheels[2] = settings.left.wheels[2].clone().radius(other);
        settings
    };
    assert_eq!(radii(1.0, 1.0e-4).validate(LAYERS), Ok(()));
    assert_refused(radii(1.0, 0.99e-4), RULE);
    assert_eq!(radii(1.0e-3, 10.0).validate(LAYERS), Ok(()));
    assert_refused(radii(1.0e-3, 10.1), RULE);
    // The review's counter-example: a 1e-36 m wheel next to a 0.3 m driven wheel would turn at
    // 1200 rad/s · 0.3 / 1e-36, which overflows.
    assert_refused(radii(0.3, 1.0e-36), RULE);
}

#[test]
fn track_inertias_stay_within_max_track_inertia_ratio() {
    let ratio = limits::MAX_TRACK_INERTIA_RATIO;
    for lighter in [limits::MIN_TRACK_INERTIA, 10.0, 1.0e3] {
        let heavier = ratio * lighter;
        let above = f32::from_bits(heavier.to_bits() + 1);
        assert_eq!(with_inertias(lighter, heavier).validate(LAYERS), Ok(()));
        assert_eq!(with_inertias(heavier, lighter).validate(LAYERS), Ok(()));
        assert_refused(with_inertias(lighter, above), "inertia ratio");
        assert_refused(with_inertias(above, lighter), "inertia ratio");
    }
}

#[test]
fn tracked_drive_envelope_bounds_the_drivetrain() {
    const ENVELOPE: &str = "drive envelope";
    let engine = TrackedVehicleSettings::default_engine;
    let transmission = TrackedVehicleSettings::default_transmission;
    let torque = |value: f32| engine().max_torque(value);
    // Valid one by one: a huge max torque, heavy tracks, gear and differential ratios of 100.
    let mut counter_example = with_inertias(1.0e6, 1.0e6)
        .engine(torque(1.0e20).normalized_torque(vec![(0.0, limits::MAX_NORMALIZED_TORQUE)]))
        .transmission(transmission().gear_ratios(vec![100.0]));
    counter_example.left = counter_example.left.clone().differential_ratio(100.0);
    assert_refused(counter_example, ENVELOPE);

    // Each factor alone, a decade inside and a decade outside.
    assert_eq!(tank().engine(torque(1.0e18)).validate(LAYERS), Ok(()));
    assert_refused(tank().engine(torque(1.0e19)), ENVELOPE);
    let tenfold = vec![(0.0, limits::MAX_NORMALIZED_TORQUE)];
    assert_refused(
        tank().engine(torque(1.0e18).normalized_torque(tenfold)),
        ENVELOPE,
    );
    let gear = |ratio: f32| transmission().gear_ratios(vec![ratio, 1.0]);
    assert_eq!(
        tank()
            .engine(torque(1.0e17))
            .transmission(gear(100.0))
            .validate(LAYERS),
        Ok(())
    );
    assert_refused(
        tank().engine(torque(1.0e18)).transmission(gear(100.0)),
        ENVELOPE,
    );
    // A reverse gear counts as much as a forward one.
    assert_refused(
        tank()
            .engine(torque(1.0e18))
            .transmission(transmission().reverse_gear_ratios(vec![-100.0])),
        ENVELOPE,
    );
    // The speed limit grows as the smallest gear shrinks.
    assert_eq!(tank().transmission(gear(1.0e-13)).validate(LAYERS), Ok(()));
    assert_refused(tank().transmission(gear(1.0e-14)), ENVELOPE);
    assert_refused(
        tank().transmission(
            transmission()
                .gear_ratios(vec![1.0e-36])
                .reverse_gear_ratios(vec![-1.0e-36]),
        ),
        "underflows",
    );
    // Two light tracks of unequal inertia: the synchronisation bounds the torque.
    let light = |torque_value: f32| {
        let mut settings = tank().engine(torque(torque_value));
        settings.left = settings.left.clone().inertia(limits::MIN_TRACK_INERTIA);
        settings.right = settings
            .right
            .clone()
            .inertia(2.0 * limits::MIN_TRACK_INERTIA);
        settings
    };
    assert_eq!(light(1.0e11).validate(LAYERS), Ok(()));
    assert_refused(light(1.0e34), ENVELOPE);
    // The brake: its impulse over the inertia and its torque over the smallest wheel radius.
    assert_eq!(
        with_left(|t| t.max_brake_torque(1.0e29)).validate(LAYERS),
        Ok(())
    );
    assert_refused(with_left(|t| t.max_brake_torque(1.0e30)), ENVELOPE);
    let mut small_wheel = with_left(|t| t.max_brake_torque(1.0e25));
    small_wheel.left.wheels[2] = small_wheel.left.wheels[2].clone().radius(3.1e-5);
    assert_eq!(small_wheel.validate(LAYERS), Ok(()));
    small_wheel.left.max_brake_torque = 1.0e26;
    assert_refused(small_wheel, ENVELOPE);
    // The max rpm sets the speed limit too.
    assert_refused(tank().engine(engine().max_rpm(1.0e20)), ENVELOPE);
}

/// A 4000 kg chassis with principal inverse inertia `inverse_inertia`, its principal frame
/// turned from body space by `body_to_principal`, its centre of mass at `center`.
fn chassis(inverse_inertia: [f64; 3], body_to_principal: Quat, center: Vec3) -> ChassisMass {
    ChassisMass {
        mass: PrincipalMass {
            inverse_mass: 1.0 / 4000.0,
            inverse_inertia,
        },
        body_to_principal,
        center_of_mass: center,
    }
}

/// The largest `√((r × d)ᵀ M (r × d))` over 3600 directions `d` in the y/z plane, for the
/// symmetric `M` given by its rows.
fn sampled_plane_lever(m: [[f64; 3]; 3], r: [f64; 3]) -> f64 {
    (0..3600)
        .map(|k| {
            let (sin, cos) = (f64::from(k) * std::f64::consts::PI / 1800.0).sin_cos();
            let c = limits::cross(r, [0.0, sin, cos]);
            (0..3)
                .flat_map(|i| (0..3).map(move |j| (i, j)))
                .map(|(i, j)| c[i] * m[i][j] * c[j])
                .sum::<f64>()
                .sqrt()
        })
        .fold(0.0, f64::max)
}

#[test]
fn track_mass_ratio_takes_the_chassis_inertia_at_the_farthest_contact() {
    let (a, b, c): (f64, f64, f64) = (1.0 / 1500.0, 1.0 / 4000.0, 1.0 / 2000.0);
    let center = Vec3::new(0.3, -0.5, 0.0);
    // `tank()`'s wheels: x = ±1, y = −0.3, z = 1, 0, −1, up +Y and forward +Z, so every ground
    // normal gives a longitudinal direction in the y/z plane. Jolt's 10 kg·m² tracks on 0.3 m
    // wheels, every contact within 0.5 + √(0.3² + 0.05²) of the attachment, where a unit lever
    // weighs at most the largest inverse inertia.
    let reach = 0.5 + 0.3_f64.hypot(0.05);
    let per_metre = a.max(b).max(c).sqrt();
    let track_mass = 10.0 / (0.3_f64 * 0.3);
    for angle in [0.0_f64, 0.4, -0.4] {
        let (half_sin, half_cos) = (0.5 * angle).sin_cos();
        let to_principal = Quat::from_xyzw(0.0, 0.0, half_sin as f32, half_cos as f32);
        // The body-space inverse inertia Rᵀ D R, with R the turn by `angle` about z.
        let (sin, cos) = angle.sin_cos();
        let xx = a * cos * cos + b * sin * sin;
        let yy = a * sin * sin + b * cos * cos;
        let xy = sin * cos * (b - a);
        let body = [[xx, xy, 0.0], [xy, yy, 0.0], [0.0, 0.0, c]];
        let expected = [1.0_f64, -1.0]
            .into_iter()
            .flat_map(|x| [1.0_f64, 0.0, -1.0].map(|z| [x - 0.3, -0.3 + 0.5, z]))
            .map(|r| {
                let lever = sampled_plane_lever(body, r) + reach * per_metre;
                track_mass * (1.0 / 4000.0 + lever * lever)
            })
            .fold(0.0, f64::max);
        let ratio = tank().largest_track_mass_ratio(&chassis([a, b, c], to_principal, center));
        assert!(
            (ratio - expected).abs() <= 1e-5 * expected,
            "angle {angle}: {ratio} vs {expected}"
        );
    }
    // A suspension force point far below the contacts sets the lever.
    let mut settings = tank();
    settings.left.wheels[0] = settings.left.wheels[0]
        .clone()
        .suspension_force_point(Some(Vec3::new(1.0, -5.0, 0.0)));
    let principal = [[a, 0.0, 0.0], [0.0, b, 0.0], [0.0, 0.0, c]];
    let lever = sampled_plane_lever(principal, [0.7, -4.5, 0.0]);
    let expected = track_mass * (1.0 / 4000.0 + lever * lever);
    let ratio = settings.largest_track_mass_ratio(&chassis([a, b, c], Quat::IDENTITY, center));
    assert!(
        (ratio - expected).abs() <= 1e-5 * expected,
        "{ratio} vs {expected}"
    );
}

/// The largest lever term `√((p × d)ᵀ M (p × d))`, `M` the body-space inverse inertia given by
/// its rows, at points `p` (from the centre of mass) at both ends and the middle of `wheel`'s
/// suspension travel moved by `contact_radius` along 27 directions, and directions `d` of the
/// wheel's forward/up plane turned about its steering axis by 61 angles within its largest
/// steering angle: a lower bound of the lever the wheel mass ratio bounds.
fn sampled_steered_lever(
    m: [[f64; 3]; 3],
    wheel: &WheelSettings,
    center: [f64; 3],
    contact_radius: f64,
) -> f64 {
    let v = |v: Vec3| [v.x, v.y, v.z].map(f64::from);
    let base = &wheel.base;
    let (position, direction) = (v(base.position), v(base.suspension_direction));
    let axis = v(base.steering_axis);
    let length = f64::from(base.suspension_max_length);
    let steer = f64::from(wheel.max_steer_angle.abs());
    let offsets: Vec<[f64; 3]> = (0..27)
        .map(|k| [k % 3, k / 3 % 3, k / 9].map(|c| f64::from(c) - 1.0))
        .map(|o| {
            let n = (o[0] * o[0] + o[1] * o[1] + o[2] * o[2]).sqrt().max(1.0);
            o.map(|c| c * contact_radius / n)
        })
        .collect();
    let mut largest = 0.0_f64;
    for t in [0.0, 0.5, 1.0] {
        for offset in &offsets {
            let p: [f64; 3] = std::array::from_fn(|k| {
                position[k] + t * length * direction[k] + offset[k] - center[k]
            });
            for i in 0..=60 {
                let angle = steer * (f64::from(i) / 30.0 - 1.0);
                for j in 0..72 {
                    let (sin, cos) = (f64::from(j) * std::f64::consts::PI / 36.0).sin_cos();
                    let d = rotated([0.0, sin, cos], axis, angle);
                    let c = limits::cross(p, d);
                    let form: f64 = (0..3)
                        .flat_map(|i| (0..3).map(move |j| (i, j)))
                        .map(|(i, j)| c[i] * m[i][j] * c[j])
                        .sum();
                    largest = largest.max(form.sqrt());
                }
            }
        }
    }
    largest
}

#[test]
fn wheel_mass_ratio_covers_every_steered_direction_and_contact() {
    // A chassis whose yaw inertia is small against its pitch and roll inertia, its principal
    // frame turned 0.4 rad about z from body space.
    let (a, b, c): (f64, f64, f64) = (1.0 / 2000.0, 1.0 / 90.0, 1.0 / 2000.0);
    let center = Vec3::new(0.1, -0.2, 0.3);
    let angle: f64 = 0.4;
    let (half_sin, half_cos) = (0.5 * angle).sin_cos();
    let to_principal = Quat::from_xyzw(0.0, 0.0, half_sin as f32, half_cos as f32);
    let (sin, cos) = angle.sin_cos();
    let xx = a * cos * cos + b * sin * sin;
    let yy = a * sin * sin + b * cos * cos;
    let xy = sin * cos * (b - a);
    let body = [[xx, xy, 0.0], [xy, yy, 0.0], [0.0, 0.0, c]];
    let chassis = chassis([a, b, c], to_principal, center);
    let rake = 30.0_f32.to_radians();
    for steer in [
        0.0_f32,
        0.5,
        70.0_f32.to_radians(),
        0.5 * std::f32::consts::PI,
    ] {
        for contact_radius in [0.35, 0.8] {
            let wheel = WheelSettings::new(Vec3::new(0.5, -1.9, 3.0))
                .radius(0.35)
                .inertia(1.0)
                .max_steer_angle(steer)
                .suspension_direction(Vec3::new(0.0, -rake.cos(), rake.sin()))
                .steering_axis(Vec3::new(0.0, rake.cos(), -rake.sin()));
            let ratio = chassis.wheel_mass_ratio(&wheel, contact_radius);
            let center = [0.1, -0.2, 0.3];
            let lever = sampled_steered_lever(body, &wheel, center, contact_radius);
            let sampled = 1.0 / (0.35_f64 * 0.35) * (1.0 / 4000.0 + lever * lever);
            assert!(
                sampled <= ratio && ratio <= 1.25 * sampled,
                "steer {steer}, contact radius {contact_radius}: {ratio} vs sampled {sampled}"
            );
        }
    }
}

/// The track speed bound of validated `settings` from rest, written out from
/// docs/limits.md#track-drive-envelope: `Ω_i = β·min(I_i, I_j / r)`.
fn speeds_from_rest(settings: &TrackedVehicleSettings) -> [f64; 2] {
    let r = 1.0 / f64::from(limits::MAX_RATIO);
    let h = f64::from(PhysicsWorld::MAX_DELTA_TIME);
    let k = f64::from(ANGULAR_VELOCITY_TO_RPM);
    let gears = || {
        let transmission = &settings.transmission;
        transmission
            .gear_ratios
            .iter()
            .chain(&transmission.reverse_gear_ratios)
            .map(|ratio| f64::from(ratio.abs()))
    };
    let g_min = gears().fold(f64::INFINITY, f64::min);
    let g_max = gears().fold(0.0, f64::max);
    let torque = f64::from(settings.engine.largest_torque());
    let n = f64::from(settings.engine.max_rpm);
    let [l, rt] = [&settings.left, &settings.right];
    let (i_l, i_r) = (f64::from(l.inertia), f64::from(rt.inertia));
    let tau = |track: &VehicleTrackSettings, inertia: f64| {
        let d = f64::from(track.differential_ratio);
        1.001 * n / (g_min * d * r * k) + d * g_max * torque * h / inertia
    };
    let (tau_l, tau_r) = (tau(l, i_l), tau(rt, i_r));
    let beta = (tau_l / i_l + tau_r / i_r)
        .max(tau_l * (1.0 / i_l + 1.0 / (r * i_r)))
        .max(tau_r * (1.0 / i_r + 1.0 / (r * i_l)));
    [beta * i_l.min(i_r / r), beta * i_r.min(i_l / r)]
}

#[test]
fn track_speeds_from_rest_match_the_envelope_and_never_shrink() {
    let light = with_inertias(limits::MIN_TRACK_INERTIA, 2.0 * limits::MIN_TRACK_INERTIA);
    for settings in [tank(), with_inertias(1.0, 50.0), light] {
        let from_rest = settings.track_speed_bound(&TrackSpeedBound::AT_REST);
        let expected = speeds_from_rest(&settings);
        for side in 0..2 {
            let relative = (from_rest.speeds[side] - expected[side]).abs() / expected[side];
            assert!(relative <= 1e-12, "{from_rest:?} vs {expected:?}");
        }
        // Starting from the bound it left, the same configuration keeps it.
        assert_eq!(settings.track_speed_bound(&from_rest), from_rest);
        // From faster tracks the bound grows to cover them, and never drops below them.
        for factor in [0.5, 10.0, 1.0e6] {
            let reached = TrackSpeedBound {
                speeds: from_rest.speeds.map(|speed| factor * speed),
                weighted: factor * from_rest.weighted,
                inertias: from_rest.inertias,
            };
            let bound = settings.track_speed_bound(&reached);
            for side in 0..2 {
                assert!(bound.speeds[side] >= reached.speeds[side], "{bound:?}");
                assert!(bound.speeds[side] >= from_rest.speeds[side], "{bound:?}");
            }
            assert!(bound.weighted >= reached.weighted && bound.weighted >= from_rest.weighted);
        }
    }
}

#[test]
fn a_reached_speed_can_refuse_a_change_valid_from_rest() {
    let settings = tank();
    assert_eq!(settings.validate(LAYERS), Ok(()));
    let inertia = [settings.left.inertia, settings.right.inertia].map(f64::from);
    assert_eq!(inertia[0], inertia[1]);
    // The track speed at which the lock torque `Ω·I / MIN_DELTA_TIME` reaches the envelope's 1e30.
    let ceiling = 1.0e30 * f64::from(PhysicsWorld::MIN_DELTA_TIME) / inertia[0];
    let reached = |speeds: [f64; 2]| TrackSpeedBound {
        speeds,
        weighted: speeds[0] / inertia[0] + speeds[1] / inertia[1],
        inertias: inertia,
    };
    let refuses = |speeds: [f64; 2]| match settings.validate_from(LAYERS, &reached(speeds)) {
        Err(VehicleError::InvalidValue(what)) => what.contains("drive envelope"),
        Ok(()) => false,
        other => panic!("{other:?}"),
    };
    assert!(!refuses([1.0e-3 * ceiling, 0.0]));
    // One track left faster than the ceiling.
    assert!(refuses([2.0 * ceiling, 0.0]));
    // Both tracks below it, but the synchronisation can move both speeds onto one track.
    assert!(refuses([0.75 * ceiling, 0.75 * ceiling]));
}

/// The corners of the track speeds `bound` allows: each `|ω|` up to its speed, and the weighted
/// speed with the bound's inertias up to its weighted bound. Every allowed state lies in their
/// convex hull, so a linear bound that holds at the corners holds for all of them.
fn corners(bound: &TrackSpeedBound) -> Vec<[f64; 2]> {
    let [s0, s1] = bound.speeds;
    let [i0, i1] = bound.inertias;
    let w = bound.weighted;
    // The largest speed of one track with the other at `other`.
    let first = |other: f64| s0.min(((w - other / i1) * i0).max(0.0));
    let second = |other: f64| s1.min(((w - other / i0) * i1).max(0.0));
    vec![
        [0.0, 0.0],
        [first(0.0), 0.0],
        [0.0, second(0.0)],
        [first(second(0.0)), second(0.0)],
        [first(0.0), second(first(0.0))],
    ]
}

/// Whether `bound` covers `state`: each speed and the weighted speed with the bound's inertias.
fn covers(bound: &TrackSpeedBound, state: [f64; 2]) -> bool {
    let weighted = state[0] / bound.inertias[0] + state[1] / bound.inertias[1];
    (0..2).all(|side| state[side] <= bound.speeds[side])
        && weighted <= bound.weighted * (1.0 + 1e-12)
}

#[test]
fn rescaled_weighted_bounds_take_either_operand() {
    let reached = |speeds: [f64; 2], weighted: f64, inertias: [f64; 2]| TrackSpeedBound {
        speeds,
        weighted,
        inertias,
    };
    // Both tracks were fast, never at once: the weighted bound is the smaller operand.
    let both = reached([10.0, 10.0], 12.0, [1.0, 1.0]);
    // One track was much faster: the speeds give the smaller operand once the fast track gets
    // the heavier inertia.
    let one = reached([1.0, 100.0], 101.0, [1.0, 1.0]);
    let cases = [
        (both, [2.0, 0.5], 24.0),
        (both, [2.0, 2.0], 6.0),
        (one, [1.0, 100.0], 2.0),
        (one, [100.0, 1.0], 100.01),
        (one, [0.5, 0.25], 402.0),
    ];
    for (bound, inertia, expected) in cases {
        let rescaled = bound.weighted_with(inertia);
        assert!(
            (rescaled - expected).abs() <= 1e-12 * expected,
            "{bound:?} {inertia:?}"
        );
        let moved = TrackSpeedBound {
            weighted: rescaled,
            inertias: inertia,
            ..bound
        };
        for corner in corners(&bound) {
            assert!(covers(&moved, corner), "{corner:?} under {moved:?}");
        }
    }
}

#[test]
fn successive_unequal_inertia_changes_cover_every_earlier_state() {
    let tracks = [
        (10.0, 10.0),
        (1.0, 50.0),
        (50.0, 1.0),
        (2.0, 0.5),
        (50.0, 1.0),
    ];
    let mut bound = TrackSpeedBound::AT_REST;
    let mut earlier = Vec::new();
    for (step, (left, right)) in tracks.into_iter().enumerate() {
        let settings = with_inertias(left, right);
        assert_eq!(
            settings.validate_from(LAYERS, &bound),
            Ok(()),
            "step {step}"
        );
        // The tracks were driven fast in between: well above what the drivetrain reaches.
        let driven = settings.track_speed_bound(&bound);
        let fast = TrackSpeedBound {
            speeds: driven.speeds.map(|speed| 3.0 * speed),
            weighted: 2.0 * driven.weighted,
            ..driven
        };
        earlier.extend(corners(&fast));
        bound = settings.track_speed_bound(&fast);
        // A state restored from any earlier configuration stays inside the bound.
        for &state in &earlier {
            assert!(
                covers(&bound, state),
                "step {step}: {state:?} under {bound:?}"
            );
        }
        // The same configuration again keeps the bound.
        assert_eq!(settings.track_speed_bound(&bound), bound, "step {step}");
    }
}
