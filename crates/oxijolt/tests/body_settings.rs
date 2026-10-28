//! Body configuration fixed at creation: locked axes (allowed degrees of freedom) and where
//! they are refused, user data, and a provided mass and inertia.

mod common;

use common::constraint_kinds::create_every_kind;
use common::controls::*;
use common::ragdoll::{conj, humanoid_parts, mul, part_shapes, ragdoll_world, skeleton};
use common::vehicle::{add_car, car_settings, car_world, chassis_settings, chassis_shape};
use common::*;
use oxijolt::*;

const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);
const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);

#[test]
fn a_plane_2d_body_stays_in_its_plane() {
    let mut world = world(GRAVITY, 1);
    // A static slab tilted about a diagonal axis, so its normal leans along z as well as x.
    let slab = Shape::new_box(Vec3::new(5.0, 0.5, 5.0)).unwrap();
    let tilt = quat_about(Vec3::new(0.6, 0.0, 0.8), 0.4);
    world
        .create_body(
            &slab,
            &BodySettings::new_static()
                .position(RVec3::new(0.0, -1.0, 0.0))
                .rotation(tilt),
        )
        .unwrap();
    let cube = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_dynamic()
                .position(RVec3::new(0.5, 2.0, 0.0))
                .linear_velocity(Vec3::new(1.0, -2.0, 3.0))
                .angular_velocity(Vec3::new(2.0, -1.0, 0.5))
                .allowed_dofs(AllowedDofs::PLANE_2D),
        )
        .unwrap();
    let start = world.body(cube).unwrap().position();
    let mut turned = false;
    for _ in 0..120 {
        step(&mut world, 1);
        let body = world.body(cube).unwrap();
        let (p, q) = (body.position(), body.rotation());
        let (v, w) = (body.linear_velocity(), body.angular_velocity());
        assert_eq!(p.z, 0.0);
        assert_eq!((q.x, q.y), (0.0, 0.0));
        assert_eq!(v.z, 0.0);
        assert_eq!((w.x, w.y), (0.0, 0.0));
        turned |= q.z != 0.0;
    }
    let end = world.body(cube).unwrap().position();
    assert!(turned, "the cube never turned about z");
    assert!(end.x != start.x && end.y < start.y, "{end:?}");
}

#[test]
fn axes_stay_world_axes_for_a_rotated_body() {
    let mut world = world(Vec3::ZERO, 1);
    // The body's own z axis points along world -y.
    let start = quat_about(Vec3::new(1.0, 0.0, 0.0), std::f32::consts::FRAC_PI_2);
    let translations =
        AllowedDofs::TRANSLATION_X | AllowedDofs::TRANSLATION_Y | AllowedDofs::TRANSLATION_Z;
    let cube = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_dynamic()
                .rotation(start)
                .allowed_dofs(translations | AllowedDofs::ROTATION_Z),
        )
        .unwrap();
    world
        .body_mut(cube)
        .unwrap()
        .set_angular_velocity(Vec3::new(1.0, 1.0, 1.0))
        .unwrap();
    assert_eq!(
        world.body(cube).unwrap().angular_velocity(),
        Vec3::new(0.0, 0.0, 1.0)
    );
    step(&mut world, 30);
    // The turn since the start is about world z, not about the body's own z.
    let q = world.body(cube).unwrap().rotation();
    let turn = mul(q, conj(start));
    assert!(turn.x.abs() < 1.0e-6 && turn.y.abs() < 1.0e-6, "{turn:?}");
    assert!(turn.z.abs() > 0.2, "{turn:?}");
}

#[test]
fn allowed_dofs_read_back() {
    let mut world = world(Vec3::ZERO, 1);
    let plane = add_locked_cube(&mut world, RVec3::ZERO, AllowedDofs::PLANE_2D);
    let rail = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_kinematic()
                .position(RVec3::new(3.0, 0.0, 0.0))
                .allowed_dofs(AllowedDofs::TRANSLATION_Y),
        )
        .unwrap();
    let free = add_cube(&mut world, RVec3::new(6.0, 0.0, 0.0));
    let fixed = world
        .create_body(&cube_shape(), &BodySettings::new_static())
        .unwrap();
    let dofs = |id| world.body(id).unwrap().allowed_dofs();
    assert_eq!(dofs(plane), AllowedDofs::PLANE_2D);
    assert_eq!(dofs(rail), AllowedDofs::TRANSLATION_Y);
    assert_eq!(dofs(free), AllowedDofs::ALL);
    assert_eq!(dofs(fixed), AllowedDofs::ALL);
}

#[test]
fn a_static_body_that_cannot_move_refuses_restricted_dofs() {
    let mut world = world(Vec3::ZERO, 1);
    let refused = world.create_body(
        &cube_shape(),
        &BodySettings::new_static().allowed_dofs(AllowedDofs::PLANE_2D),
    );
    assert!(
        matches!(refused, Err(BodyError::InvalidValue(_))),
        "{refused:?}"
    );
    let without_translation = world.create_body(
        &cube_shape(),
        &BodySettings::new_dynamic().allowed_dofs(AllowedDofs::ROTATION_Y),
    );
    assert!(
        matches!(without_translation, Err(BodyError::InvalidValue(_))),
        "{without_translation:?}"
    );
    assert_eq!(world.body_count(), 0);
}

#[test]
fn bodies_with_restricted_dofs_are_refused_by_every_constraint_kind_vehicles_and_ragdolls() {
    let mut world = world(Vec3::ZERO, 1);
    let locked = add_locked_cube(&mut world, RVec3::ZERO, AllowedDofs::PLANE_2D);
    let free = add_cube(&mut world, RVec3::new(1.5, 0.0, 0.0));
    for (body1, body2) in [(locked, free), (free, locked)] {
        let results = create_every_kind(&mut world, body1, body2);
        assert_eq!(results.len(), 12);
        for result in results {
            assert_eq!(
                result,
                Err(ConstraintError::Body(BodyError::RestrictedDofs(locked)))
            );
        }
    }
    assert_eq!(world.constraint_count(), 0);
    // The same two bodies with all six degrees of freedom take every kind.
    let other = add_cube(&mut world, RVec3::new(0.0, 0.0, 3.0));
    let accepted = create_every_kind(&mut world, other, free);
    assert!(accepted.iter().all(Result::is_ok), "{accepted:?}");
    step(&mut world, 2);

    let (mut world, layers) = car_world(GRAVITY, 1);
    let chassis = world
        .create_body(
            &chassis_shape(),
            &chassis_settings(&layers, RVec3::new(0.0, 2.0, 0.0), Quat::IDENTITY)
                .allowed_dofs(AllowedDofs::PLANE_2D),
        )
        .unwrap();
    let settings = car_settings(VehicleCollisionTester::ray(layers.probe));
    assert_eq!(
        world.create_wheeled_vehicle(chassis, &settings),
        Err(VehicleError::Body(BodyError::RestrictedDofs(chassis)))
    );
    assert_eq!(world.vehicle_ids().count(), 0);
    add_car(
        &mut world,
        &layers,
        RVec3::new(5.0, 2.0, 0.0),
        Quat::IDENTITY,
    );

    let (_, layers) = ragdoll_world(1);
    let shapes = part_shapes();
    let mut parts = humanoid_parts(&shapes, layers.ragdoll);
    parts[1].body = parts[1].body.clone().allowed_dofs(AllowedDofs::PLANE_2D);
    assert!(matches!(
        RagdollSettings::new(&skeleton(), &parts),
        Err(RagdollError::InvalidValue(_))
    ));
}

#[test]
fn rotated_rebase_refuses_restricted_dofs_and_changes_nothing() {
    let mut world = world(Vec3::ZERO, 1);
    let free = add_cube(&mut world, RVec3::new(-3.0, 0.0, 0.0));
    let rail = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_dynamic()
                .linear_velocity(Vec3::new(1.0, 0.0, 0.0))
                .allowed_dofs(AllowedDofs::TRANSLATION_X),
        )
        .unwrap();
    step(&mut world, 1);
    let saved = world.save_state();
    let before = [motion(&world, free), motion(&world, rail)];
    let quarter_turn = quat_about(Z, std::f32::consts::FRAC_PI_2);
    assert_eq!(
        world.rebase(&[free, rail], quarter_turn, RVec3::ZERO),
        Err(BodyError::RestrictedDofs(rail))
    );
    assert_eq!([motion(&world, free), motion(&world, rail)], before);
    assert_eq!(world.gravity(), Vec3::ZERO);
    world.restore_state(&saved).unwrap();
}

#[test]
fn translation_rebase_keeps_restricted_dofs() {
    let mut world = world(Vec3::ZERO, 1);
    let rail = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_dynamic()
                .linear_velocity(Vec3::new(1.0, 0.0, 0.0))
                .allowed_dofs(AllowedDofs::TRANSLATION_X),
        )
        .unwrap();
    world
        .rebase(&[rail], Quat::IDENTITY, RVec3::new(10.0, 2.0, -3.0))
        .unwrap();
    let body = world.body(rail).unwrap();
    assert_eq!(body.position(), RVec3::new(10.0, 2.0, -3.0));
    assert_eq!(body.linear_velocity(), Vec3::new(1.0, 0.0, 0.0));
    assert_eq!(body.allowed_dofs(), AllowedDofs::TRANSLATION_X);
    step(&mut world, 10);
    let p = world.body(rail).unwrap().position();
    assert!(p.x > 10.1 && p.y == 2.0 && p.z == -3.0, "{p:?}");
}

#[test]
fn user_data_round_trips() {
    let mut world = world(GRAVITY, 1);
    let floor = add_floor(&mut world);
    let keys = [0, 7, u64::MAX];
    let bodies: Vec<BodyId> = keys
        .iter()
        .enumerate()
        .map(|(i, &key)| {
            let settings = BodySettings::new_dynamic()
                .position(RVec3::new(2.0 * i as Real, 2.0, 0.0))
                .user_data(key);
            world.create_body(&cube_shape(), &settings).unwrap()
        })
        .collect();
    let read = |world: &PhysicsWorld| -> Vec<u64> {
        bodies
            .iter()
            .map(|&id| world.body(id).unwrap().user_data())
            .collect()
    };
    assert_eq!(read(&world), keys);
    let saved = world.save_state();
    step(&mut world, 30);
    assert_eq!(read(&world), keys);
    world.restore_state(&saved).unwrap();
    assert_eq!(read(&world), keys);
    let mut all = vec![floor];
    all.extend(&bodies);
    world
        .rebase(
            &all,
            quat_about(Vec3::new(0.0, 1.0, 0.0), 0.5),
            RVec3::new(3.0, 0.0, 0.0),
        )
        .unwrap();
    assert_eq!(read(&world), keys);

    let capsule = Shape::new_capsule(0.5, 0.3).unwrap();
    let character = world
        .create_character(
            &CharacterSettings::new(&capsule)
                .user_data(42)
                .inner_body(Some(InnerBody {
                    shape: &capsule,
                    object_layer: ObjectLayer::MOVING,
                })),
            RVec3::new(-5.0, 1.0, 0.0),
            Quat::IDENTITY,
        )
        .unwrap();
    let inner = world.character(character).unwrap().inner_body().unwrap();
    assert_eq!(world.body(inner).unwrap().user_data(), 42);
}

#[test]
fn ragdoll_parts_refuse_user_data() {
    let (mut world, layers) = ragdoll_world(1);
    let shapes = part_shapes();
    let mut parts = humanoid_parts(&shapes, layers.ragdoll);
    parts[2].body = parts[2].body.clone().user_data(5);
    assert!(matches!(
        RagdollSettings::new(&skeleton(), &parts),
        Err(RagdollError::InvalidValue(_))
    ));
    let settings = common::ragdoll::humanoid_settings(layers.ragdoll);
    let ragdoll = world
        .create_ragdoll(&settings, None, Activation::Activate)
        .unwrap();
    for &part in world.ragdoll(ragdoll).unwrap().body_ids() {
        assert_eq!(world.body(part).unwrap().user_data(), 0);
    }
}

#[test]
fn changing_settings_after_creation_changes_no_body() {
    let mut world = world(GRAVITY, 1);
    let mut settings = BodySettings::new_static()
        .sensor(true)
        .user_data(11)
        .allowed_dofs(AllowedDofs::PLANE_2D)
        .allow_dynamic_or_kinematic(true);
    let first = world.create_body(&cube_shape(), &settings).unwrap();
    settings = settings
        .sensor(false)
        .user_data(12)
        .allowed_dofs(AllowedDofs::ALL)
        .allow_dynamic_or_kinematic(false)
        .position(RVec3::new(3.0, 0.0, 0.0));
    let second = world.create_body(&cube_shape(), &settings).unwrap();
    let config = |id| {
        let body = world.body(id).unwrap();
        (
            body.is_sensor(),
            body.user_data(),
            body.allowed_dofs(),
            body.can_be_kinematic_or_dynamic(),
        )
    };
    assert_eq!(config(first), (true, 11, AllowedDofs::PLANE_2D, true));
    assert_eq!(config(second), (false, 12, AllowedDofs::ALL, false));
}

#[test]
fn remove_body_guards_still_hold() {
    let (mut world, layers) = ragdoll_world(1);
    let ragdoll = world
        .create_ragdoll(
            &common::ragdoll::humanoid_settings(layers.ragdoll),
            None,
            Activation::Activate,
        )
        .unwrap();
    let part = world.ragdoll(ragdoll).unwrap().body_ids()[0];
    assert_eq!(
        world.remove_body(part),
        Err(BodyError::OwnedByRagdoll(part))
    );

    let capsule = Shape::new_capsule(0.7, 0.4).unwrap();
    let character = world
        .create_character(
            &CharacterSettings::new(&capsule).inner_body(Some(InnerBody {
                shape: &capsule,
                object_layer: layers.ragdoll,
            })),
            RVec3::new(5.0, 1.0, 0.0),
            Quat::IDENTITY,
        )
        .unwrap();
    let inner = world.character(character).unwrap().inner_body().unwrap();
    assert_eq!(
        world.remove_body(inner),
        Err(BodyError::OwnedByCharacter(inner))
    );

    let anchor = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_static().position(RVec3::new(-5.0, 3.0, 0.0)),
        )
        .unwrap();
    let cube = add_cube(&mut world, RVec3::new(-5.0, 1.0, 0.0));
    world
        .create_constraint(
            anchor,
            cube,
            &PointConstraintSettings::new(RVec3::new(-5.0, 2.0, 0.0)),
        )
        .unwrap();
    for body in [anchor, cube] {
        assert_eq!(
            world.remove_body(body),
            Err(BodyError::UsedByConstraint(body))
        );
    }

    let (mut world, layers) = car_world(GRAVITY, 1);
    let (chassis, _) = add_car(
        &mut world,
        &layers,
        RVec3::new(0.0, 2.0, 0.0),
        Quat::IDENTITY,
    );
    assert_eq!(
        world.remove_body(chassis),
        Err(BodyError::UsedByVehicle(chassis))
    );
}

/// A 2 x 0.5 x 1 m box.
fn plank() -> Shape {
    Shape::new_box(Vec3::new(1.0, 0.25, 0.5)).unwrap()
}

/// The angular velocity a body of `shape` made from `settings` has right after the angular
/// impulse `(1, 1, 1)` N·m·s, in a world without gravity.
fn spin_after_unit_impulse(shape: &Shape, settings: &BodySettings) -> Vec3 {
    let mut world = world(Vec3::ZERO, 1);
    let id = world.create_body(shape, settings).unwrap();
    world
        .body_mut(id)
        .unwrap()
        .add_angular_impulse(Vec3::new(1.0, 1.0, 1.0))
        .unwrap();
    world.body(id).unwrap().angular_velocity()
}

#[test]
fn provided_inertia_spins_the_body_as_given() {
    let provided = BodySettings::new_dynamic()
        .mass_and_inertia(3.0, Inertia::Principal(Vec3::new(1.0, 2.0, 4.0)));
    let w = spin_after_unit_impulse(&plank(), &provided);
    let distance = |a: Vec3| length(Vec3::new(a.x - 1.0, a.y - 0.5, a.z - 0.25));
    assert!(distance(w) <= 1.0e-6, "{w:?}");
    let scaled = spin_after_unit_impulse(&plank(), &BodySettings::new_dynamic().mass(3.0));
    assert!(distance(scaled) > 0.1, "{scaled:?}");
}

/// `Inertia::Tensor` of `R · diag(moments) · Rᵀ` for the rotation `q`, rounded to `f32` and
/// exactly symmetric.
fn rotated_tensor(moments: [f64; 3], q: Quat) -> Inertia {
    let [x, y, z, w] = [q.x, q.y, q.z, q.w].map(f64::from);
    let r = [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - w * z),
            2.0 * (x * z + w * y),
        ],
        [
            2.0 * (x * y + w * z),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - w * x),
        ],
        [
            2.0 * (x * z - w * y),
            2.0 * (y * z + w * x),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ];
    let entry = |i: usize, j: usize| {
        let (i, j) = (i.min(j), i.max(j));
        (0..3).map(|k| r[i][k] * moments[k] * r[j][k]).sum::<f64>() as f32
    };
    Inertia::Tensor(std::array::from_fn(|i| {
        std::array::from_fn(|j| entry(i, j))
    }))
}

/// A rotation that leaves no axis of the body a principal axis.
fn tilt() -> Quat {
    quat_about(Vec3::new(0.3, -0.5, 0.8), 0.7)
}

/// Checks that a body of `mass` and `inertia` is refused with a rule containing `phrase` and
/// that the world keeps its bodies.
fn assert_refused(world: &mut PhysicsWorld, mass: f32, inertia: Inertia, phrase: &str) {
    let before = world.body_count();
    let settings = BodySettings::new_dynamic().mass_and_inertia(mass, inertia);
    match world.create_body(&plank(), &settings) {
        Err(BodyError::InvalidValue(rule)) => {
            assert!(rule.contains(phrase), "{inertia:?}: {rule}")
        }
        other => panic!("{inertia:?} with {mass} kg: {other:?}"),
    }
    assert_eq!(world.body_count(), before);
}

const VALUES: &str = "finite and symmetric";
const FLOOR: &str = "at least limits::MIN_PROVIDED_INERTIA";
const CEILING: &str = "at most 6 * mass * limits::MAX_SHAPE_EXTENT^2";
const CONDITION: &str = "positive definite and well conditioned";
const MASS: &str = "mass must be between limits::MIN_MASS and limits::MAX_MASS";

/// `6 · mass · MAX_SHAPE_EXTENT²` in `f64`, the largest moment a body of `mass` may be given.
fn ceiling(mass: f32) -> f64 {
    6.0 * f64::from(mass) * f64::from(limits::MAX_SHAPE_EXTENT).powi(2)
}

/// The largest `f32` at most `value`.
fn f32_at_most(value: f64) -> f32 {
    let rounded = value as f32;
    if f64::from(rounded) > value {
        rounded.next_down()
    } else {
        rounded
    }
}

#[test]
fn invalid_provided_inertia_is_refused() {
    let mut world = world(Vec3::ZERO, 1);
    let principal = |x: f32, y: f32, z: f32| Inertia::Principal(Vec3::new(x, y, z));
    let mut tensor = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    assert_refused(&mut world, 1.0, principal(f32::NAN, 1.0, 1.0), VALUES);
    tensor[1][2] = f32::INFINITY;
    tensor[2][1] = f32::INFINITY;
    assert_refused(&mut world, 1.0, Inertia::Tensor(tensor), VALUES);
    tensor[1][2] = 0.1;
    tensor[2][1] = 0.0;
    assert_refused(&mut world, 1.0, Inertia::Tensor(tensor), VALUES);

    for moments in [
        (0.0, 1.0, 1.0),
        (1.0, -1.0, 1.0),
        (1.0, 1.0, 5.0e-7),
        (1.0e-30, 1.0, 1.0),
    ] {
        assert_refused(
            &mut world,
            1.0,
            principal(moments.0, moments.1, moments.2),
            FLOOR,
        );
    }
    let below_floor = [
        [5.0e-7, 1.0e-8, 0.0],
        [1.0e-8, 5.0e-7, 0.0],
        [0.0, 0.0, 5.0e-7],
    ];
    assert_refused(&mut world, 1.0, Inertia::Tensor(below_floor), FLOOR);

    let indefinite = [[1.0, 2.0, 0.0], [2.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    assert_refused(&mut world, 1.0, Inertia::Tensor(indefinite), CONDITION);
    let needle = rotated_tensor([1.0, 1.0, 1.0e-5], tilt());
    assert_refused(&mut world, 1.0, needle, CONDITION);

    let above = (ceiling(1.0) as f32).next_up();
    assert_refused(&mut world, 1.0, principal(1.0, above, 1.0), CEILING);
    // Every moment is below the ceiling, the Frobenius norm 1.01 times above it.
    let c = ceiling(1.0);
    let wide = rotated_tensor([0.7 * c, 0.7 * c, 0.2 * c], tilt());
    assert_refused(&mut world, 1.0, wide, CEILING);

    for mass in [
        0.0,
        limits::MIN_MASS.next_down(),
        limits::MAX_MASS.next_up(),
    ] {
        assert_refused(&mut world, mass, principal(1.0, 1.0, 1.0), MASS);
    }
}

#[test]
fn the_last_mass_setter_wins() {
    let provided = Inertia::Principal(Vec3::new(1.0, 2.0, 4.0));
    let mass_last = BodySettings::new_dynamic()
        .mass_and_inertia(3.0, provided)
        .mass(2.0);
    assert_eq!(mass_last, BodySettings::new_dynamic().mass(2.0));
    assert_eq!(
        spin_after_unit_impulse(&plank(), &mass_last),
        spin_after_unit_impulse(&plank(), &BodySettings::new_dynamic().mass(2.0))
    );

    let inertia_last = BodySettings::new_dynamic()
        .mass(2.0)
        .mass_and_inertia(3.0, provided);
    let only_inertia = BodySettings::new_dynamic().mass_and_inertia(3.0, provided);
    assert_eq!(inertia_last, only_inertia);
    assert_eq!(
        spin_after_unit_impulse(&plank(), &inertia_last),
        spin_after_unit_impulse(&plank(), &only_inertia)
    );
}

#[test]
fn set_shape_recomputes_provided_inertia_from_the_shape() {
    let shape = plank();
    let mut world = world(Vec3::ZERO, 1);
    let provided = BodySettings::new_dynamic()
        .mass_and_inertia(3.0, Inertia::Principal(Vec3::new(1.0, 2.0, 4.0)));
    let id = world.create_body(&shape, &provided).unwrap();
    let mut body = world.body_mut(id).unwrap();
    body.set_shape(&shape, Some(3.0), Activation::DontActivate)
        .unwrap();
    body.add_angular_impulse(Vec3::new(1.0, 1.0, 1.0)).unwrap();
    let after = world.body(id).unwrap().angular_velocity();
    let scaled = spin_after_unit_impulse(&shape, &BodySettings::new_dynamic().mass(3.0));
    assert_eq!(after, scaled);
    assert_ne!(after, spin_after_unit_impulse(&shape, &provided));
}

/// The smallest scale `s` in `[low, high]`, to about 1e-6 relative, for which a body of `mass`
/// with `inertia(s)` is accepted, given that `low` is refused and `high` accepted; returns the
/// accepted inertia and the refused one just below it.
fn boundary(
    world: &mut PhysicsWorld,
    mass: f32,
    inertia: impl Fn(f64) -> Inertia,
    (mut low, mut high): (f64, f64),
) -> (Inertia, Inertia) {
    let accepted = |world: &mut PhysicsWorld, s: f64| {
        let settings = BodySettings::new_dynamic().mass_and_inertia(mass, inertia(s));
        match world.create_body(&plank(), &settings) {
            Ok(id) => {
                world.remove_body(id).unwrap();
                true
            }
            Err(BodyError::InvalidValue(_)) => false,
            Err(error) => panic!("{error}"),
        }
    };
    assert!(!accepted(world, low) && accepted(world, high));
    while high - low > 1.0e-6 * high {
        let middle = 0.5 * (low + high);
        if accepted(world, middle) {
            high = middle;
        } else {
            low = middle;
        }
    }
    (inertia(high), inertia(low))
}

/// The largest value `x * 0.99^k` (for the first such `k`) that `accepts` takes.
fn largest_accepted(mut x: f32, mut accepts: impl FnMut(f32) -> bool) -> f32 {
    for _ in 0..10_000 {
        if accepts(x) {
            return x;
        }
        x *= 0.99;
    }
    panic!("nothing accepted down to {x}");
}

/// Drops a body of `mass` and `inertia` onto a static floor after the largest angular impulse
/// and torque it accepts, and checks 60 ticks of finite motion with a non-zero spin (an
/// overflowing angular velocity clamp gives an exact zero in a release build). The body never
/// sleeps, which would zero its spin too.
fn steps_finitely(mass: f32, inertia: Inertia) {
    let mut world = world(GRAVITY, 1);
    add_floor(&mut world);
    let settings = BodySettings::new_dynamic()
        .position(RVec3::new(0.0, 0.3, 0.0))
        .allow_sleeping(false)
        .mass_and_inertia(mass, inertia);
    let id = world.create_body(&plank(), &settings).unwrap();
    let along = |m: f32| Vec3::new(0.48 * m, 0.6 * m, 0.64 * m);
    let mut body = world.body_mut(id).unwrap();
    largest_accepted(1.0e18, |m| body.add_angular_impulse(along(m)).is_ok());
    largest_accepted(1.0e22, |m| body.add_torque(along(m)).is_ok());
    for tick in 0..60 {
        step(&mut world, 1);
        let body = world.body(id).unwrap();
        let (p, q) = (body.position(), body.rotation());
        let (v, w) = (body.linear_velocity(), body.angular_velocity());
        let finite = [p.x, p.y, p.z].iter().all(|c| c.is_finite())
            && [q.x, q.y, q.z, q.w, v.x, v.y, v.z, w.x, w.y, w.z]
                .iter()
                .all(|c| c.is_finite());
        assert!(
            finite,
            "{mass} kg, {inertia:?}, tick {tick}: {p:?} {q:?} {v:?} {w:?}"
        );
        assert_ne!(w, Vec3::ZERO, "{mass} kg, {inertia:?}, tick {tick}");
    }
}

/// Principal moments as a function of a scale.
type Moments = fn(f64) -> [f64; 3];

#[test]
fn provided_inertia_at_its_bounds_steps_finitely() {
    let mut world = world(Vec3::ZERO, 1);
    let floor = f64::from(limits::MIN_PROVIDED_INERTIA);
    let principal = |[x, y, z]: [f32; 3]| Inertia::Principal(Vec3::new(x, y, z));
    for mass in [limits::MIN_MASS, limits::MAX_MASS] {
        let top = ceiling(mass);
        let mut cases = vec![
            principal([limits::MIN_PROVIDED_INERTIA; 3]),
            principal([f32_at_most(top); 3]),
        ];
        for k in 0..3 {
            let mut moments = [1.0; 3];
            moments[k] = limits::MIN_PROVIDED_INERTIA;
            cases.push(principal(moments));
            let mut moments = [limits::MIN_PROVIDED_INERTIA; 3];
            moments[k] = f32_at_most(top);
            cases.push(principal(moments));
        }
        // Rotated tensors at the floor (repeated and close moments), at the ceiling, and at the
        // conditioning bound with one and with two small moments.
        let families: [(Moments, (f64, f64)); 5] = [
            (|s| [s, s, s], (0.5 * floor, 10.0 * floor)),
            (|s| [s, 1.1 * s, 1.3 * s], (0.5 * floor, 10.0 * floor)),
            (|s| [s, 0.9 * s, 0.6 * s], (top, 0.1 * top)),
            (|s| [1.0, 1.0, s], (1.0e-6, 1.0e-2)),
            (|s| [s, 1.0e-3, 1.0e-3], (10.0, 1.0e-2)),
        ];
        for (moments, range) in families {
            let rotated = |s: f64| rotated_tensor(moments(s), tilt());
            let (inside, outside) = if range.0 < range.1 {
                boundary(&mut world, mass, rotated, range)
            } else {
                // A ceiling: search the inverse scale.
                let inverse = |t: f64| rotated(1.0 / t);
                boundary(&mut world, mass, inverse, (1.0 / range.0, 1.0 / range.1))
            };
            let settings = BodySettings::new_dynamic().mass_and_inertia(mass, outside);
            assert!(world.create_body(&plank(), &settings).is_err());
            cases.push(inside);
        }
        for inertia in cases {
            steps_finitely(mass, inertia);
        }
    }
}
