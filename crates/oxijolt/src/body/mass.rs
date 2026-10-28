//! A body's mass override: a mass alone, or a mass with the inertia the caller provides.

use oxijolt_sys::*;

use crate::Vec3;

/// The inertia tensor a body is given with
/// [`BodySettings::mass_and_inertia`](crate::BodySettings::mass_and_inertia), in kg·m².
///
/// It is taken about the centre of mass the body's shape defines (the override does not move it)
/// and in the body's local axes. The values must be finite, the principal moments at least
/// [`limits::MIN_PROVIDED_INERTIA`] and at most `6 · mass · limits::MAX_SHAPE_EXTENT²`, and a
/// tensor that is not diagonal must be positive definite and well conditioned; the rules are
/// derived in [docs/limits.md#provided-inertia].
///
/// [`limits::MIN_PROVIDED_INERTIA`]: crate::limits::MIN_PROVIDED_INERTIA
/// [docs/limits.md#provided-inertia]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#provided-inertia
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Inertia {
    /// The principal moments about the body's x, y and z axes, which are then its principal
    /// axes.
    Principal(Vec3),
    /// The full tensor as Jolt's `MassProperties::mInertia` holds it: `tensor[i][j]` is the entry
    /// in row `i` and column `j`, a moment on the diagonal and the negated product of inertia
    /// `-∫ x_i x_j dm` off it. It must be symmetric.
    Tensor([[f32; 3]; 3]),
}

impl Inertia {
    /// The tensor in rows; a `Principal` inertia is its diagonal.
    pub(crate) fn tensor(self) -> [[f32; 3]; 3] {
        match self {
            Self::Principal(moments) => [
                [moments.x, 0.0, 0.0],
                [0.0, moments.y, 0.0],
                [0.0, 0.0, moments.z],
            ],
            Self::Tensor(tensor) => tensor,
        }
    }
}

/// How a body's mass properties are overridden (Jolt `EOverrideMassProperties`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum MassOverride {
    /// The mass in kg; the inertia is the shape's, scaled to it (`CalculateInertia`).
    Mass(f32),
    /// The mass in kg and the inertia, the shape's ignored (`MassAndInertiaProvided`).
    MassAndInertia { mass: f32, inertia: Inertia },
}

impl MassOverride {
    /// The overriding mass in kg.
    pub(crate) fn mass(self) -> f32 {
        match self {
            Self::Mass(mass) | Self::MassAndInertia { mass, .. } => mass,
        }
    }

    /// The provided inertia tensor in rows, `None` when the inertia comes from the shape.
    pub(crate) fn tensor(self) -> Option<[[f32; 3]; 3]> {
        match self {
            Self::Mass(_) => None,
            Self::MassAndInertia { inertia, .. } => Some(inertia.tensor()),
        }
    }
}

/// Jolt's mass properties for `mass` and the inertia `tensor` in rows: column `j` of the 4x4
/// matrix is column `j` of the tensor with a zero `w`, and the last column is `(0, 0, 0, 1)`.
pub(crate) fn to_jph(mass: f32, tensor: [[f32; 3]; 3]) -> JPH_MassProperties {
    let column = |j: usize| JPH_Vec4 {
        x: tensor[0][j],
        y: tensor[1][j],
        z: tensor[2][j],
        w: 0.0,
    };
    JPH_MassProperties {
        mass,
        inertia: JPH_Mat4 {
            column: [
                column(0),
                column(1),
                column(2),
                JPH_Vec4 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            ],
        },
    }
}
