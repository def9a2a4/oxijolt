//! The drivetrain of a live vehicle: the configuration last applied, which Jolt does not save,
//! and how a change to it is checked and handed to the controller.

use oxijolt_sys::*;

use super::tracked::{ChassisMass, TrackSpeedBound};
use super::{
    MotorcycleSettings, TrackedVehicleSettings, VehicleCollisionTester,
    VehicleDifferentialSettings, VehicleEngineSettings, VehicleTrackSettings,
    VehicleTransmissionSettings, WheeledVehicleSettings,
};
use crate::{limits, TrackSide, VehicleError};

/// The drivetrain configuration of a live vehicle as last applied, which Jolt does not save.
#[derive(Clone)]
pub(crate) enum LiveDrivetrain {
    /// A wheeled vehicle's settings, or a motorcycle's wheeled part.
    Wheeled(WheeledVehicleSettings),
    /// A tracked vehicle's settings, how fast any configuration it had can have left its
    /// tracks turning (running maximum, see docs/limits.md#track-drive-envelope), and its
    /// chassis, which cannot change while the vehicle exists (`check_not_owned`).
    Tracked {
        settings: TrackedVehicleSettings,
        track_speed_bound: TrackSpeedBound,
        chassis: ChassisMass,
    },
}

impl LiveDrivetrain {
    /// The engine last applied.
    pub(crate) fn engine(&self) -> &VehicleEngineSettings {
        match self {
            Self::Wheeled(settings) => &settings.engine,
            Self::Tracked { settings, .. } => &settings.engine,
        }
    }

    /// The transmission last applied.
    pub(crate) fn transmission(&self) -> &VehicleTransmissionSettings {
        match self {
            Self::Wheeled(settings) => &settings.transmission,
            Self::Tracked { settings, .. } => &settings.transmission,
        }
    }

    /// This drivetrain with `engine`, validated as at creation with the vehicle's current
    /// collision `tester`.
    pub(crate) fn with_engine(
        &self,
        engine: VehicleEngineSettings,
        tester: VehicleCollisionTester,
        object_layer_count: u32,
    ) -> Result<Self, VehicleError> {
        let mut candidate = self.clone();
        match &mut candidate {
            Self::Wheeled(settings) => settings.engine = engine,
            Self::Tracked { settings, .. } => settings.engine = engine,
        }
        candidate.validated(tester, object_layer_count)
    }

    /// This drivetrain with `transmission`, validated as at creation with the vehicle's current
    /// collision `tester`. The gear counts and the clutch release time must stay as they are.
    pub(crate) fn with_transmission(
        &self,
        transmission: VehicleTransmissionSettings,
        tester: VehicleCollisionTester,
        object_layer_count: u32,
    ) -> Result<Self, VehicleError> {
        check_fixed_transmission(self.transmission(), &transmission)?;
        let mut candidate = self.clone();
        match &mut candidate {
            Self::Wheeled(settings) => settings.transmission = transmission,
            Self::Tracked { settings, .. } => settings.transmission = transmission,
        }
        candidate.validated(tester, object_layer_count)
    }

    /// This candidate with `tester`, checked whole; a tracked one's drive envelope starts from
    /// the speeds earlier configurations allowed, and its bound grows to the new speeds.
    fn validated(
        mut self,
        tester: VehicleCollisionTester,
        object_layer_count: u32,
    ) -> Result<Self, VehicleError> {
        match &mut self {
            Self::Wheeled(settings) => {
                settings.collision_tester = tester;
                settings.validate(object_layer_count)?;
            }
            Self::Tracked {
                settings,
                track_speed_bound,
                ..
            } => {
                settings.collision_tester = tester;
                settings.validate_from(object_layer_count, track_speed_bound)?;
                *track_speed_bound = settings.track_speed_bound(track_speed_bound);
            }
        }
        Ok(self)
    }

    /// Copies this drivetrain's engine into the vehicle's controller.
    ///
    /// # Safety
    /// `controller` is the live controller of a vehicle of this variant's kind (a motorcycle's
    /// for [`Wheeled`](Self::Wheeled) included), which nothing else uses during the call, and no
    /// step runs.
    pub(crate) unsafe fn apply_engine(&self, controller: *mut JPH_VehicleController) {
        self.engine().with_jph(|engine| match self {
            // SAFETY: the controller is a `WheeledVehicleController` (a `MotorcycleController`
            // derives from it with single inheritance), live and unshared (contract). `engine`
            // and its curve live for the call; the extension copies them.
            Self::Wheeled(_) => unsafe {
                JPH_WheeledVehicleController_SetEngine(controller.cast(), engine)
            },
            // SAFETY: as above, for a `TrackedVehicleController`.
            Self::Tracked { .. } => unsafe {
                JPH_TrackedVehicleController_SetEngine(controller.cast(), engine)
            },
        });
    }

    /// Copies this drivetrain's transmission into the vehicle's controller.
    ///
    /// # Safety
    /// As for [`apply_engine`](Self::apply_engine); the controller's gear counts and clutch
    /// release time are this transmission's ([`with_transmission`](Self::with_transmission)
    /// checked them against the transmission last applied).
    pub(crate) unsafe fn apply_transmission(&self, controller: *mut JPH_VehicleController) {
        let transmission = self.transmission().create();
        let copied = match self {
            // SAFETY: as in `apply_engine`; the guard owns the transmission settings for the
            // call, which the extension copies.
            Self::Wheeled(_) => unsafe {
                JPH_WheeledVehicleController_SetTransmission(
                    controller.cast(),
                    transmission.as_ptr(),
                )
            },
            // SAFETY: as above, for a `TrackedVehicleController`.
            Self::Tracked { .. } => unsafe {
                JPH_TrackedVehicleController_SetTransmission(
                    controller.cast(),
                    transmission.as_ptr(),
                )
            },
        };
        if !copied {
            unreachable!("the gear counts and clutch release time were checked");
        }
    }
}

impl LiveDrivetrain {
    /// The settings of a wheeled vehicle or a motorcycle.
    fn wheeled(&self) -> &WheeledVehicleSettings {
        match self {
            Self::Wheeled(settings) => settings,
            Self::Tracked { .. } => unreachable!("a wheeled controller has a wheeled drivetrain"),
        }
    }

    /// The tracked vehicle's settings.
    fn tracked(&self) -> &TrackedVehicleSettings {
        match self {
            Self::Tracked { settings, .. } => settings,
            Self::Wheeled(_) => unreachable!("a tracked controller has a tracked drivetrain"),
        }
    }

    /// The largest [wheel mass ratio](limits::MAX_WHEEL_MASS_RATIO) of a wheeled vehicle's or
    /// a motorcycle's wheels on `chassis` with the collision `tester`, as derived in
    /// [docs/limits.md#wheel-mass-ratio].
    ///
    /// [docs/limits.md#wheel-mass-ratio]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#wheel-mass-ratio
    pub(crate) fn largest_wheel_mass_ratio(
        &self,
        chassis: &ChassisMass,
        tester: &VehicleCollisionTester,
    ) -> f64 {
        self.wheeled()
            .wheels
            .iter()
            .map(|wheel| chassis.wheel_mass_ratio(wheel, tester.contact_radius(&wheel.base)))
            .fold(0.0, f64::max)
    }

    /// The differentials last applied.
    pub(crate) fn differentials(&self) -> &[VehicleDifferentialSettings] {
        &self.wheeled().differentials
    }

    /// The limited slip ratio between the differentials last applied.
    pub(crate) fn differential_limited_slip_ratio(&self) -> f32 {
        self.wheeled().differential_limited_slip_ratio
    }

    /// This wheeled drivetrain with `differentials` and `limited_slip_ratio`, validated as at
    /// creation with the vehicle's current collision `tester`.
    pub(crate) fn with_differentials(
        &self,
        differentials: Vec<VehicleDifferentialSettings>,
        limited_slip_ratio: f32,
        tester: VehicleCollisionTester,
        object_layer_count: u32,
    ) -> Result<Self, VehicleError> {
        let mut settings = self.wheeled().clone();
        settings.differentials = differentials;
        settings.differential_limited_slip_ratio = limited_slip_ratio;
        Self::Wheeled(settings).validated(tester, object_layer_count)
    }

    /// The track on `side` last applied.
    pub(crate) fn track(&self, side: TrackSide) -> &VehicleTrackSettings {
        let settings = self.tracked();
        match side {
            TrackSide::Left => &settings.left,
            TrackSide::Right => &settings.right,
        }
    }

    /// This tracked drivetrain with the tracks `left` and `right`, validated as at creation
    /// with the vehicle's current collision `tester`, its chassis and the track speeds earlier
    /// configurations allowed. Each track's wheels and driven wheel must stay as they are.
    pub(crate) fn with_tracks(
        &self,
        left: VehicleTrackSettings,
        right: VehicleTrackSettings,
        tester: VehicleCollisionTester,
        object_layer_count: u32,
    ) -> Result<Self, VehicleError> {
        let current = [self.track(TrackSide::Left), self.track(TrackSide::Right)];
        for (current, new) in current.into_iter().zip([&left, &right]) {
            if new.wheels != current.wheels || new.driven_wheel != current.driven_wheel {
                return Err(VehicleError::FixedAtCreation("track wheels"));
            }
        }
        let mut candidate = self.clone();
        if let Self::Tracked { settings, .. } = &mut candidate {
            settings.left = left;
            settings.right = right;
        }
        let candidate = candidate.validated(tester, object_layer_count)?;
        if let Self::Tracked {
            settings, chassis, ..
        } = &candidate
        {
            if !limits::is_track_mass_ratio(settings.largest_track_mass_ratio(chassis)) {
                return Err(VehicleError::InvalidValue(limits::TRACK_MASS_RATIO_RULE));
            }
        }
        Ok(candidate)
    }

    /// Copies this wheeled drivetrain's differentials and their limited slip ratio into the
    /// vehicle's controller.
    ///
    /// # Safety
    /// As for [`apply_engine`](Self::apply_engine), for a [`Wheeled`](Self::Wheeled) drivetrain.
    pub(crate) unsafe fn apply_differentials(&self, controller: *mut JPH_VehicleController) {
        let differentials: Vec<JPH_VehicleDifferentialSettings> = self
            .differentials()
            .iter()
            .map(|differential| differential.to_jph())
            .collect();
        let controller: *mut JPH_WheeledVehicleController = controller.cast();
        // SAFETY: the controller is a live `WheeledVehicleController` that nothing else uses
        // (contract; a `MotorcycleController` derives from it with single inheritance).
        // `differentials` holds as many validated entries as passed and lives for the call,
        // which copies them.
        unsafe {
            JPH_WheeledVehicleController_SetDifferentials(
                controller,
                differentials.as_ptr(),
                differentials.len() as u32,
            );
            JPH_WheeledVehicleController_SetDifferentialLimitedSlipRatio(
                controller,
                self.differential_limited_slip_ratio(),
            );
        }
    }

    /// Copies this tracked drivetrain's tracks into the vehicle's controller.
    ///
    /// # Safety
    /// As for [`apply_engine`](Self::apply_engine), for a [`Tracked`](Self::Tracked)
    /// drivetrain; the controller's track wheels and driven wheels are this drivetrain's
    /// ([`with_tracks`](Self::with_tracks) checked them against the tracks last applied).
    pub(crate) unsafe fn apply_tracks(&self, controller: *mut JPH_VehicleController) {
        self.tracked().for_each_jph_track(|side, track| {
            let jolt_side = match side {
                TrackSide::Left => JPH_TrackSide_Left,
                TrackSide::Right => JPH_TrackSide_Right,
            };
            // SAFETY: the controller is a live `TrackedVehicleController` that nothing else
            // uses (contract); `track` and its wheel list live for the call, which copies them.
            let copied = unsafe {
                JPH_TrackedVehicleController_SetTrack(controller.cast(), jolt_side, track)
            };
            if !copied {
                unreachable!("the track wheels were checked");
            }
        });
    }
}

/// Refuses a transmission whose gear counts or clutch release time differ from `current`'s:
/// Jolt's saved state holds the current gear, which indexes the ratios, and the clutch release
/// countdown, which it divides by the release time.
fn check_fixed_transmission(
    current: &VehicleTransmissionSettings,
    new: &VehicleTransmissionSettings,
) -> Result<(), VehicleError> {
    if new.gear_ratios.len() != current.gear_ratios.len()
        || new.reverse_gear_ratios.len() != current.reverse_gear_ratios.len()
    {
        return Err(VehicleError::FixedAtCreation("gear counts"));
    }
    if new.clutch_release_time != current.clutch_release_time {
        return Err(VehicleError::FixedAtCreation("clutch release time"));
    }
    Ok(())
}

impl WheeledVehicleSettings {
    /// The drivetrain a wheeled vehicle created from these validated settings starts with.
    pub(crate) fn live_drivetrain(&self) -> LiveDrivetrain {
        LiveDrivetrain::Wheeled(self.clone())
    }
}

impl MotorcycleSettings {
    /// The drivetrain a motorcycle created from these validated settings starts with: its
    /// wheeled part.
    pub(crate) fn live_drivetrain(&self) -> LiveDrivetrain {
        self.vehicle.live_drivetrain()
    }
}

impl TrackedVehicleSettings {
    /// The drivetrain a tracked vehicle created from these validated settings on `chassis`
    /// starts with, its tracks at rest.
    pub(crate) fn live_drivetrain(&self, chassis: ChassisMass) -> LiveDrivetrain {
        LiveDrivetrain::Tracked {
            settings: self.clone(),
            track_speed_bound: self.track_speed_bound(&TrackSpeedBound::AT_REST),
            chassis,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::limits::PrincipalMass;
    use crate::{ObjectLayer, Quat, TrackedWheelSettings, Vec3};

    const LAYERS: u32 = 2;

    /// A valid tank: three wheels per track, driven at the first, a ray tester on layer 1.
    fn tank() -> TrackedVehicleSettings {
        let track = |x: f32| {
            let wheels = [1.0, 0.0, -1.0]
                .map(|z| TrackedWheelSettings::new(Vec3::new(x, -0.3, z)))
                .to_vec();
            VehicleTrackSettings::new(wheels, 0)
        };
        TrackedVehicleSettings::new(track(1.0), track(-1.0), tester())
    }

    /// A 4000 kg chassis with the inertia of a 3.4 x 1 x 6.4 m box.
    fn chassis() -> ChassisMass {
        let mass = 4000.0;
        let inertia = |a: f64, b: f64| mass / 12.0 * (a * a + b * b);
        ChassisMass {
            mass: PrincipalMass {
                inverse_mass: 1.0 / mass,
                inverse_inertia: [inertia(1.0, 6.4), inertia(3.4, 6.4), inertia(3.4, 1.0)]
                    .map(|moment| 1.0 / moment),
            },
            body_to_principal: Quat::IDENTITY,
            center_of_mass: Vec3::ZERO,
        }
    }

    fn tester() -> VehicleCollisionTester {
        VehicleCollisionTester::ray(ObjectLayer::new(1))
    }

    fn bound(drivetrain: &LiveDrivetrain) -> TrackSpeedBound {
        match drivetrain {
            LiveDrivetrain::Tracked {
                track_speed_bound, ..
            } => *track_speed_bound,
            LiveDrivetrain::Wheeled(_) => panic!("not a tracked drivetrain"),
        }
    }

    #[test]
    fn live_changes_keep_the_running_track_speed_bound() {
        let engine = TrackedVehicleSettings::default_engine();
        let created = tank().live_drivetrain(chassis());
        let stronger = created
            .with_engine(engine.clone().max_torque(5000.0), tester(), LAYERS)
            .unwrap();
        assert!(bound(&stronger).speeds[0] > bound(&created).speeds[0]);
        // Back to the engine of creation: the tracks may still turn as fast as the stronger
        // engine took them.
        let back = stronger
            .with_engine(engine.clone(), tester(), LAYERS)
            .unwrap();
        assert_eq!(bound(&back), bound(&stronger));
        let again = back.with_engine(engine, tester(), LAYERS).unwrap();
        assert_eq!(bound(&again), bound(&back));
    }
}
