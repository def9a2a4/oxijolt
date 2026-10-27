//! Reading and changing the drivetrain of a live vehicle: engine and transmission of every
//! kind, differentials of wheeled vehicles and motorcycles, tracks of tracked vehicles.

use super::{TrackedVehicle, VehicleKind, VehicleMut, VehicleRef, WheeledControllerKind};
use crate::{
    TrackSide, VehicleDifferentialSettings, VehicleEngineSettings, VehicleError,
    VehicleTrackSettings, VehicleTransmissionSettings,
};

impl<K: VehicleKind> VehicleRef<'_, K> {
    /// The engine settings the vehicle was created with or last given by
    /// [`VehicleMut::set_engine`]. The engine's speed is [`engine_rpm`](Self::engine_rpm).
    pub fn engine_settings(&self) -> &VehicleEngineSettings {
        self.entry.drivetrain.engine()
    }

    /// The transmission settings the vehicle was created with or last given by
    /// [`VehicleMut::set_transmission`]. The gear it is in is
    /// [`current_gear`](Self::current_gear).
    pub fn transmission_settings(&self) -> &VehicleTransmissionSettings {
        self.entry.drivetrain.transmission()
    }
}

impl<K: VehicleKind> VehicleMut<'_, K> {
    /// Replaces the engine's settings (Jolt `GetEngine` of the vehicle's controller).
    ///
    /// The engine is validated as at creation, against the rest of this vehicle's drivetrain,
    /// its wheels and its current collision tester; for a tracked vehicle the
    /// [track drive envelope] also covers the track speeds every earlier configuration allowed.
    /// The next step uses the new engine. The engine's rpm is kept, and Jolt clamps it to the
    /// new rpm range before it uses it. The settings are not part of
    /// [`WorldState`](crate::WorldState):
    /// [`PhysicsWorld::restore_state`](crate::PhysicsWorld::restore_state) does not undo them.
    ///
    /// Fails with [`VehicleError::InvalidValue`] as at creation; nothing changes then.
    ///
    /// [track drive envelope]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#track-drive-envelope
    pub fn set_engine(&mut self, engine: VehicleEngineSettings) -> Result<(), VehicleError> {
        let candidate = self.entry.drivetrain.with_engine(
            engine,
            self.entry.collision_tester,
            self.object_layer_count,
        )?;
        let controller = self.controller();
        // SAFETY: the controller is this vehicle's, whose kind the drivetrain was created for;
        // the world is borrowed mutably through this view, so no step runs.
        unsafe { candidate.apply_engine(controller) };
        self.entry.drivetrain = candidate;
        Ok(())
    }

    /// Replaces the transmission's settings (Jolt `GetTransmission` of the vehicle's
    /// controller).
    ///
    /// The transmission is validated as at creation, against the rest of this vehicle's
    /// drivetrain, its wheels and its current collision tester; for a tracked vehicle the
    /// [track drive envelope] also covers the track speeds every earlier configuration allowed.
    /// The next step uses the new transmission. The current gear, the clutch state and the
    /// switch timers are kept. The settings are not part of [`WorldState`](crate::WorldState):
    /// [`PhysicsWorld::restore_state`](crate::PhysicsWorld::restore_state) does not undo them.
    ///
    /// Fails with [`VehicleError::FixedAtCreation`] when the number of forward or reverse gears
    /// or the clutch release time differs from the vehicle's: Jolt's state saves the current
    /// gear and the clutch release countdown, but not the ratios or the release time they rely
    /// on. Fails with [`VehicleError::InvalidValue`] as at creation. Nothing changes on failure.
    ///
    /// [track drive envelope]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#track-drive-envelope
    pub fn set_transmission(
        &mut self,
        transmission: VehicleTransmissionSettings,
    ) -> Result<(), VehicleError> {
        let candidate = self.entry.drivetrain.with_transmission(
            transmission,
            self.entry.collision_tester,
            self.object_layer_count,
        )?;
        let controller = self.controller();
        // SAFETY: as in `set_engine`; `with_transmission` kept the controller's gear counts and
        // clutch release time.
        unsafe { candidate.apply_transmission(controller) };
        self.entry.drivetrain = candidate;
        Ok(())
    }
}

impl<K: WheeledControllerKind> VehicleRef<'_, K> {
    /// The differentials the vehicle was created with or last given by
    /// [`VehicleMut::set_differentials`].
    pub fn differentials(&self) -> &[VehicleDifferentialSettings] {
        self.entry.drivetrain.differentials()
    }

    /// The limited slip ratio between the differentials, as created or last given by
    /// [`VehicleMut::set_differentials`].
    pub fn differential_limited_slip_ratio(&self) -> f32 {
        self.entry.drivetrain.differential_limited_slip_ratio()
    }
}

impl<K: WheeledControllerKind> VehicleMut<'_, K> {
    /// Replaces the differentials and the limited slip ratio between them (Jolt
    /// `GetDifferentials` and `SetDifferentialLimitedSlipRatio` of the vehicle's
    /// `WheeledVehicleController`). Both go together: the engine torque ratios must add up to 1
    /// over the whole list, and the step coefficients depend on every differential.
    ///
    /// The differentials are validated as at creation
    /// ([`WheeledVehicleSettings::differential_limited_slip_ratio`](crate::WheeledVehicleSettings::differential_limited_slip_ratio)
    /// states the ratio's rule), against this vehicle's wheels, engine, transmission and current
    /// collision tester. The next step uses them. Jolt keeps no state of its own in a
    /// differential, so the wheel and engine speeds carry over. The settings are not part of
    /// [`WorldState`](crate::WorldState):
    /// [`PhysicsWorld::restore_state`](crate::PhysicsWorld::restore_state) does not undo them.
    ///
    /// Fails with [`VehicleError::InvalidValue`] as at creation; nothing changes then.
    pub fn set_differentials(
        &mut self,
        differentials: Vec<VehicleDifferentialSettings>,
        limited_slip_ratio: f32,
    ) -> Result<(), VehicleError> {
        let candidate = self.entry.drivetrain.with_differentials(
            differentials,
            limited_slip_ratio,
            self.entry.collision_tester,
            self.object_layer_count,
        )?;
        let controller = self.controller();
        // SAFETY: the controller is this vehicle's, a `WheeledVehicleController` or a
        // `MotorcycleController` (`K: WheeledControllerKind`), whose drivetrain is wheeled; the
        // world is borrowed mutably through this view, so no step runs.
        unsafe { candidate.apply_differentials(controller) };
        self.entry.drivetrain = candidate;
        Ok(())
    }
}

impl VehicleRef<'_, TrackedVehicle> {
    /// The settings of the track on `side`, as created or last given by
    /// [`VehicleMut::set_tracks`]. The track's speed is [`track`](Self::track).
    pub fn track_settings(&self, side: TrackSide) -> &VehicleTrackSettings {
        self.entry.drivetrain.track(side)
    }
}

impl VehicleMut<'_, TrackedVehicle> {
    /// Replaces the settings of both tracks (Jolt `GetTracks` of the vehicle's
    /// `TrackedVehicleController`): inertia, angular damping, max brake torque and differential
    /// ratio. Both go together because the inertia ratio rule couples the tracks.
    ///
    /// The tracks are validated as at creation, against this vehicle's drivetrain, its current
    /// collision tester and its chassis ([`limits::MAX_TRACK_MASS_RATIO`](crate::limits::MAX_TRACK_MASS_RATIO));
    /// the [track drive envelope] also covers the track speeds every earlier configuration
    /// allowed. The next step uses them. The tracks' speeds are kept. The settings are not part
    /// of [`WorldState`](crate::WorldState):
    /// [`PhysicsWorld::restore_state`](crate::PhysicsWorld::restore_state) does not undo them.
    ///
    /// Fails with [`VehicleError::FixedAtCreation`] when a track's wheels or driven wheel differ
    /// from the vehicle's: Jolt reads them on every step, and the vehicle's wheels are fixed.
    /// Fails with [`VehicleError::InvalidValue`] as at creation. Nothing changes on failure.
    ///
    /// [track drive envelope]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#track-drive-envelope
    pub fn set_tracks(
        &mut self,
        left: VehicleTrackSettings,
        right: VehicleTrackSettings,
    ) -> Result<(), VehicleError> {
        let candidate = self.entry.drivetrain.with_tracks(
            left,
            right,
            self.entry.collision_tester,
            self.object_layer_count,
        )?;
        let controller = self.controller();
        // SAFETY: the controller is this vehicle's `TrackedVehicleController`, whose drivetrain
        // is tracked; the world is borrowed mutably through this view, so no step runs.
        // `with_tracks` kept the controller's track wheels and driven wheels.
        unsafe { candidate.apply_tracks(controller) };
        self.entry.drivetrain = candidate;
        Ok(())
    }
}
