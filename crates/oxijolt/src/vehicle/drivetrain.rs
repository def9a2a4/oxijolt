//! Reading and changing the drivetrain of a live vehicle of any kind.

use super::{VehicleKind, VehicleMut, VehicleRef};
use crate::{VehicleEngineSettings, VehicleError, VehicleTransmissionSettings};

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
