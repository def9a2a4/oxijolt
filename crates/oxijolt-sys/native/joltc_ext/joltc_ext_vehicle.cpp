// Motorcycle target lean access, the vehicle step listener, live drivetrain settings of wheeled and
// tracked controllers and the tire max impulse callback reset. Part of this fork's joltc additions
// (see joltc_ext.cpp).
// Licensed under the MIT License (MIT), like joltc.
//
// A JPH_VehicleConstraint is a reinterpret_cast of the most derived JPH::VehicleConstraint, as in
// joltc; a JPH_PhysicsStepListener is a reinterpret_cast of the JPH::PhysicsStepListener base, as
// joltc's AsPhysicsStepListener expects. Every joltc vehicle controller handle is a reinterpret_cast of a Jolt controller class that
// derives from JPH::VehicleController by single inheritance (joltc's DEF_MAP_DECL), so the
// generic handle reaches the virtual GetSettings, and a JPH_WheeledVehicleController handle of a
// motorcycle is its JPH::WheeledVehicleController base. JPH_LinearCurve and
// JPH_VehicleTransmissionSettings are reinterpret_casts of JPH::LinearCurve and
// JPH::VehicleTransmissionSettings (DEF_MAP_DECL).

#include <Jolt/Jolt.h>

#include <Jolt/Core/LinearCurve.h>
#include <Jolt/Physics/PhysicsStepListener.h>
#include <Jolt/Physics/Vehicle/MotorcycleController.h>
#include <Jolt/Physics/Vehicle/TrackedVehicleController.h>
#include <Jolt/Physics/Vehicle/VehicleConstraint.h>
#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>

#include "joltc_ext_internal.h"

namespace
{
	// Names Jolt's protected MotorcycleController::mTargetLean. Inside a class derived from
	// MotorcycleController, &Derived::member forms a pointer to the protected member of the base
	// ([class.protected]); the pointer applies to any MotorcycleController. Never instantiated.
	struct MotorcycleTargetLean final : JPH::MotorcycleController
	{
		using Member = JPH::Vec3 JPH::MotorcycleController::*;

		static Member Get()
		{
			return &MotorcycleTargetLean::mTargetLean;
		}
	};

	// The controller as a MotorcycleController, or null when it is of another kind. Jolt is built
	// without C++ RTTI; its own RTTI on the controller's settings tells the kind.
	const JPH::MotorcycleController* AsMotorcycle(const JPH_VehicleController* controller)
	{
		const JPH::VehicleController* jolt = reinterpret_cast<const JPH::VehicleController*>(controller);
		const JPH::Ref<JPH::VehicleControllerSettings> settings = jolt->GetSettings();
		if (!JPH::IsKindOf(settings.GetPtr(), JPH_RTTI(JPH::MotorcycleControllerSettings)))
		{
			return nullptr;
		}
		return static_cast<const JPH::MotorcycleController*>(jolt);
	}

	// Forwards to VehicleConstraint::OnStep and keeps its gravity override to one force per
	// collision step (see JPH_VehicleStepListener_Create).
	class VehicleStepListener final : public JPH::PhysicsStepListener
	{
	public:
		explicit VehicleStepListener(JPH::VehicleConstraint* vehicle) : mVehicle(vehicle)
		{
		}

		void OnStep(const JPH::PhysicsStepListenerContext& context) override
		{
			if (context.mIsFirstStep)
			{
				mOverrideAdded = false;
			}
			// The condition under which VehicleConstraint::OnStep adds the override force.
			JPH::Body* body = mVehicle->GetVehicleBody();
			const bool addsOverride = body->IsInBroadPhase() && mVehicle->IsGravityOverridden() && body->IsActive();
			if (addsOverride && mOverrideAdded)
			{
				// The accumulator still holds this update's override; OnStep adds no other force.
				const JPH::Vec3 force = body->GetAccumulatedForce();
				ForwardStep(context);
				body->ResetForce();
				body->AddForce(force);
				return;
			}
			ForwardStep(context);
			mOverrideAdded = mOverrideAdded || addsOverride;
		}

	private:
		// VehicleConstraint::OnStep is private there and public in the PhysicsStepListener base.
		void ForwardStep(const JPH::PhysicsStepListenerContext& context)
		{
			static_cast<JPH::PhysicsStepListener*>(mVehicle)->OnStep(context);
		}

		JPH::VehicleConstraint* mVehicle;
		// Whether OnStep added the override force in an earlier collision step of this update.
		bool mOverrideAdded = false;
	};

	JPH::WheeledVehicleController& AsJoltWheeled(JPH_WheeledVehicleController* controller)
	{
		return *reinterpret_cast<JPH::WheeledVehicleController*>(controller);
	}

	JPH::TrackedVehicleController& AsJoltTracked(JPH_TrackedVehicleController* controller)
	{
		return *reinterpret_cast<JPH::TrackedVehicleController*>(controller);
	}

	// Copies the engine configuration; mCurrentRPM stays, Jolt clamps it before use.
	void CopyEngine(JPH::VehicleEngine& engine, const JPH_VehicleEngineSettings& settings)
	{
		JPH_ASSERT(settings.normalizedTorque);

		engine.mMaxTorque = settings.maxTorque;
		engine.mMinRPM = settings.minRPM;
		engine.mMaxRPM = settings.maxRPM;
		engine.mInertia = settings.inertia;
		engine.mAngularDamping = settings.angularDamping;
		engine.mNormalizedTorque = *reinterpret_cast<const JPH::LinearCurve*>(settings.normalizedTorque);
	}

	// Copies the transmission configuration unless it changes what the transmission's state depends
	// on: the gear counts (GetCurrentRatio indexes the ratios with the current gear) and the clutch
	// release time (the clutch friction divides the countdown by it). Gear, clutch friction and the
	// timers stay.
	bool CopyTransmission(JPH::VehicleTransmission& transmission, const JPH::VehicleTransmissionSettings& settings)
	{
		if (settings.mGearRatios.size() != transmission.mGearRatios.size()
			|| settings.mReverseGearRatios.size() != transmission.mReverseGearRatios.size()
			|| settings.mClutchReleaseTime != transmission.mClutchReleaseTime)
		{
			return false;
		}
		static_cast<JPH::VehicleTransmissionSettings&>(transmission) = settings;
		return true;
	}
}

bool JPH_MotorcycleController_GetTargetLean(const JPH_VehicleController* controller, JPH_Vec3* result)
{
	JPH_ASSERT(controller);
	JPH_ASSERT(result);

	const JPH::MotorcycleController* motorcycle = AsMotorcycle(controller);
	if (motorcycle == nullptr)
	{
		return false;
	}
	const JPH::Vec3& lean = motorcycle->*MotorcycleTargetLean::Get();
	result->x = lean.GetX();
	result->y = lean.GetY();
	result->z = lean.GetZ();
	return true;
}

bool JPH_MotorcycleController_SetTargetLean(JPH_VehicleController* controller, const JPH_Vec3* value)
{
	JPH_ASSERT(controller);
	JPH_ASSERT(value);

	const JPH::MotorcycleController* motorcycle = AsMotorcycle(controller);
	if (motorcycle == nullptr)
	{
		return false;
	}
	// The caller passed a mutable controller; the const view was only for the kind check.
	const_cast<JPH::MotorcycleController*>(motorcycle)->*MotorcycleTargetLean::Get() = joltc_ext::ToVec3(*value);
	return true;
}

JPH_VehicleStepListener* JPH_VehicleStepListener_Create(JPH_VehicleConstraint* constraint)
{
	JPH_ASSERT(constraint);

	return reinterpret_cast<JPH_VehicleStepListener*>(new VehicleStepListener(reinterpret_cast<JPH::VehicleConstraint*>(constraint)));
}

void JPH_VehicleStepListener_Destroy(JPH_VehicleStepListener* listener)
{
	delete reinterpret_cast<VehicleStepListener*>(listener);
}

JPH_PhysicsStepListener* JPH_VehicleStepListener_AsPhysicsStepListener(JPH_VehicleStepListener* listener)
{
	JPH_ASSERT(listener);

	JPH::PhysicsStepListener* joltListener = static_cast<JPH::PhysicsStepListener*>(reinterpret_cast<VehicleStepListener*>(listener));
	return reinterpret_cast<JPH_PhysicsStepListener*>(joltListener);
}

void JPH_WheeledVehicleController_SetEngine(JPH_WheeledVehicleController* controller, const JPH_VehicleEngineSettings* settings)
{
	JPH_ASSERT(controller);
	JPH_ASSERT(settings);

	CopyEngine(AsJoltWheeled(controller).GetEngine(), *settings);
}

void JPH_TrackedVehicleController_SetEngine(JPH_TrackedVehicleController* controller, const JPH_VehicleEngineSettings* settings)
{
	JPH_ASSERT(controller);
	JPH_ASSERT(settings);

	CopyEngine(AsJoltTracked(controller).GetEngine(), *settings);
}

bool JPH_WheeledVehicleController_SetTransmission(JPH_WheeledVehicleController* controller, const JPH_VehicleTransmissionSettings* settings)
{
	JPH_ASSERT(controller);
	JPH_ASSERT(settings);

	return CopyTransmission(AsJoltWheeled(controller).GetTransmission(), *reinterpret_cast<const JPH::VehicleTransmissionSettings*>(settings));
}

bool JPH_TrackedVehicleController_SetTransmission(JPH_TrackedVehicleController* controller, const JPH_VehicleTransmissionSettings* settings)
{
	JPH_ASSERT(controller);
	JPH_ASSERT(settings);

	return CopyTransmission(AsJoltTracked(controller).GetTransmission(), *reinterpret_cast<const JPH::VehicleTransmissionSettings*>(settings));
}

void JPH_WheeledVehicleController_SetDifferentials(JPH_WheeledVehicleController* controller, const JPH_VehicleDifferentialSettings* values, uint32_t count)
{
	JPH_ASSERT(controller);
	JPH_ASSERT(count == 0 || values);

	JPH::WheeledVehicleController::Differentials& differentials = AsJoltWheeled(controller).GetDifferentials();
	differentials.resize(count);
	for (uint32_t i = 0; i < count; ++i)
	{
		JPH::VehicleDifferentialSettings& differential = differentials[i];
		differential.mLeftWheel = values[i].leftWheel;
		differential.mRightWheel = values[i].rightWheel;
		differential.mDifferentialRatio = values[i].differentialRatio;
		differential.mLeftRightSplit = values[i].leftRightSplit;
		differential.mLimitedSlipRatio = values[i].limitedSlipRatio;
		differential.mEngineTorqueRatio = values[i].engineTorqueRatio;
	}
}

void JPH_WheeledVehicleController_SetDifferentialLimitedSlipRatio(JPH_WheeledVehicleController* controller, float value)
{
	JPH_ASSERT(controller);

	AsJoltWheeled(controller).SetDifferentialLimitedSlipRatio(value);
}

bool JPH_TrackedVehicleController_SetTrack(JPH_TrackedVehicleController* controller, JPH_TrackSide side, const JPH_VehicleTrackSettings* track)
{
	JPH_ASSERT(controller);
	JPH_ASSERT(track);
	JPH_ASSERT(track->wheelsCount == 0 || track->wheels);

	const int index = static_cast<int>(side);
	if (index < 0 || index >= static_cast<int>(JPH::ETrackSide::Num))
	{
		return false;
	}
	JPH::VehicleTrack& target = AsJoltTracked(controller).GetTracks()[index];
	if (target.mDrivenWheel != track->drivenWheel || target.mWheels.size() != track->wheelsCount)
	{
		return false;
	}
	for (uint32_t i = 0; i < track->wheelsCount; ++i)
	{
		if (target.mWheels[i] != track->wheels[i])
		{
			return false;
		}
	}
	target.mInertia = track->inertia;
	target.mAngularDamping = track->angularDamping;
	target.mMaxBrakeTorque = track->maxBrakeTorque;
	target.mDifferentialRatio = track->differentialRatio;
	return true;
}

void JPH_WheeledVehicleController_ResetTireMaxImpulseCallback(JPH_WheeledVehicleController* controller)
{
	JPH_ASSERT(controller);

	// Jolt's default, WheeledVehicleController::mTireMaxImpulseCallback's initializer.
	AsJoltWheeled(controller).SetTireMaxImpulseCallback(
		[](JPH::uint, float& outLongitudinalImpulse, float& outLateralImpulse, float inSuspensionImpulse, float inLongitudinalFriction, float inLateralFriction, float, float, float)
		{
			outLongitudinalImpulse = inLongitudinalFriction * inSuspensionImpulse;
			outLateralImpulse = inLateralFriction * inSuspensionImpulse;
		});
}
