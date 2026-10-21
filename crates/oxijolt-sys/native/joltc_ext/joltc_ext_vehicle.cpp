// Motorcycle target lean access and the vehicle step listener. Part of this fork's joltc additions
// (see joltc_ext.cpp).
// Licensed under the MIT License (MIT), like joltc.
//
// A JPH_VehicleConstraint is a reinterpret_cast of the most derived JPH::VehicleConstraint, as in
// joltc; a JPH_PhysicsStepListener is a reinterpret_cast of the JPH::PhysicsStepListener base, as
// joltc's AsPhysicsStepListener expects. Every joltc vehicle controller handle is a reinterpret_cast of a Jolt controller class that
// derives from JPH::VehicleController by single inheritance (joltc's DEF_MAP_DECL), so the
// generic handle reaches the virtual GetSettings.

#include <Jolt/Jolt.h>

#include <Jolt/Physics/PhysicsStepListener.h>
#include <Jolt/Physics/Vehicle/MotorcycleController.h>
#include <Jolt/Physics/Vehicle/VehicleConstraint.h>

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
