// Jolt's single-threaded job system. Part of this fork's joltc additions (see joltc_ext.cpp).
// Licensed under the MIT License (MIT), like joltc.
//
// Handles: a JPH_JobSystem is a reinterpret_cast of the JPH::JobSystem base, as in joltc, so
// JPH_JobSystem_Destroy deletes it through the virtual destructor.

#include <Jolt/Jolt.h>

#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Physics/PhysicsSettings.h>

#include "joltc_ext.h"

JPH_JobSystem* JPH_JobSystemSingleThreaded_Create(uint32_t maxJobs)
{
	JPH::JobSystem* jobSystem = new JPH::JobSystemSingleThreaded(maxJobs > 0 ? maxJobs : JPH::cMaxPhysicsJobs);
	return reinterpret_cast<JPH_JobSystem*>(jobSystem);
}
