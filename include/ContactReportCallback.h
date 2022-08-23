#pragma once

#include "PxPhysicsAPI.h"

class ContactReportCallback : public physx::PxSimulationEventCallback
{
	void onConstraintBreak(physx::PxConstraintInfo* constraints, physx::PxU32 count);
	void onWake(physx::PxActor** actors, physx::PxU32 count);
	void onSleep(physx::PxActor** actors, physx::PxU32 count);
	void onTrigger(physx::PxTriggerPair* pairs, physx::PxU32 count);
	void onAdvance(const physx::PxRigidBody* const*, const physx::PxTransform*, const physx::PxU32);
	void onContact(const physx::PxContactPairHeader& pairHeader, const physx::PxContactPair* pairs, physx::PxU32 nbPairs);
};
