#pragma once

#include "PxPhysicsAPI.h"

namespace MotionPlanner
{
	/// @brief Simulation event callback to report contacts.
	class ContactReportCallback : public physx::PxSimulationEventCallback
	{
		/// @brief Called when contraint is broken.
		/// @param constraints Constraint information.
		/// @param count Number of constraints.
		void onConstraintBreak(physx::PxConstraintInfo* constraints, physx::PxU32 count);

		/// @brief Called when actor is woken.
		/// @param actors Actors being woken.
		/// @param count Number of actors.
		void onWake(physx::PxActor** actors, physx::PxU32 count);

		/// @brief Called when actors go to sleep.
		/// @param actors Actors going to sleep.
		/// @param count Number of actors.
		void onSleep(physx::PxActor** actors, physx::PxU32 count);

		/// @brief Called when trigger volume is hit.
		/// @param pairs Trigger pair.
		/// @param count Number of pairs.
		void onTrigger(physx::PxTriggerPair* pairs, physx::PxU32 count);

		/// @brief Called on simulation advance.
		/// @param rigidBody Rigid bodies.
		/// @param transform Transforms.
		/// @param count Number of rigid bodies/transforms.
		void onAdvance(const physx::PxRigidBody* const* rigidBody, const physx::PxTransform* transform, const physx::PxU32 count);

		/// @brief When a contact is found, extract the contact and add it to the contact manager.
		/// @param pairHeader Information about the contact pair.
		/// @param pairs Contact pair stream.
		/// @param nbPairs Number of pairs in the contact pair stream.
		void onContact(const physx::PxContactPairHeader& pairHeader, const physx::PxContactPair* pairs, physx::PxU32 nbPairs);
	};
}
