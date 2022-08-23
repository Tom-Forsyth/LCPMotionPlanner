#include "ContactReportCallback.h"
#include "PxPhysicsAPI.h"
#include <vector>
#include "ContactPoint.h"
#include <iostream>
#include <Eigen/Dense>
#include <iostream>

void ContactReportCallback::onConstraintBreak(physx::PxConstraintInfo* constraints, physx::PxU32 count)
{
	PX_UNUSED(constraints);
	PX_UNUSED(count);
}

void ContactReportCallback::onWake(physx::PxActor** actors, physx::PxU32 count)
{
	PX_UNUSED(actors);
	PX_UNUSED(count);
}

void ContactReportCallback::onSleep(physx::PxActor** actors, physx::PxU32 count)
{
	PX_UNUSED(actors);
	PX_UNUSED(count);
}

void ContactReportCallback::onTrigger(physx::PxTriggerPair* pairs, physx::PxU32 count)
{
	PX_UNUSED(pairs);
	PX_UNUSED(count);
}

void ContactReportCallback::onAdvance(const physx::PxRigidBody* const*, const physx::PxTransform*, const physx::PxU32)
{

}

void ContactReportCallback::onContact(const physx::PxContactPairHeader& pairHeader, const physx::PxContactPair* pairs, physx::PxU32 nbPairs)
{
	std::vector<physx::PxContactPairPoint> contactPoints;
	for (physx::PxU32 i = 0; i < nbPairs; i++)
	{
		physx::PxU32 contactCount = pairs[i].contactCount;
		if (contactCount)
		{
			contactPoints.resize(contactCount);
			pairs[i].extractContacts(&contactPoints[0], contactCount);

			// Should add bool in ContactPoint called isCollider, and check this bool for both of the pairs to make sure it is indeed always the first of the pair that is a collider.
			// Should also add support for multiple contact points, or at least take the min.
			for (physx::PxU32 j = 0; j < contactCount; j++)
			{
				// Offset contact point to the collider, not the obtacle.
				Eigen::Vector3d obsPoint(contactPoints[j].position[0], contactPoints[j].position[1], contactPoints[j].position[2]);
				Eigen::Vector3d obsNormal(contactPoints[j].normal[0], contactPoints[j].normal[1], contactPoints[j].normal[2]);
				double separation = contactPoints[j].separation;
				Eigen::Vector3d colliderPoint = obsPoint + (separation/2 * obsNormal);

				physx::PxRigidActor* actor1 = pairHeader.actors[0]; // collider
				physx::PxRigidActor* actor2 = pairHeader.actors[1]; // obstacle

				ContactPoint* contact1 = static_cast<ContactPoint*>(actor1->userData);
				contact1->m_isActive = true;
				contact1->m_distance = separation;
				contact1->m_point = colliderPoint;
				contact1->m_normal = obsNormal;
			}
		}
	}
}
