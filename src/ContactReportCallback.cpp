#include "ContactReportCallback.h"
#include "ContactPoint.h"
#include "PhysicsScene.h"
#include "ContactManager.h"
#include "PhysXHelperFunctions.h"
#include "Shape.h"
#include "ObjectType.h"
#include "PxPhysicsAPI.h"
#include <vector>
#include <string>
#include <Eigen/Dense>

namespace MotionPlanner
{
	
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

	void ContactReportCallback::onAdvance(const physx::PxRigidBody* const* rigidBody, const physx::PxTransform* transform, const physx::PxU32 count)
	{
		PX_UNUSED(rigidBody);
		PX_UNUSED(transform);
		PX_UNUSED(count);
	}

	void ContactReportCallback::onContact(const physx::PxContactPairHeader& pairHeader, const physx::PxContactPair* pairs, physx::PxU32 nbPairs)
	{
		// Get the scene's contact manager.
		const physx::PxScene* pxScene = (pairHeader.actors[0])->getScene();
		PhysicsScene* physicsScene = static_cast<PhysicsScene*>(pxScene->userData);

		// For each pair in the contact stream, loop over all the patches, and the contacts of each patch.
		for (physx::PxU32 i = 0; i < nbPairs; i++)
		{
			// Current contact pair.
			const physx::PxContactPair& pair = pairs[i];

			// Iterator for the pair's compressed contact stream.
			physx::PxContactStreamIterator iter(pair.contactPatches, pair.contactPoints, pair.getInternalFaceIndices(), pair.patchCount, pair.contactCount);

			// Flag to determine if the provided contact is flipped with respect to the shape pair.
			const physx::PxU32 flippedContacts = (pair.flags & physx::PxContactPairFlag::eINTERNAL_CONTACTS_ARE_FLIPPED);

			// Get the collision actors for both shapes.
			const Shape* collider0 = static_cast<const Shape*>(pair.shapes[0]->userData);
			const Shape* collider1 = static_cast<const Shape*>(pair.shapes[1]->userData);

			// Get the parent body names and object types.
			const std::string linkName0 = collider0->getParentBodyName();
			const std::string linkName1 = collider1->getParentBodyName();
			const ObjectType objectType0 = collider0->getObjectType();
			const ObjectType objectType1 = collider1->getObjectType();

			// Determine if the first shape of the pair is the robot or obstacle and assign the name.
			const bool firstShapeIsRobot = (objectType0 == ObjectType::RobotGeometry) ? true : false;
			const std::string linkName = firstShapeIsRobot ? linkName0 : linkName1;

			// Iterate over the patches of the pair and contact points of the patch.
			while (iter.hasNextPatch())
			{
				iter.nextPatch();
				while (iter.hasNextContact())
				{
					iter.nextContact();

					// Extract the contact point, normal, and separation.
					const physx::PxVec3 pxPoint = iter.getContactPoint();
					const physx::PxVec3 pxNormal = iter.getContactNormal();
					const physx::PxReal pxSeparation = iter.getSeparation();

					// Convert to Eigen/standard types.
					const Eigen::Vector3d point = pxVecToEigenVec(pxPoint);
					Eigen::Vector3d normal = pxVecToEigenVec(pxNormal);
					const double separation = static_cast<double>(pxSeparation);

					// Normal points from second shape to first shape.
					// We need the normal to point to the robot (direction of compensating velocity).
					// Therefore, if the first shape is not the robot, flip the normal.
					if (!firstShapeIsRobot)
					{
						normal = -normal;
					}

					/// @bug Cannot figure out how to determine which shape the contact is generated on.
					/// Ex: [ ]x   [ ]    or     [ ]     x[ ]
					/// Where [ ] is a box and x is the contact.
					/// So, for now, I will always assume the contact is generated on the robot.

					// Create contact point and add to the contact manager.
					const ContactPoint contactPoint(point, normal, separation, true);
					physicsScene->addContact(linkName, contactPoint);
				}
			}

		}
	}
}
