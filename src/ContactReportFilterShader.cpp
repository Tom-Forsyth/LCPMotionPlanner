#include "ContactReportFilterShader.h"
#include "PxPhysicsAPI.h"
#include "ObjectType.h"

namespace CollisionAvoidance
{
	// Collision pair filter shader.
	physx::PxFilterFlags contactReportFilterShader(physx::PxFilterObjectAttributes attributes0, physx::PxFilterData filterData0,
		physx::PxFilterObjectAttributes attributes1, physx::PxFilterData filterData1,
		physx::PxPairFlags& pairFlags, const void* constantBlock, physx::PxU32 constantBlockSize)
	{
		// Unused parameters.
		PX_UNUSED(attributes0);
		PX_UNUSED(attributes1);
		PX_UNUSED(constantBlockSize);
		PX_UNUSED(constantBlock);

		// Determine the type of the objects.
		const uint32_t objectType0 = filterData0.word0;
		const uint32_t objectType1 = filterData1.word0;
		const uint32_t robotGeometry = static_cast<uint32_t>(ObjectType::RobotGeometry);

		// Ignore self collisions of the robot.
		if ((objectType0 == robotGeometry) && (objectType1 == robotGeometry))
		{
			return physx::PxFilterFlag::eKILL;
		}

		// Ignore interactions that do not involve the robot.
		if ((objectType0 != robotGeometry) && (objectType1 != robotGeometry))
		{
			return physx::PxFilterFlag::eKILL;
		}

		// For the remaining pairs, we want to generate contact information, but not resolve.
		pairFlags = physx::PxPairFlag::eDETECT_DISCRETE_CONTACT | physx::PxPairFlag::eNOTIFY_CONTACT_POINTS |
			physx::PxPairFlag::eNOTIFY_TOUCH_PERSISTS | physx::PxPairFlag::eNOTIFY_TOUCH_FOUND;
		return physx::PxFilterFlag::eDEFAULT;
	}
}
