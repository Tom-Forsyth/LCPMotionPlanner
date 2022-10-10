#pragma once

#include "PxPhysicsAPI.h"

namespace MotionPlanner
{
	/// @brief Collision filter called for each pair to exclude robot-robot and obstacle-obstacle interactions from generating contacts.
	/// @param attributes0 Attributes of the first shape.
	/// @param filterData0 Filtering data of the first shape.
	/// @param attributes1 Attributes of the second shape.
	/// @param filterData1 Filtering data of the second shape.
	/// @param pairFlags Bitfield of raised pair flags.
	/// @param constantBlock Void pointer of memory to include in filtering.
	/// @param constantBlockSize Size of the memory of the void pointer.
	/// @return Bitfield of raised filtering flags.
	/// @bug Try to use PxFilterData instead of void pointer for classification information and callback.
	physx::PxFilterFlags contactReportFilterShader(physx::PxFilterObjectAttributes attributes0, physx::PxFilterData filterData0,
		physx::PxFilterObjectAttributes attributes1, physx::PxFilterData filterData1,
		physx::PxPairFlags& pairFlags, const void* constantBlock, physx::PxU32 constantBlockSize);
}
