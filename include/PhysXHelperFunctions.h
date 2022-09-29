#pragma once

#include "Sphere.h"
#include "Capsule.h"
#include "Box.h"
#include "PxPhysicsAPI.h"
#include <Eigen/Dense>
#include <memory>

namespace MotionPlanner
{
	/// @brief Convert an Eigen::Matrix4d to a PxTransform.
	/// @param eigenMatrix Eigen matrix.
	/// @return PhysX transform.
	physx::PxTransform eigenMatrixToPxTransform(const Eigen::Matrix4d& eigenMatrix);

	// Convert a PxVec3 to an Eigen::Vector3d.

	/// @brief Convert a PxVec3 to an Eigen::Vector3d.
	/// @param pxVec PhysX Vector.
	/// @return Eigen vector.
	Eigen::Vector3d pxVecToEigenVec(const physx::PxVec3& pxVec);

	/// @brief Convert a Sphere to a PxSphereGeometry.
	/// @param sphere Sphere.
	/// @return PhysX sphere geometry.
	physx::PxSphereGeometry getPxSphereGeometry(const Sphere& sphere);

	/// @brief Convert a Capsule to a PxCapsuleGeometry.
	/// @param capsule Capsule.
	/// @return PhysX capsule geometry.
	physx::PxCapsuleGeometry getPxCapsuleGeometry(const Capsule& capsule);

	/// @brief Convert a Box to a PxBoxGeometry.
	/// @param box Box.
	/// @return PhysX box geometry.
	physx::PxBoxGeometry getPxBoxGeometry(const Box& box);

	/// @brief Convert a Shape to the corresponding PxGeometry.
	/// @param shape Shape.
	/// @return PhysX geometry.
	std::unique_ptr<physx::PxGeometry> getPxGeometry(const Shape& shape);
}