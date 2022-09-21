#pragma once

#include "Sphere.h"
#include "Capsule.h"
#include "Box.h"
#include "PxPhysicsAPI.h"
#include <Eigen/Dense>
#include <memory>

namespace CollisionAvoidance
{
	// Convert an Eigen::Matrix4d to a PxTransform.
	physx::PxTransform eigenMatrixToPxTransform(const Eigen::Matrix4d& eigenMatrix);

	// Convert a Sphere to a PxSphereGeometry.
	physx::PxSphereGeometry getPxSphereGeometry(const Sphere& sphere);

	// Convert a Capsule to a PxCapsuleGeometry.
	physx::PxCapsuleGeometry getPxCapsuleGeometry(const Capsule& capsule);

	// Convert a Box to a PxBoxGeometry.
	physx::PxBoxGeometry getPxBoxGeometry(const Box& box);

	// Convert a Shape to a PxGeometry.
	std::unique_ptr<physx::PxGeometry> getPxGeometry(const Shape& shape);
}