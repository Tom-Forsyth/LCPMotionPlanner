#include "PhysXHelperFunctions.h"
#include "Shape.h"
#include "Sphere.h"
#include "Capsule.h"
#include "Box.h"
#include "PxPhysicsAPI.h"
#include <Eigen/Dense>
#include <exception>
#include <memory>

namespace MotionPlanner
{
	physx::PxTransform eigenMatrixToPxTransform(const Eigen::Matrix4d& eigenMatrix)
	{
		float values[16];
		for (size_t i = 0; i < 16; i++)
		{
			values[i] = static_cast<float>(eigenMatrix(i));
		}
		physx::PxMat44 physxMatrix(values);
		return physx::PxTransform(physxMatrix);
	}

	Eigen::Vector3d pxVecToEigenVec(const physx::PxVec3& pxVec)
	{
		return Eigen::Vector3d(static_cast<double>(pxVec[0]), static_cast<double>(pxVec[1]), static_cast<double>(pxVec[2]));
	}

	physx::PxSphereGeometry getPxSphereGeometry(const Sphere& sphere)
	{
		const physx::PxReal radius = static_cast<physx::PxReal>(sphere.m_radius);
		return physx::PxSphereGeometry(radius);
	}

	physx::PxCapsuleGeometry getPxCapsuleGeometry(const Capsule& capsule)
	{
		const physx::PxReal radius = static_cast<physx::PxReal>(capsule.m_radius);
		const physx::PxReal halfHeight = static_cast<physx::PxReal>(capsule.m_halfHeight);
		return physx::PxCapsuleGeometry(radius, halfHeight);
	}

	physx::PxBoxGeometry getPxBoxGeometry(const Box& box)
	{
		const physx::PxReal xRadius = static_cast<physx::PxReal>(box.m_radii(0));
		const physx::PxReal yRadius = static_cast<physx::PxReal>(box.m_radii(1));
		const physx::PxReal zRadius = static_cast<physx::PxReal>(box.m_radii(2));
		return physx::PxBoxGeometry(xRadius, yRadius, zRadius);
	}

	std::unique_ptr<physx::PxGeometry> getPxGeometry(const Shape& shape)
	{
		ShapeType shapeType = shape.getShapeType();
		if (shapeType == ShapeType::Sphere)
		{
			const Sphere& sphere = static_cast<const Sphere&>(shape);
			return std::move(std::make_unique<physx::PxSphereGeometry>(getPxSphereGeometry(sphere)));
		}
		else if (shapeType == ShapeType::Capsule)
		{
			const Capsule& capsule = static_cast<const Capsule&>(shape);
			return std::move(std::make_unique<physx::PxCapsuleGeometry>(getPxCapsuleGeometry(capsule)));

		}
		else if (shapeType == ShapeType::Box)
		{
			const Box& box = static_cast<const Box&>(shape);
			return std::move(std::make_unique<physx::PxBoxGeometry>(getPxBoxGeometry(box)));
		}
		else
		{
			throw std::invalid_argument("Shape type must be a sphere, capsule, or box.");
		}
	}
}