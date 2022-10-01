#include "CollisionAggregate.h"
#include "Sphere.h"
#include "Capsule.h"
#include "Box.h"
#include <vector>
#include <map>
#include <Eigen/Dense>

namespace MotionPlanner
{
	CollisionAggregate::CollisionAggregate()
	{

	}

	CollisionAggregate::~CollisionAggregate()
	{

	}

	void CollisionAggregate::addShape(const Shape& shape)
	{
		ShapeType shapeType = shape.getShapeType();
		if (shapeType == ShapeType::Sphere)
		{
			m_spheres.emplace_back(static_cast<const Sphere&>(shape));
		}
		if (shapeType == ShapeType::Capsule)
		{
			m_capsules.emplace_back(static_cast<const Capsule&>(shape));
		}
		if (shapeType == ShapeType::Box)
		{
			m_boxes.emplace_back(static_cast<const Box&>(shape));
		}

		m_localTransforms.emplace(shape.getName(), shape.getTransform());
	}

	std::vector<Sphere> CollisionAggregate::getSpheres() const
	{
		return m_spheres;
	}

	std::vector<Capsule> CollisionAggregate::getCapsules() const
	{
		return m_capsules;
	}

	std::vector<Box> CollisionAggregate::getBoxes() const
	{
		return m_boxes;
	}

	void CollisionAggregate::updateColliderTransforms(const Eigen::Matrix4d& worldTransform)
	{
		for (Sphere& sphere : m_spheres)
		{
			const std::string name = sphere.getName();
			const Eigen::Matrix4d localTransform = m_localTransforms.at(name);
			sphere.setTransform(worldTransform * localTransform);
		}

		for (Capsule& capsule : m_capsules)
		{
			const std::string name = capsule.getName();
			const Eigen::Matrix4d localTransform = m_localTransforms.at(name);
			capsule.setTransform(worldTransform * localTransform);
		}

		for (Box& box : m_boxes)
		{
			const std::string name = box.getName();
			const Eigen::Matrix4d localTransform = m_localTransforms.at(name);
			box.setTransform(worldTransform * localTransform);
		}
	}

	std::vector<const Shape*> CollisionAggregate::getColliders() const
	{
		// Setup vector of shape pointers.
		std::vector<const Shape*> colliders;
		size_t numColliders = m_spheres.size() + m_capsules.size() + m_boxes.size();
		colliders.reserve(numColliders);

		// Loop over all colliders and add pointer to the vector.
		for (const Sphere& sphere : m_spheres)
		{
			const Shape* spherePointer = &sphere;
			colliders.emplace_back(spherePointer);
		}
		for (const Capsule& capsule : m_capsules)
		{
			const Shape* capsulePointer = &capsule;
			colliders.emplace_back(capsulePointer);
		}
		for (const Box& box : m_boxes)
		{
			const Shape* boxPointer = &box;
			colliders.emplace_back(boxPointer);
		}

		// Return vector.
		return colliders;
	}

	void CollisionAggregate::setParentBodyName(const std::string& parentBodyName)
	{
		for (Sphere& sphere : m_spheres)
		{
			sphere.setParentBodyName(parentBodyName);
		}
		for (Capsule& capsule : m_capsules)
		{
			capsule.setParentBodyName(parentBodyName);
		}
		for (Box& box : m_boxes)
		{
			box.setParentBodyName(parentBodyName);
		}
	}
}
