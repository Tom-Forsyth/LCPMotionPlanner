#include "CollisionAggregate.h"
#include "Sphere.h"
#include "Capsule.h"
#include "Box.h"
#include <vector>
#include <map>
#include <Eigen/Dense>

namespace CollisionAvoidance
{
	// Default constructor.
	CollisionAggregate::CollisionAggregate()
		: m_spheres(std::vector<Sphere> {}), m_capsules(std::vector<Capsule> {}), m_boxes(std::vector<Box> {}) { }


	// Add sphere.
	void CollisionAggregate::addShape(const Sphere& sphere)
	{
		m_spheres.emplace_back(sphere);
		m_localTransforms.emplace(sphere.getName(), sphere.getTransform());
	}

	// Add capsule.
	void CollisionAggregate::addShape(const Capsule& capsule)
	{
		m_capsules.emplace_back(capsule);
		m_localTransforms.emplace(capsule.getName(), capsule.getTransform());
	}

	// Add box.
	void CollisionAggregate::addShape(const Box& box)
	{
		m_boxes.emplace_back(box);
		m_localTransforms.emplace(box.getName(), box.getTransform());
	}

	// Get spheres.
	std::vector<Sphere> CollisionAggregate::getSpheres() const
	{
		return m_spheres;
	}

	// Get capsules.
	std::vector<Capsule> CollisionAggregate::getCapsules() const
	{
		return m_capsules;
	}

	// Get boxes.
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

	std::vector<Shape*> CollisionAggregate::getColliders() const
	{
		// Setup vector of shape pointers.
		std::vector<Shape*> colliders;
		size_t numColliders = m_spheres.size() + m_capsules.size() + m_boxes.size();
		colliders.reserve(numColliders);

		// Loop over all colliders and add pointer to the vector.
		for (const Sphere& sphere : m_spheres)
		{
			colliders.emplace_back(&sphere);
		}
		for (const Capsule& capsule : m_capsules)
		{
			colliders.emplace_back(&capsule);
		}
		for (const Box& box : m_boxes)
		{
			colliders.emplace_back(&box);
		}

		// Return vector.
		return colliders;
	}
}
