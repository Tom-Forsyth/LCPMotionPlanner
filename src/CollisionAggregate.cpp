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
		m_worldTransforms.emplace(sphere.getName(), Eigen::Matrix4d::Identity());
	}

	// Add capsule.
	void CollisionAggregate::addShape(const Capsule& capsule)
	{
		m_capsules.emplace_back(capsule);
		m_worldTransforms.emplace(capsule.getName(), Eigen::Matrix4d::Identity());
	}

	// Add box.
	void CollisionAggregate::addShape(const Box& box)
	{
		m_boxes.emplace_back(box);
		m_worldTransforms.emplace(box.getName(), Eigen::Matrix4d::Identity());
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

	void CollisionAggregate::updateWorldTransforms(const Eigen::Matrix4d& worldTransform)
	{
		for (const Sphere& sphere : m_spheres)
		{
			const std::string name = sphere.getName();
			const Eigen::Matrix4d localTransform = sphere.getTransform();
			m_worldTransforms.at(name) = worldTransform * localTransform;
		}

		for (const Capsule& capsule : m_capsules)
		{
			const std::string name = capsule.getName();
			const Eigen::Matrix4d localTransform = capsule.getTransform();
			m_worldTransforms.at(name) = worldTransform * localTransform;
		}

		for (const Box& box : m_boxes)
		{
			const std::string name = box.getName();
			const Eigen::Matrix4d localTransform = box.getTransform();
			m_worldTransforms.at(name) = worldTransform * localTransform;
		}
	}
}
