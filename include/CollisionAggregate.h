#pragma once

#include "Sphere.h"
#include "Capsule.h"
#include "Box.h"
#include <vector>
#include <map>
#include <Eigen/Dense>

namespace CollisionAvoidance
{
	class CollisionAggregate
	{
	private:
		std::vector<Sphere> m_spheres;
		std::vector<Capsule> m_capsules;
		std::vector<Box> m_boxes;

		// World transforms of the shapes.
		std::map<std::string, Eigen::Matrix4d> m_worldTransforms;

	public:
		// Default constructor.
		CollisionAggregate();

		// Add shapes.
		void addShape(const Sphere& sphere);
		void addShape(const Capsule& capsule);
		void addShape(const Box& box);

		// Get shapes.
		std::vector<Sphere> getSpheres() const;
		std::vector<Capsule> getCapsules() const;
		std::vector<Box> getBoxes() const;

		// Update the world transforms of the shapes.
		void updateWorldTransforms(const Eigen::Matrix4d& worldTransform);
	};
}
