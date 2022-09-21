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

		// Local transforms of the robot geometry actors with respect to the body.
		std::map<std::string, Eigen::Matrix4d> m_localTransforms;

	public:
		// Default constructor.
		CollisionAggregate();

		// Add shapes to the aggregate. Store local transform.
		void addShape(const Sphere& sphere);
		void addShape(const Capsule& capsule);
		void addShape(const Box& box);

		// Get shapes.
		std::vector<Sphere> getSpheres() const;
		std::vector<Capsule> getCapsules() const;
		std::vector<Box> getBoxes() const;

		// Update the transforms of the shapes based on the current world transform of its parent body.
		void updateColliderTransforms(const Eigen::Matrix4d& worldTransform);

		// Get a vector of pointers to the shapes.
		std::vector<Shape*> getColliders() const;
	};
}
