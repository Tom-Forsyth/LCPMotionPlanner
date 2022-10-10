#pragma once

#include "Sphere.h"
#include "Capsule.h"
#include "Box.h"
#include <vector>
#include <map>
#include <Eigen/Dense>

namespace MotionPlanner
{
	/// @brief Hold the collision primatives for a rigid body.
	class CollisionAggregate
	{
	private:
		/// @brief Vector of sphere actors.
		std::vector<Sphere> m_spheres;

		/// @brief Vector of capsule actors.
		std::vector<Capsule> m_capsules;

		/// @brief Vector of box actors.
		std::vector<Box> m_boxes;

		/// @brief Local transforms of the actors with respect to the rigid body frame.
		std::map<std::string, Eigen::Matrix4d> m_localTransforms;

	public:
		/// @brief Constructor.
		CollisionAggregate();

		/// @brief Destructor.
		~CollisionAggregate();

		/// @brief Add a shape to the collision aggregate.
		/// @param shape Shape actor.
		void addShape(const Shape& shape);

		/// @brief Get the spheres in collision aggregate.
		/// @return Vector of spheres.
		std::vector<Sphere> getSpheres() const;

		/// @brief Get the capsules in collision aggregate.
		/// @return Vector of capsules.
		std::vector<Capsule> getCapsules() const;

		/// @brief Get the boxes in collision aggregate.
		/// @return Vector of boxes.
		std::vector<Box> getBoxes() const;

		/// @brief Update the transforms of the shape actors.
		/// @param worldTransform World space transform of the parent body that owns the shape actors.
		void updateColliderTransforms(const Eigen::Matrix4d& worldTransform);

		/// @brief Get a vector of pointers to all the shape actors in the aggregate.
		/// @return Vector of shape pointers.
		std::vector<const Shape*> getColliders() const;

		/// @brief Set the parent body name of all the shapes.
		/// @param parentBodyName Name of parent rigid body.
		void setParentBodyName(const std::string& parentBodyName);
	};
}
