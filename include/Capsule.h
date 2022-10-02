#pragma once

#include "Shape.h"
#include "ObjectType.h"
#include <Eigen/Dense>
#include <string>

namespace MotionPlanner
{
	/// @brief Capsule geometry actor.
	class Capsule : public Shape
	{
	public:
		/// @brief Half height of capsule.
		double m_halfHeight;

		/// @brief Radius of capsule.
		double m_radius;

		/// @brief Constructor.
		/// @param origin Position of actor.
		/// @param rollPitchYaw Orientation of actor.
		/// @param halfHeight Half height of capsule.
		/// @param radius Radius of capsule.
		/// @param name Name of actor.
		/// @param objectType Object type classification (Robot, Obstacle, etc).
		Capsule(const Eigen::Vector3d& origin, const Eigen::Vector3d& rollPitchYaw, double halfHeight, double radius, const std::string& name, ObjectType objectType);
	};
}
