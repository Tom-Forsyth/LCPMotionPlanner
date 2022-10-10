#pragma once

#include "Shape.h"
#include "ObjectType.h"
#include <Eigen/Dense>
#include <string>

namespace MotionPlanner
{
	/// @brief Sphere actor.
	class Sphere : public Shape
	{
	public:
		/// @brief Radius of sphere.
		double m_radius;

		/// @brief Constructor.
		/// @param origin Position of actor.
		/// @param rollPitchYaw Orientation of actor.
		/// @param radius Radius of sphere.
		/// @param name Name of actor.
		/// @param objectType Object type classification (Robot, Obstacle, ...).
		Sphere(const Eigen::Vector3d& origin, const Eigen::Vector3d& rollPitchYaw, double radius, const std::string& name, ObjectType objectType);
	};
}
