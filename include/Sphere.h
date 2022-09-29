#pragma once

#include "Shape.h"
#include "ObjectType.h"
#include <Eigen/Dense>
#include <string>

namespace MotionPlanner
{
	class Sphere : public Shape
	{
	public:
		double m_radius;

		// Origin + RPY constructor.
		Sphere(const Eigen::Vector3d& origin, const Eigen::Vector3d& rollPitchYaw, double radius, const std::string& name, ObjectType objectType);
	};
}
