#include "Sphere.h"
#include <Eigen/Dense>

namespace CollisionAvoidance
{
	// Origin + RPY constructor.
	Sphere::Sphere(const Eigen::Vector3d& origin, const Eigen::Vector3d& rollPitchYaw, double radius, const std::string& name, int objectType)
		: Shape(origin, rollPitchYaw, name, objectType), m_radius(radius) { }
}
