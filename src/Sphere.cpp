#include "Sphere.h"
#include <Eigen/Dense>

// Origin + RPY constructor.
Sphere::Sphere(const Eigen::Vector3d& origin, const Eigen::Vector3d& rollPitchYaw, double radius, const std::string& name)
	: Shape(origin, rollPitchYaw, name), m_radius(radius) { }
