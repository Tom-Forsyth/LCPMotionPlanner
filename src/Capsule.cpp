#include "Capsule.h"
#include <Eigen/Dense>
#include <string>

// Origin + RPY constructor.
Capsule::Capsule(const Eigen::Vector3d& origin, const Eigen::Vector3d& rollPitchYaw, double halfHeight, double radius, const std::string& name)
	: Shape(origin, rollPitchYaw, name), m_halfHeight(halfHeight), m_radius(radius) { }
