#include "Capsule.h"
#include <Eigen/Dense>
#include <string>

// Origin + RPY constructor.
Capsule::Capsule(const Eigen::Vector3d& origin, const Eigen::Vector3d& rollPitchYaw, double halfHeight, double radius, const std::string& name, int objectType)
	: Shape(origin, rollPitchYaw, name, objectType), m_halfHeight(halfHeight), m_radius(radius) { }
