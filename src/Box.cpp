#include "Box.h"
#include <Eigen/Dense>
#include <string>

namespace CollisionAvoidance
{
	// Origin + RPY constructor.
	Box::Box(const Eigen::Vector3d& origin, const Eigen::Vector3d& rollPitchYaw, const Eigen::Vector3d& radii, const std::string& name, int objectType)
		: Shape(origin, rollPitchYaw, name, objectType), m_radii(radii) { }
}
