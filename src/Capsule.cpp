#include "Capsule.h"
#include "Shape.h"
#include "ObjectType.h"
#include <Eigen/Dense>
#include <string>

namespace MotionPlanner
{
	Capsule::Capsule(const Eigen::Vector3d& origin, const Eigen::Vector3d& rollPitchYaw, double halfHeight, double radius, const std::string& name, ObjectType objectType)
		: Shape(origin, rollPitchYaw, name, objectType, ShapeType::Capsule), m_halfHeight(halfHeight), m_radius(radius) { }
}
