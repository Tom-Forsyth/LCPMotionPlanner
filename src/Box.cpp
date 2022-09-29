#include "Box.h"
#include "Shape.h"
#include "ObjectType.h"
#include <Eigen/Dense>
#include <string>

namespace MotionPlanner
{
	Box::Box(const Eigen::Vector3d& origin, const Eigen::Vector3d& rollPitchYaw, const Eigen::Vector3d& halfExtents, const std::string& name, ObjectType objectType)
		: Shape(origin, rollPitchYaw, name, objectType, ShapeType::Box), m_halfExtents(halfExtents) { }
}
