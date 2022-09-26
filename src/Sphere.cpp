#include "Sphere.h"
#include "Shape.h"
#include "ObjectType.h"
#include <Eigen/Dense>
#include <string>

namespace CollisionAvoidance
{
	// Origin + RPY constructor.
	Sphere::Sphere(const Eigen::Vector3d& origin, const Eigen::Vector3d& rollPitchYaw, double radius, const std::string& name, ObjectType objectType)
		: Shape(origin, rollPitchYaw, name, objectType, ShapeType::Sphere), m_radius(radius) { }
}
