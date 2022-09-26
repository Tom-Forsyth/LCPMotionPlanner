#include "ContactPoint.h"
#include <Eigen/Dense>

namespace CollisionAvoidance
{
	ContactPoint::ContactPoint()
	{

	}

	ContactPoint::ContactPoint(const Eigen::Vector3d& point, const Eigen::Vector3d& normal, double distance, bool isActive)
		: m_point(point), m_normal(normal), m_distance(distance), m_isActive(isActive)
	{

	}
}