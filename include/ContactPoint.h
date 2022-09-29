#pragma once

#include <Eigen/Dense>

namespace MotionPlanner
{
	struct ContactPoint
	{
		// TODO: Remove m_ from these and functions that use it.
		Eigen::Vector3d m_point = Eigen::Vector3d::Zero();
		Eigen::Vector3d m_normal = Eigen::Vector3d::Zero();
		double m_distance = 1000;
		bool m_isActive = false;

		ContactPoint();
		ContactPoint(const Eigen::Vector3d& point, const Eigen::Vector3d& normal, double distance, bool isActive);
	};
}
