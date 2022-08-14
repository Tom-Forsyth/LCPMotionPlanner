#pragma once

#include <Eigen/Dense>

struct ContactPoint
{
	Eigen::Vector3d m_point = Eigen::Vector3d::Zero();
	Eigen::Vector3d m_normal = Eigen::Vector3d::Zero();
	double m_distance = 1000;
	bool m_isActive = false;
};
