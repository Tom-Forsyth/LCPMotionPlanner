#pragma once

#include <Eigen/Dense>

namespace MotionPlanner
{
	/// @brief Struct to hold contact information.
	struct ContactPoint
	{
		/// @brief Position of the contact point.
		Eigen::Vector3d m_point = Eigen::Vector3d::Zero();

		/// @brief Normal of the contact point, typically pointing towards the robot.
		Eigen::Vector3d m_normal = Eigen::Vector3d::Zero();

		/// @brief Separation distance to contact pair.
		double m_distance = 1000;

		/// @brief Flag to determine if the contact is currently active and needs to be solved.
		bool m_isActive = false;

		/// @brief Default constructor.
		ContactPoint();

		/// @brief Constructor.
		/// @param point Position of the contact point.
		/// @param normal Normal of the contact point, typically pointer towards the robot.
		/// @param distance Separation distance to the contact pair.
		/// @param isActive Flag to determine if the contact is currently active.
		ContactPoint(const Eigen::Vector3d& point, const Eigen::Vector3d& normal, double distance, bool isActive);
	};
}
