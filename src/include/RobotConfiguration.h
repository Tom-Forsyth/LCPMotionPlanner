#pragma once

#include <Eigen/Dense>

namespace MotionPlanner
{
	/// @brief Task space and configuration space location of robot.
	struct RobotConfiguration
	{
		/// @brief Pose of robot end-effector.
		Eigen::Matrix4d endEffectorPose;

		/// @brief Joint displacements of robot.
		Eigen::VectorXd jointDisplacements;

		/// @brief Constructor.
		RobotConfiguration(const Eigen::Matrix4d& argEndEffectorPose, const Eigen::VectorXd& argJointDisplacements);
	};
}
