#pragma once

#include <Eigen/Dense>
#include <vector>

namespace MotionPlanner
{
	/// @brief Results from the motion planner.
	struct MotionPlanResults
	{
		/// @brief Starting end-effector pose.
		Eigen::Matrix4d startPose;

		/// @brief Desired end-effector pose.
		Eigen::Matrix4d goalPose;

		/// @brief Achieved end-effector pose.
		Eigen::Matrix4d achievedPose;

		/// @brief Initial joint displacements.
		Eigen::VectorXd startJointDisplacements;

		/// @brief Achieved joint displacements.
		Eigen::VectorXd achievedJointDisplacements;

		/// @brief Planner exit code.
		int exitCode;

		/// @brief Plan computation time in ms.
		double computeTimeMilli;

		/// @brief Iterations of the inner planning loop.
		size_t plannerIterations;

		/// @brief Linear displacement of end effector through entire motion.
		double linearDisplacement;

		/// @brief Joint space trajectory of motion plan.
		std::vector<Eigen::VectorXd> motionPlan;
	};
}
