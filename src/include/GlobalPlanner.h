#pragma once

#include "RobotConfiguration.h"
#include "MotionPlanResults.h"
#include "PlannerExitCodes.h"
#include "TaskSpaceSampler.h"
#include <Eigen/Dense>
#include <vector>

namespace MotionPlanner
{
	class SpatialManipulator;

	/// @brief Global RRT framework for local LCP/SclERP planner.
	class GlobalPlanner
	{
		/// @brief Pointer to the spatial manipulator to generate the plan with.
		SpatialManipulator* m_spatialManipulator;

		/// @brief Goal pose.
		Eigen::Matrix4d m_goalPose;

		/// @brief Nodes representing valid robot configurations.
		std::vector<RobotConfiguration> m_robotConfigurationNodes;

		/// @brief Joint trajectory of the robot after planning.
		std::vector<Eigen::VectorXd> m_jointTrajectory;

		/// @brief Global planner exit code.
		GlobalPlannerExitCode m_exitCode = GlobalPlannerExitCode::Undefined;

		/// @brief True when the planner is running.
		bool m_isRunning = true;

		/// @brief Draws samples from the task space.
		TaskSpaceSampler m_sampler;

		/// @brief Add robot configuration.
		/// @param pose End-effector pose.
		/// @param jointDisplacements Robot joint displacements.
		void addConfiguration(const Eigen::Matrix4d& pose, const Eigen::VectorXd& jointDisplacements);

		/// @brief Attempt to generate local planner from the start configuration to the goal pose.
		/// @param startConfig Starting end-effector pose and joint configuration.
		/// @param goalPose Goal end-effector pose.
		/// @return Results of the local planner.
		MotionPlanResults generateLocalPlan(const RobotConfiguration& startConfig, const Eigen::Matrix4d& goalPose) const;

	public:
		/// @brief Constructor.
		GlobalPlanner(SpatialManipulator* spatialManipulator, const Eigen::Matrix4d& goalPose);

		/// @brief Run the global planner to generate a motion plan.
		void computePlan();

		/// @brief Get the joint trajectory of the robot after planning.
		MotionPlanResults getPlanResults() const;
	};
}
