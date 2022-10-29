#include "GlobalPlanner.h"
#include "SpatialManipulator.h"
#include "RobotConfiguration.h"
#include "MotionPlanResults.h"
#include "PlannerExitCodes.h"
#include "LocalPlanner.h"
#include <Eigen/Dense>

namespace MotionPlanner
{
	GlobalPlanner::GlobalPlanner(SpatialManipulator* spatialManipulator, const Eigen::Matrix4d& goalPose)
		: m_spatialManipulator(spatialManipulator), m_goalPose(goalPose)
	{
		addConfiguration(m_spatialManipulator->getEndFrameSpatialTransform(), m_spatialManipulator->getJointDisplacements());

		// Reserve space for joint trajectory vector.
		size_t maxLocalPlanSize = 2000;
		size_t maxGlobalPlanSize = 10;
		m_jointTrajectory.reserve(maxLocalPlanSize * maxGlobalPlanSize);
	}

	void GlobalPlanner::computePlan()
	{
		// Try to go straight to goal.
		MotionPlanResults directAttempt = generateLocalPlan(m_robotConfigurationNodes.front(), m_goalPose);

		// If successful, return, else try RRT method.
		if (static_cast<LocalPlannerExitCode>(directAttempt.exitCode) == LocalPlannerExitCode::Success)
		{
			// Add plan to joint trajectory and robot configurations.
			for (const Eigen::VectorXd& jointConfig : directAttempt.motionPlan)
			{
				m_jointTrajectory.emplace_back(jointConfig);
			}
			m_robotConfigurationNodes.emplace_back(directAttempt.achievedPose, directAttempt.achievedJointDisplacements);

			// Stop execution.
			m_exitCode = GlobalPlannerExitCode::Success;
			m_isRunning = false;
		}
		else
		{
			/*
			* RRT Global Planner!
			*/
		}
	}

	void GlobalPlanner::addConfiguration(const Eigen::Matrix4d& pose, const Eigen::VectorXd& jointDisplacements)
	{
		m_robotConfigurationNodes.emplace_back(RobotConfiguration(pose, jointDisplacements));
	}

	MotionPlanResults GlobalPlanner::getPlanResults() const
	{
		// Plan results and goal pose.
		MotionPlanResults globalPlanResults;
		globalPlanResults.exitCode = static_cast<int>(m_exitCode);
		globalPlanResults.motionPlan = m_jointTrajectory;
		globalPlanResults.goalPose = m_goalPose;

		// Starting parameters.
		const RobotConfiguration& initialConfig = m_robotConfigurationNodes.front();
		globalPlanResults.startPose = initialConfig.endEffectorPose;
		globalPlanResults.startJointDisplacements = initialConfig.jointDisplacements;

		// Achieved parameters.
		const RobotConfiguration& finalConfig = m_robotConfigurationNodes.back();
		globalPlanResults.achievedPose = finalConfig.endEffectorPose;
		globalPlanResults.achievedJointDisplacements = finalConfig.jointDisplacements;

		return globalPlanResults;
	}

	MotionPlanResults GlobalPlanner::generateLocalPlan(const RobotConfiguration& startConfig, const Eigen::Matrix4d& goalPose) const
	{
		// Set robot to start configuration.
		m_spatialManipulator->setJointDisplacements(startConfig.jointDisplacements);

		// Run local planner.
		LocalPlanner localPlanner(m_spatialManipulator, goalPose);
		localPlanner.computePlan();
		return localPlanner.getPlanResults();
	}
}
