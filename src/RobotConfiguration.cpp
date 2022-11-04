#include "RobotConfiguration.h"
#include <Eigen/Dense>

namespace MotionPlanner
{
	RobotConfiguration::RobotConfiguration()
	{

	}

	RobotConfiguration::RobotConfiguration(const Eigen::Matrix4d& argEndEffectorPose, const Eigen::VectorXd& argJointDisplacements)
		: endEffectorPose(argEndEffectorPose), jointDisplacements(argJointDisplacements)
	{

	}
}
