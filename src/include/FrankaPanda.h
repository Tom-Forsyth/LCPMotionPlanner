#pragma once

#include "SpatialManipulator.h"
#include <Eigen/Dense>

namespace MotionPlanner
{
	/// @brief Franka Panda manipulator.
	class FrankaPanda : public SpatialManipulator
	{
	public:
		/// @brief Default constructor.
		FrankaPanda();

		/// @brief Constructor at a pose.
		/// @param baseTransform Pose of base.
		FrankaPanda(const Eigen::Matrix4d& baseTransform);

		/// @brief Function to create kinematic chain based on dimensions of manipulator.
		void initRigidBodyChain();
	};
}
