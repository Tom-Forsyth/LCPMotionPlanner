#pragma once

#include "SpatialManipulator.h"
#include <Eigen/Dense>

namespace CollisionAvoidance
{
	class FrankaPanda : public SpatialManipulator
	{
	public:
		FrankaPanda();
		FrankaPanda(const Eigen::Matrix4d& baseTransform);
		void initRigidBodyChain();
	};
}
