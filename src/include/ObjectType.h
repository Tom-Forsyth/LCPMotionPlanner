#pragma once

namespace MotionPlanner
{
	/// @brief Flag to determine object type for collision filtering.
	enum class ObjectType
	{
		Unassigned,
		RobotGeometry,
		Obstacle,
		Visual
	};
}

