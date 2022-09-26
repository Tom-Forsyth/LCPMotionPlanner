#pragma once

namespace CollisionAvoidance
{
	// Flag to determine object type for collision filtering.
	enum class ObjectType
	{
		Unassigned,
		RobotGeometry,
		Obstacle,
		Visual
	};
}

