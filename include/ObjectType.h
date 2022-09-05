#pragma once

// Flags to determine object type for collision filtering.

namespace CollisionAvoidance
{
	struct ObjectType
	{
		enum Enum {
			eUnassigned,
			eRobotGeometry,
			eObstacle,
			eVisual
		};
	};
}
