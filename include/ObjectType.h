#pragma once

// Flags to determine object type for collision filtering.

struct ObjectType
{
	enum Enum {
		eUnassigned,
		eRobotGeometry,
		eObstacle,
		eVisual
	};
};
