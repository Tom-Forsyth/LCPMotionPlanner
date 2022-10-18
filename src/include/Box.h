#pragma once

#include "Shape.h"
#include "ObjectType.h"
#include <Eigen/Dense>
#include <string>

namespace MotionPlanner
{
	/// @brief Box geometry actor.
	class Box : public Shape
	{
	public:
		/// @brief Half extents of the box.
		Eigen::Vector3d m_halfExtents;

		/// @brief Constructor.
		/// @param origin Position of actor.
		/// @param rollPitchYaw Orientation of actor.
		/// @param halfExtents Half extents/radii of box.
		/// @param name Name of actor.
		/// @param objectType Object type classification (Robot, Obstacle, etc).
		Box(const Eigen::Vector3d& origin, const Eigen::Vector3d& rollPitchYaw, const Eigen::Vector3d& halfExtents, const std::string& name, ObjectType objectType);
	};
}
