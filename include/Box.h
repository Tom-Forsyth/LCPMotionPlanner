#pragma once

#include "Shape.h"
#include <Eigen/Dense>
#include <string>

class Box : public Shape
{
public:
	// Vector of X, Y, and Z direction radii.
	Eigen::Vector3d m_radii;

	// Origin + RPY constructor.
	Box(const Eigen::Vector3d& origin, const Eigen::Vector3d& rollPitchYaw, const Eigen::Vector3d& radii, const std::string& name, int objectType);

};