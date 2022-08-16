#pragma once

#include "Shape.h"
#include <Eigen/Dense>
#include <string>

class Capsule : public Shape
{
public:
	double m_halfHeight;
	double m_radius;

	// Origin + RPY constructor.
	Capsule(const Eigen::Vector3d& origin, const Eigen::Vector3d& rollPitchYaw, double halfHeight, double radius, const std::string& name);

};