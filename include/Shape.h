#pragma once

#include <Eigen/Dense>
#include <string>
#include <ContactPoint.h>

class Shape
{
public:
	ContactPoint* m_contactPoint = nullptr;

private:
	Eigen::Matrix4d m_transform;
	Eigen::Vector3d m_origin;
	Eigen::Vector3d m_rollPitchYaw;
	std::string m_name;

public:
	// Origin + RPY constructor.
	Shape(const Eigen::Vector3d& origin, const Eigen::Vector3d& rollPitchYaw, const std::string& name);

	// Compute transform from origin + RPY.
	void computeTransform();

	// Get current transform.
	Eigen::Matrix4d getTransform() const;

	// Get name.
	std::string getName() const;


};