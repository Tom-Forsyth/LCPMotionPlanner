#include "Shape.h"
#include <Eigen/Dense>
#include <string>
#include "ContactPoint.h"

// Origin + RPY constructor.
Shape::Shape(const Eigen::Vector3d& origin, const Eigen::Vector3d& rollPitchYaw, const std::string& name)
	: m_origin(origin), m_rollPitchYaw(rollPitchYaw), m_transform(Eigen::Matrix4d::Identity()), m_name(name)
{
	computeTransform();
	m_contactPoint = new ContactPoint;
}

// Compute transform from origin + RPY.
void Shape::computeTransform()
{
	// Create angle-axis representations of roll, pitch, and yaw.
	Eigen::AngleAxisd rollAngleAxis(m_rollPitchYaw[0], Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd pitchAngleAxis(m_rollPitchYaw[1], Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd yawAngleAxis(m_rollPitchYaw[2], Eigen::Vector3d::UnitZ());

	// Create quaternion and convert into rotation matrix.
	Eigen::Quaterniond q = yawAngleAxis * pitchAngleAxis * rollAngleAxis;
	Eigen::Matrix3d R = q.matrix();

	// Create transformation matrix.
	Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
	T.block(0, 0, 3, 3) = R;
	T.block(0, 3, 3, 1) = m_origin;
	m_transform = T;
}

// Get current transform.
Eigen::Matrix4d Shape::getTransform() const
{
	return m_transform;
}

// Get name.
std::string Shape::getName() const
{
	return m_name;
}
