#pragma once

#include <Eigen/Dense>

class Joint
{
public:

	enum Type
	{
		REVOLUTE,
		PRISMATIC,
		FIXED
	};

private:
	// Members.
	Joint::Type m_type;
	Eigen::Vector3d m_axis;
	Eigen::Vector3d m_point;
	double m_displacement;
	Eigen::Vector<double, 6> m_twistCoord;
	Eigen::Matrix4d m_twist;
	Eigen::Matrix4d m_relativeTransformation;

public:
	// Constructor.
	Joint(const Joint::Type& type, const Eigen::Vector3d& axis, const Eigen::Vector3d& point);

	// Compute twist coordinate and twist.
	void computeTwistCoord();
	void computeTwist();

	// Compute displacement transformation.
	void computeRelativeTransformation();
	Eigen::Matrix4d getRelativeTransformation() const;

	// Set joint displacement.
	void setDisplacement(const double& displacement);

	// Get type.
	Joint::Type getType() const;

};
