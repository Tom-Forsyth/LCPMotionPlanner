#pragma once

#include <Eigen/Dense>

namespace CollisionAvoidance
{
	enum class JointType
	{
		Revolute,
		Prismatic,
		Fixed
	};

	class Joint
	{
	private:
		// Members.
		JointType m_type;
		Eigen::Vector3d m_axis;
		Eigen::Vector3d m_point;
		double m_displacement;
		Eigen::Vector<double, 6> m_twistCoord;
		Eigen::Matrix4d m_twist;
		Eigen::Matrix4d m_relativeTransformation;

	public:
		// Constructor.
		Joint(const JointType& type, const Eigen::Vector3d& axis, const Eigen::Vector3d& point);

		// Compute twist coordinate and twist.
		void computeTwistCoord();
		void computeTwist();

		// Compute displacement transformation.
		void computeRelativeTransformation();
		Eigen::Matrix4d getRelativeTransformation() const;

		// Set joint displacement.
		void setDisplacement(const double& displacement);

		// Get type.
		JointType getType() const;

		// Get twist coordinate.
		Eigen::Vector<double, 6> getTwistCoord() const;

		// Get displacement.
		double getDisplacement() const;
	};
}
