#pragma once

#include <Eigen/Dense>

namespace MotionPlanner
{
	/// @brief Joint type classification.
	enum class JointType
	{
		Revolute,
		Prismatic,
		Fixed
	};

	/// @brief Joint.
	class Joint
	{
	private:
		/// @brief Joint type.
		JointType m_type;

		/// @brief Joint axis.
		Eigen::Vector3d m_axis;

		/// @brief Position of joint.
		Eigen::Vector3d m_point;

		/// @brief Joint displacement.
		double m_displacement = 0;

		/// @brief Twist coordinate of joint.
		Eigen::Vector<double, 6> m_twistCoord = Eigen::Vector<double, 6>::Zero();

		/// @brief Joint twist.
		Eigen::Matrix4d m_twist = Eigen::Matrix4d::Zero();

		/// @brief Relative transformation computed from exponential formula of joint twist.
		Eigen::Matrix4d m_relativeTransformation = Eigen::Matrix4d::Identity();

		/// @brief Compute twist coordinate.
		void computeTwistCoord();

		/// @brief Compute twist.
		void computeTwist();

		/// @brief Compute relative transfmation using exponential formula.
		void computeRelativeTransformation();

	public:
		/// @brief Constructor.
		/// @param type Joint type.
		/// @param axis Joint axis.
		/// @param point Origin.
		Joint(const JointType& type, const Eigen::Vector3d& axis, const Eigen::Vector3d& point);

		/// @brief Get joint displacement.
		/// @return Joint displacement.
		double getDisplacement() const;

		/// @brief Set joint displacement.
		/// @param displacement Joint displacement.
		void setDisplacement(const double& displacement);

		/// @brief Get relative transformation.
		/// @return Relative transformation.
		Eigen::Matrix4d getRelativeTransformation() const;

		/// @brief Get joint type.
		/// @return Joint type.
		JointType getType() const;

		/// @brief Get twist coordinate.
		/// @return Twist coordinate.
		Eigen::Vector<double, 6> getTwistCoord() const;
	};
}
