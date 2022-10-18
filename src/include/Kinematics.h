#pragma once

#include <Eigen/Dense>

namespace MotionPlanner
{
    /// @brief Robotics kinematics functions.
    namespace Kinematics
    {
        /// @brief Compute skew symmetric form of an axis.
        /// @param w Axis.
        /// @return Skew symmetric form of axis.
        Eigen::Matrix3d skew(const Eigen::Vector3d& w);

        /// @brief Compute the rotation matrix from axis-angle representation.
        /// @param axis Axis.
        /// @param angle Angle.
        /// @return Rotation matrix.
        Eigen::Matrix3d AxisAngletoRot(const Eigen::Vector3d& axis, double angle);

        /// @brief Compute the adjoint matrix of a transformation matrix.
        /// @param G Transformation matrix.
        /// @return Adjoint form of transformation matrix.
        Eigen::Matrix<double, 6, 6> adjoint(const Eigen::Matrix4d& G);

        /// @brief Compute an efficient inverse of a transformation matrix.
        /// @param transform Transformation matrix.
        /// @return Inverse transformation.
        Eigen::Matrix4d transformInverse(const Eigen::Matrix4d& transform);

        /// @brief Convert the spatial jacobian to an analytic jacobian at a given point represented in the spatial frame.
        /// @param spatialJacobian Spatial jacobian.
        /// @param point Point in spatial frame at which to find the analytic jacobian. 
        /// @return Analytic jacobian.
        Eigen::MatrixXd spatialToAnalyticJacobian(const Eigen::MatrixXd& spatialJacobian, const Eigen::Vector3d& point);

        /// @brief Calculate jacobian to convert gammaDot to thetaDot.
        /// @param J Spatial jacobian of end-effector.
        /// @param pose Transform of end-effector.
        /// @return B matrix to convert gammaDot to thetaDot.
        Eigen::MatrixXd BMatrix(const Eigen::MatrixXd& J, const Eigen::Matrix4d& pose);
    }
}
