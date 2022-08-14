#pragma once

#include <Eigen/Dense>

// Robotics kinematics functions.
namespace Kinematics
{
    Eigen::Matrix3d skew(const Eigen::Vector3d& w);
    Eigen::Matrix3d AxisAngletoRot(const Eigen::Vector3d& axis, double angle);
}
