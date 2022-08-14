#include "Kinematics.h"
#include <Eigen/Dense>

namespace Kinematics
{
    // Skew symmetric matrix.
    Eigen::Matrix3d skew(const Eigen::Vector3d& w) {
        Eigen::Matrix3d wHat{
            {0, -w(2), w(1)},
            {w(2), 0, -w(0)},
            {-w(1), w(0), 0}
        };
        return wHat;
    }

    
    // Axis angle to rotation matrix.
    Eigen::Matrix3d AxisAngletoRot(const Eigen::Vector3d& axis, double angle) {
        Eigen::Matrix3d wHat = skew(axis);
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d R = I + wHat * sin(angle) + ((1 - cos(angle)) * (wHat * wHat));
        return R;
    }
}