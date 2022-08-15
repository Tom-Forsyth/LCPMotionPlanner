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

    // Adjoint of transformation matrix.
    Eigen::Matrix<double, 6, 6> adjoint(const Eigen::Matrix4d& G) {
        Eigen::Matrix3d R = G.block(0, 0, 3, 3);
        Eigen::Vector3d p = G.block(0, 3, 3, 1);
        Eigen::Matrix3d pHat = skew(p);
        Eigen::Matrix3d prod = pHat * R;
        Eigen::Matrix<double, 6, 6> ad = Eigen::Matrix<double, 6, 6>::Zero();
        ad.block(0, 0, 3, 3) = R;
        ad.block(0, 3, 3, 3) = prod;
        ad.block(3, 3, 3, 3) = R;
        return ad;
    }
}