#include "Kinematics.h"
#include "DualQuaternion.h"
#include <Eigen/Dense>

namespace MotionPlanner
{
    namespace Kinematics
    {
        Eigen::Matrix3d skew(const Eigen::Vector3d& w) {
            Eigen::Matrix3d wHat{
                {0, -w(2), w(1)},
                {w(2), 0, -w(0)},
                {-w(1), w(0), 0}
            };
            return wHat;
        }

        Eigen::Matrix3d AxisAngletoRot(const Eigen::Vector3d& axis, double angle) {
            Eigen::Matrix3d wHat = skew(axis);
            Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
            Eigen::Matrix3d R = I + wHat * sin(angle) + ((1 - cos(angle)) * (wHat * wHat));
            return R;
        }

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

        Eigen::Matrix4d transformInverse(const Eigen::Matrix4d& transform)
        {
            Eigen::Matrix4d invTransform = Eigen::Matrix4d::Identity();
            Eigen::Matrix3d Rtrans = transform.block(0, 0, 3, 3).transpose();
            Eigen::Vector3d p = transform.block(0, 3, 3, 1);
            invTransform.block(0, 0, 3, 3) = Rtrans;
            invTransform.block(0, 3, 3, 1) = -Rtrans * p;
            return invTransform;
        }

        Eigen::MatrixXd spatialToAnalyticJacobian(const Eigen::MatrixXd& spatialJacobian, const Eigen::Vector3d& point)
        {
            Eigen::Matrix<double, 6, 6> conversionMatrix = Eigen::Matrix<double, 6, 6>::Identity();
            conversionMatrix.block(0, 3, 3, 3) = -skew(point);
            return conversionMatrix * spatialJacobian;
        }

        Eigen::MatrixXd BMatrix(const Eigen::MatrixXd& J, const Eigen::Matrix4d& pose) {
            // Alternate representations of end-effector pose.
            Eigen::Vector3d p = pose.block(0, 3, 3, 1);
            DualQuaternion A(pose);
            Eigen::Quaternion<double> Q = A.m_real;
            Eigen::Vector4d vecQ = Q.coeffs();
            Eigen::Vector4d q{ 0, 0, 0, 0 };
            q(0) = vecQ(3);
            q.segment(1, 3) = vecQ.segment(0, 3);
            Eigen::Matrix3d pHat = skew(p);

            // Jacobians.
            Eigen::Matrix<double, 3, 4> J1{
                {-q(1), q(0), q(3), -q(2)},
                {-q(2), -q(3), q(0), q(1)},
                {-q(3), q(2), -q(1), q(0)}
            };
            Eigen::Matrix<double, 6, 7> J2 = Eigen::Matrix<double, 6, 7>::Zero();
            J2.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
            J2.block(0, 3, 3, 4) = 2 * pHat * J1;
            J2.block(3, 3, 3, 4) = 2 * J1;

            // Compute B.
            Eigen::MatrixXd Jnew = J;
            //Jnew.row(5) = Eigen::VectorXd::Zero(J.cols());
            Eigen::MatrixXd pinvJ = Jnew.completeOrthogonalDecomposition().pseudoInverse();
            Eigen::MatrixXd B = pinvJ * J2;
            return B;
        }
    }
}
