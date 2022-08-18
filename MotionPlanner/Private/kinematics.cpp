#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include "DualQuaternion.h"

namespace kinematics {

    // Skew symmetric matrix.
    Eigen::Matrix3d skew(const Eigen::Vector3d &w) {
        Eigen::Matrix3d wHat {
            {0, -w(2), w(1)},
            {w(2), 0, -w(0)},
            {-w(1), w(0), 0}
        };
        return wHat;
    }

    // Axis angle to rotation matrix.
    Eigen::Matrix3d AxisAngletoRot(const Eigen::Vector3d &axis, const double &angle) {
        Eigen::Matrix3d wHat = skew(axis);
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d R = I + wHat*sin(angle) + ((1 - cos(angle)) * (wHat * wHat));
        return R;
    }

    // Compute transformations/exponentials for each individual frame.
    std::vector<Eigen::Matrix4d> getGList(const Eigen::Matrix<double, Eigen::Dynamic, 3> &axisJoints, const Eigen::Matrix<double, Eigen::Dynamic, 3> &qJoints, const Eigen::VectorXd &jointAngles, const Eigen::Vector<std::string, Eigen::Dynamic> &typeJoints) {
        std::vector<Eigen::Matrix4d> gList {};
        int dim = typeJoints.size();

        for (int i {0}; i < dim; i++) {
            Eigen::Matrix4d g = Eigen::MatrixXd::Zero(4, 4);
            std::string jointType = typeJoints[i];
            double angle = jointAngles[i];
            Eigen::Vector3d q = qJoints.row(i);
            Eigen::Vector3d w = axisJoints.row(i);
            Eigen::Matrix3d I = Eigen::MatrixXd::Identity(3, 3);

            if (jointType == "R") {
                Eigen::Matrix3d R = AxisAngletoRot(w, angle);
                Eigen::Vector3d topRight = (I-R) * q;
                g.topLeftCorner(3, 3) = R;
                g.topRightCorner(3, 1) = topRight;
                g(3, 3) = 1;

            } else if (jointType == "P") {
                // Need to implement.

            }
            gList.push_back(g);
        }
        return gList;
    }

    // Forward kinematics using each frames transformation matrix.
    Eigen::Matrix4d forwardKinematics(const int &frame, const std::vector<Eigen::Matrix4d> &gList, const std::vector<Eigen::Matrix4d> &gst0Array) {
        Eigen::Matrix4d g = Eigen::Matrix4d::Identity();
        int dof = gList.size();
        for (int i {0}; i < frame + 1; i++) {
            if (i < dof) {
                g = g * gList.at(i);
            }
        }
        return (g * gst0Array.at(frame));
    } 

    // Twist coordinate.
    Eigen::Vector<double, 6> twistCoordinate(const Eigen::Vector3d &u, const Eigen::Vector3d &q, const std::string &jointType) {
        Eigen::Vector<double, 6> twist = Eigen::VectorXd::Zero(6);
        if (jointType == "R") {
            twist.segment(0, 3) = -u.cross(q);
            twist.segment(3, 3) = u;
        } else if (jointType == "P") {
            twist.segment(0, 3) = u;
        }
        return twist;
    }

    // Twist.
    Eigen::Matrix4d twist(const Eigen::Vector<double, 6> &twistCoordinate) {
        Eigen::Matrix4d twist = Eigen::MatrixXd::Zero(4, 4);
        Eigen::Vector3d v = twistCoordinate.segment(0, 3);
        Eigen::Vector3d w = twistCoordinate.segment(3, 3);
        Eigen::Matrix3d wHat = skew(w);
        twist.topLeftCorner(3, 3) = wHat;
        twist.topRightCorner(3, 1) = v;
        return twist;
    }

    // Adjoint of transformation matrix.
    Eigen::Matrix<double, 6, 6> adjointOfG(const Eigen::Matrix4d &G) {
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

    // Spatial manipulator jacobian.
    Eigen::MatrixXd spatialJacobian(const Eigen::VectorXd &jointAngles, const Eigen::MatrixXd &axisJoints, const Eigen::MatrixXd &qJoints, const Eigen::Vector<std::string, Eigen::Dynamic> &typeJoints) {
        // Get list of twist coordinates and transformation matrices.
        int dof = jointAngles.size();
        std::vector<Eigen::Matrix4d> gList = getGList(axisJoints, qJoints, jointAngles, typeJoints);
        std::vector<Eigen::Vector<double, 6>> twistCoordList {};
        for (int i {0}; i < dof; i++) {
            Eigen::Vector3d axis = axisJoints.row(i);
            Eigen::Vector3d q = qJoints.row(i);
            std::string jointType = typeJoints(i);
            Eigen::Vector<double, 6> twistCoord = twistCoordinate(axis, q, jointType);
            twistCoordList.push_back(twistCoord);
        }

        // Initialize J.
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, dof);
        J.col(0) = twistCoordList.at(0);

        // Compute each column of J.
        for (int i {1}; i < dof; i++) {
            Eigen::Matrix4d g = Eigen::Matrix4d::Identity();
            for (int j {0}; j < i; j++) {
                g = g * gList.at(j);
            }
            Eigen::Matrix<double, 6, Eigen::Dynamic> AdG = adjointOfG(g);
            Eigen::Vector<double, 6> twistCoord = twistCoordList.at(i);
            Eigen::Vector<double, 6> twistCoordPrime = AdG * twistCoord;
            J.col(i) = twistCoordPrime;
        }

        return J;
    }

    // B Matrix.
    Eigen::MatrixXd BMatrix(const Eigen::MatrixXd &J, const Eigen::Matrix4d &pose) {
        // Representations.
        Eigen::Vector3d p = pose.block(0, 3, 3, 1);
        DualQuaternion A {pose};
        Eigen::Quaternion<double> Q = A.real;
        Eigen::Vector4d vecQ = Q.coeffs();
        Eigen::Vector4d q {0, 0, 0, 0};
        q(0) = vecQ(3);
        q.segment(1, 3) = vecQ.segment(0, 3);
        Eigen::Matrix3d pHat = skew(p);

        // Jacobians.
        Eigen::Matrix<double, 3, 4> J1 {
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
