#ifndef KINEMATICS_H_
#define KINEMATICS_H_
#include <vector>
#include <string>
#include <Eigen/Dense>

// Robotics kinematics functions.
namespace kinematics {
    Eigen::Matrix3d skew(const Eigen::Vector3d &w);
    Eigen::Matrix3d AxisAngletoRot(const Eigen::Vector3d &axis, const double &angle);
    std::vector<Eigen::Matrix4d> getGList(const Eigen::Matrix<double, Eigen::Dynamic, 3> &axisJoints, const Eigen::Matrix<double, Eigen::Dynamic, 3> &qJoints, const Eigen::VectorXd &jointAngles, const Eigen::Vector<std::string, Eigen::Dynamic> &typeJoints);
    Eigen::Matrix4d forwardKinematics(const int &frame, const std::vector<Eigen::Matrix4d> &gList, const std::vector<Eigen::Matrix4d> &gst0Array);
    Eigen::Vector<double, 6> twistCoordinate(const Eigen::Vector3d &u, const Eigen::Vector3d &q, const std::string &jointType);
    Eigen::Matrix4d twist(const Eigen::Vector<double, 6> &twistCoordinate);
    Eigen::Matrix<double, 6, 6> adjointOfG(const Eigen::Matrix4d &G);
    Eigen::MatrixXd spatialJacobian(const Eigen::VectorXd &angles, const Eigen::MatrixXd &axisJoints, const Eigen::MatrixXd &qJoints, const Eigen::Vector<std::string, Eigen::Dynamic> &typeJoints);
    Eigen::MatrixXd BMatrix(const Eigen::MatrixXd &J, const Eigen::Matrix4d &pose);
}

#endif