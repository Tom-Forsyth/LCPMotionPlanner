#pragma once

#include <Eigen/Dense>

class DualQuaternion {
public:
    // Attributes.
    Eigen::Quaterniond m_real;
    Eigen::Quaterniond m_dual;

    // Constructors.
    DualQuaternion();
    DualQuaternion(const Eigen::Quaterniond& real, const Eigen::Quaterniond& dual);
    DualQuaternion(const Eigen::Matrix4d& transform);

    //Methods.
    DualQuaternion operator+(const DualQuaternion& dualQuaternion) const;
    DualQuaternion operator-(const DualQuaternion& dualQuaternion) const;
    DualQuaternion operator*(const DualQuaternion& dualQuaternion) const;
    DualQuaternion conjugate() const;
    double norm() const;
    void normalize();
    DualQuaternion pow(const double& exponent) const;
    DualQuaternion ScLERP(const DualQuaternion& dualQuaternionEnd, const double& tau) const;
    Eigen::Matrix4d toTransformationMatrix() const;
    Eigen::Vector<double, 7> toConcat() const;
};