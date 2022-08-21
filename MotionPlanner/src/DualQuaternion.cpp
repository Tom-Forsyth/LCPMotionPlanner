#include "DualQuaternion.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <cmath>
#include <vector>
#include "DualNumber.h"

// Default constructor.
DualQuaternion::DualQuaternion() {
    this->real = Eigen::Quaternion<double> {1, 0, 0, 0};
    this->dual = Eigen::Quaternion<double> {1, 0, 0, 0};
}

// Quaternion constructor.
DualQuaternion::DualQuaternion(const Eigen::Quaternion<double> &real, const Eigen::Quaternion<double> &dual) {
    this->real = real;
    this->dual = dual;
}

// Transformation matrix constructor.
DualQuaternion::DualQuaternion(const Eigen::Matrix4d &T) {
    Eigen::Matrix3d R = T.block(0, 0, 3, 3);
    Eigen::Quaternion<double> real {R};
    Eigen::Vector4d pReverse = T.block(0, 3, 4, 1);
    pReverse(3) = 0;
    pReverse = 0.5 * pReverse;
    Eigen::Quaternion<double> Q {pReverse};
    Eigen::Quaternion<double> dual = Q * real;
    this->real = real;
    this->dual = dual;
}


// Addition.
DualQuaternion DualQuaternion::operator+(const DualQuaternion &D) {
    Eigen::Vector4d realVec = this->real.coeffs() + D.real.coeffs();
    Eigen::Vector4d dualVec = this->dual.coeffs() + D.dual.coeffs();
    Eigen::Quaternion<double> real {realVec};
    Eigen::Quaternion<double> dual {dualVec};
    DualQuaternion A {real, dual};
    return A;
}

// Subtraction.
DualQuaternion DualQuaternion::operator-(const DualQuaternion &D) {
    Eigen::Vector4d realVec = this->real.coeffs() - D.real.coeffs();
    Eigen::Vector4d dualVec = this->dual.coeffs() - D.dual.coeffs();
    Eigen::Quaternion<double> real {realVec};
    Eigen::Quaternion<double> dual {dualVec};
    DualQuaternion A {real, dual};
    return A;
}

// Multiplication.
DualQuaternion DualQuaternion::operator*(const DualQuaternion &D) {
    Eigen::Quaternion<double> real = this->real * D.real;
    Eigen::Quaternion<double> q1 = this->dual * D.real; 
    Eigen::Quaternion<double> q2 = this->real * D.dual;
    Eigen::Quaternion<double> dual = Eigen::Quaternion<double> {q1.coeffs() + q2.coeffs()};
    DualQuaternion A {real, dual};
    return A;
}

// Conjugate.
DualQuaternion DualQuaternion::conjugate() {
    Eigen::Quaternion<double> real = this->real.conjugate();
    Eigen::Quaternion<double> dual = this->dual.conjugate();
    DualQuaternion A {real, dual};
    return A;
}

// Magnitude.
double DualQuaternion::norm() {
    return this->real.norm();
}

// Normalize.
void DualQuaternion::normalize() {
    double norm = this->norm();
    Eigen::Vector4d realVec = this->real.coeffs() / norm;
    Eigen::Vector4d dualVec = this->dual.coeffs() / norm;
    Eigen::Quaternion<double> real {realVec};
    Eigen::Quaternion<double> dual {dualVec};
    this->real = real;
    this->dual = dual;
}

// Power.
DualQuaternion DualQuaternion::pow(const double &n) {
    // Get axis, angle, and p.
    Eigen::AngleAxis<double> angAxis {this->real};
    Eigen::Vector3d axis = angAxis.axis();
    double angle = angAxis.angle();
    Eigen::Vector4d p4 = 2 * (this->dual * this->real.conjugate()).coeffs();
    Eigen::Vector3d p = p4.head(3);

    // Alternate representation.
    double d = p.dot(axis);
    Eigen::Vector3d M = 0.5 * (p.cross(axis) + ((p - d*axis) * (1/tan(angle/2))));

    // Dual numbers.
    double const1 = n * angle/2;
    double const2 = n * d/2;
    DualNumber D1 {cos(const1), -const2*sin(const1)};
    DualNumber D2 {sin(const1), const2*cos(const1)};

    std::vector<DualNumber> dualVec {};
    for (int i {0}; i < 3; i++) {
        DualNumber D {axis(i), M(i)};
        dualVec.push_back(D * D2);
    }

    // Construct dual quaternion.
    Eigen::Quaternion<double> real {D1.real, dualVec.at(0).real, dualVec.at(1).real, dualVec.at(2).real};
    Eigen::Quaternion<double> dual {D1.dual, dualVec.at(0).dual, dualVec.at(1).dual, dualVec.at(2).dual};
    DualQuaternion A {real, dual};
    return A;
}

// Screw linear interpolation.
DualQuaternion DualQuaternion::ScLERP(const DualQuaternion &D, const double &tau) {
    DualQuaternion A1 = this->conjugate() * D;
    return (*this * A1.pow(tau));
}

// Convert to transformation matrix.
Eigen::Matrix4d DualQuaternion::toTransformationMatrix() {
    Eigen::Matrix3d R = this->real.toRotationMatrix();
    Eigen::Vector4d p = 2 * (this->dual * this->real.conjugate()).coeffs();
    Eigen::Matrix4d T = Eigen::Matrix4d::Zero();
    T.block(0, 0, 3, 3) = R;
    T.block(0, 3, 4, 1) = p;
    T(3, 3) = 1;
    return T;
}

// Convert to position & orientation quaternion concatination representation.
Eigen::Vector<double, 7> DualQuaternion::toConcat() {
    Eigen::Vector4d p4 = 2 * (this->dual * this->real.conjugate()).coeffs();
    Eigen::Vector3d p = p4.head(3);
    Eigen::Vector4d vecQ = this->real.coeffs();
    Eigen::Vector4d Q {vecQ(3), vecQ(0), vecQ(1), vecQ(2)};
    Eigen::Vector<double, 7> gamma = Eigen::Vector<double, 7>::Zero();
    gamma.segment(0, 3) = p;
    gamma.segment(3, 4) = Q;
    return gamma;
}
