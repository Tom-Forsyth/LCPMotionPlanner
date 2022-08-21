#ifndef DUALQUATERNION_H_
#define DUALQUATERNION_H_
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Dual Quaternion class.
class DualQuaternion {
    public:
        // Attributes.
        Eigen::Quaternion<double> real;
        Eigen::Quaternion<double> dual;

        // Constructors.
        DualQuaternion();
        DualQuaternion(const Eigen::Quaternion<double> &real, const Eigen::Quaternion<double> &dual);
        DualQuaternion(const Eigen::Matrix4d &T);

        //Methods.
        DualQuaternion operator+(const DualQuaternion &D);
        DualQuaternion operator-(const DualQuaternion &D);
        DualQuaternion operator*(const DualQuaternion &D);
        DualQuaternion conjugate();
        double norm();
        void normalize();
        DualQuaternion pow(const double &n);
        DualQuaternion ScLERP(const DualQuaternion &D, const double &tau);
        Eigen::Matrix4d toTransformationMatrix();
        Eigen::Vector<double, 7> toConcat();
};

#endif