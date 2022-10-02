#pragma once

#include <Eigen/Dense>

namespace MotionPlanner
{
    /// @brief Dual quaternion.
    class DualQuaternion {
    public:
        /// @brief Real quaternion component.
        Eigen::Quaterniond m_real;

        /// @brief Dual quaternion component.
        Eigen::Quaterniond m_dual;

        /// @brief Default constructor.
        DualQuaternion();

        /// @brief Constructor from quaternion components.
        /// @param real Real quaternion component.
        /// @param dual Dual quaternion component.
        DualQuaternion(const Eigen::Quaterniond& real, const Eigen::Quaterniond& dual);

        /// @brief Constructor from transformation matrix.
        /// @param transform Transformation matrix.
        DualQuaternion(const Eigen::Matrix4d& transform);

        /// @brief Addition.
        /// @param dualQuaternion Dual quaternion to add to this.
        /// @return Sum.
        DualQuaternion operator+(const DualQuaternion& dualQuaternion) const;

        /// @brief Subtraction.
        /// @param dualQuaternion Dual quaternion to subtract from this.
        /// @return Difference.
        DualQuaternion operator-(const DualQuaternion& dualQuaternion) const;

        /// @brief Multiplication.
        /// @param dualQuaternion Dual quaternion to multiply this by.
        /// @return Product.
        DualQuaternion operator*(const DualQuaternion& dualQuaternion) const;

        /// @brief Compute conjugate.
        /// @return Conjugate of the dual quaternion.
        DualQuaternion conjugate() const;

        /// @brief Compute the magnitude/norm.
        /// @return Magnitude of the dual quaternion.
        double norm() const;

        /// @brief Normalize the dual quaternion to make it a dual unit quaternion.
        void normalize();

        /// @brief Compute the dual quaternion raised to a power.
        /// @param exponent Power to raise the dual quaternion to.
        /// @return Dual quaternion after being raised by power.
        DualQuaternion pow(const double& exponent) const;

        /// @brief Screw linear interpolation.
        /// @param dualQuaternionEnd End point dual quaternion.
        /// @param tau Interpolation parameter in the range [0, 1].
        /// @return Interpolated dual quaternion.
        DualQuaternion ScLERP(const DualQuaternion& dualQuaternionEnd, const double& tau) const;

        /// @brief Get the transformation matrix representing this dual quaternion.
        /// @return Transformation matrix.
        Eigen::Matrix4d toTransformationMatrix() const;

        /// @brief Convert to the position-quaterion concatination representation.
        /// Ex: [x y z q0 q1 q2 q3]
        /// @return Concatination representation.
        Eigen::Vector<double, 7> toConcat() const;
    };
}
