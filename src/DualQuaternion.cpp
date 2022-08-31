#include "DualQuaternion.h"
#include "DualNumber.h"
#include <Eigen/Dense>
#include <vector>

namespace CollisionAvoidance
{
    // Default constructor.
    DualQuaternion::DualQuaternion()
        : m_real(Eigen::Quaterniond(1, 0, 0, 0)), m_dual(Eigen::Quaterniond(1, 0, 0, 0)) { }

    // From quaternions constructor.
    DualQuaternion::DualQuaternion(const Eigen::Quaterniond& real, const Eigen::Quaterniond& dual)
        : m_real(real), m_dual(dual) { }

    // From transform constructor.
    DualQuaternion::DualQuaternion(const Eigen::Matrix4d& transform)
    {
        Eigen::Matrix3d R = transform.block(0, 0, 3, 3);
        Eigen::Quaterniond realVec(R);
        Eigen::Vector4d pReverse = transform.block(0, 3, 4, 1);
        pReverse(3) = 0;
        pReverse = 0.5 * pReverse;
        Eigen::Quaterniond realQuat(pReverse);
        m_real = realVec;
        m_dual = Eigen::Quaterniond(realQuat * realVec);
    }

    // Addition.
    DualQuaternion DualQuaternion::operator+(const DualQuaternion& dualQuaternion) const
    {
        Eigen::Vector4d realVec = m_real.coeffs() + dualQuaternion.m_real.coeffs();
        Eigen::Vector4d dualVec = m_dual.coeffs() + dualQuaternion.m_dual.coeffs();
        return DualQuaternion(Eigen::Quaterniond(realVec), Eigen::Quaterniond(dualVec));
    }

    // Subtraction.
    DualQuaternion DualQuaternion::operator-(const DualQuaternion& dualQuaternion) const
    {
        Eigen::Vector4d realVec = m_real.coeffs() - dualQuaternion.m_real.coeffs();
        Eigen::Vector4d dualVec = m_dual.coeffs() - dualQuaternion.m_dual.coeffs();
        return DualQuaternion(Eigen::Quaterniond(realVec), Eigen::Quaterniond(dualVec));
    }

    // Multiplication.
    DualQuaternion DualQuaternion::operator*(const DualQuaternion& dualQuaternion) const
    {
        Eigen::Quaterniond realQuat = m_real * dualQuaternion.m_real;
        Eigen::Quaterniond q1 = m_dual * dualQuaternion.m_real;
        Eigen::Quaterniond q2 = m_real * dualQuaternion.m_dual;
        Eigen::Quaterniond dualQuat = Eigen::Quaterniond(q1.coeffs() + q2.coeffs());
        return DualQuaternion(realQuat, dualQuat);
    }

    // Conjugate.
    DualQuaternion DualQuaternion::conjugate() const
    {
        return DualQuaternion(m_real.conjugate(), m_dual.conjugate());
    }

    // Magnitude.
    double DualQuaternion::norm() const
    {
        return m_real.norm();
    }

    // Normalize.
    void DualQuaternion::normalize()
    {
        double norm = this->norm();
        Eigen::Vector4d realVec = m_real.coeffs() / norm;
        Eigen::Vector4d dualVec = m_dual.coeffs() / norm;
        m_real = Eigen::Quaterniond(realVec);
        m_dual = Eigen::Quaterniond(dualVec);
    }

    // Power.
    DualQuaternion DualQuaternion::pow(const double& exponent) const
    {
        // Get axis and angle from the real portion of the dual quaternion.
        Eigen::AngleAxis<double> axisAngle(m_real);
        Eigen::Vector3d axis = axisAngle.axis();
        double angle = axisAngle.angle();

        // Get the translation vector from the dual portion of the dual quaternion.
        Eigen::Vector4d transVecHomo = 2 * (m_dual * m_real.conjugate()).coeffs();
        Eigen::Vector3d transVec = transVecHomo.head(3);

        // Compute the ttranslation along the screw axis by projecting the displacement vector, and the moment vector.
        double displacement = 0;
        Eigen::Vector3d moment = Eigen::Vector3d::Zero();
        if (angle < 1e-10) // Should this be mod pi?
        {
            axis = transVec.normalized(); // Axis has no meaning at singularities so we choose this.
            displacement = transVec.dot(axis);
        }
        else
        {
            displacement = transVec.dot(axis);
            moment = 0.5 * ((transVec.cross(axis)) + (transVec - displacement * axis) * (1 / tan(angle / 2)));
        }

        // Construct new dual quaternion.
        double const1 = exponent * angle / 2;
        double const2 = exponent * displacement / 2;
        DualNumber dualNum1{ cos(const1), -const2 * sin(const1) };
        DualNumber dualNum2{ sin(const1), const2 * cos(const1) };

        std::vector<DualNumber> dualVec;
        dualVec.reserve(3);
        for (size_t i = 0; i < 3; i++) {
            DualNumber dualNum3(axis(i), moment(i));
            dualVec.emplace_back(dualNum3 * dualNum2);
        }

        Eigen::Quaterniond realQuat(dualNum1.m_real, dualVec[0].m_real, dualVec[1].m_real, dualVec[2].m_real);
        Eigen::Quaterniond dualQuat(dualNum1.m_dual, dualVec[0].m_dual, dualVec[1].m_dual, dualVec[2].m_dual);
        return DualQuaternion(realQuat, dualQuat);
    }

    // Screw linear interpolation.
    DualQuaternion DualQuaternion::ScLERP(const DualQuaternion& dualQuaternionEnd, const double& tau) const
    {
        DualQuaternion dualQuatInterp = this->conjugate() * dualQuaternionEnd;
        return *this * dualQuatInterp.pow(tau);
    }

    // Convert to transformation matrix.
    Eigen::Matrix4d DualQuaternion::toTransformationMatrix() const
    {
        Eigen::Matrix3d R = m_real.toRotationMatrix();
        Eigen::Vector4d p = 2 * (m_dual * m_real.conjugate()).coeffs();
        Eigen::Matrix4d transformation = Eigen::Matrix4d::Zero();
        transformation.block(0, 0, 3, 3) = R;
        transformation.block(0, 3, 4, 1) = p;
        transformation(3, 3) = 1;
        return transformation;
    }

    // Convert to position & orientation quaternion concatination representation.
    Eigen::Vector<double, 7> DualQuaternion::toConcat() const
    {
        Eigen::Vector4d p4 = 2 * (m_dual * m_real.conjugate()).coeffs();
        Eigen::Vector3d p = p4.head(3);
        Eigen::Vector4d vecQ = m_real.coeffs();
        Eigen::Vector4d Q(vecQ(3), vecQ(0), vecQ(1), vecQ(2));
        Eigen::Vector<double, 7> gamma = Eigen::Vector<double, 7>::Zero();
        gamma.segment(0, 3) = p;
        gamma.segment(3, 4) = Q;
        return gamma;
    }
}
