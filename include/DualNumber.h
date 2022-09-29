#pragma once

namespace MotionPlanner
{
    class DualNumber {
    public:
        // Attributes.
        double m_real;
        double m_dual;

        // Constructors.
        DualNumber();
        DualNumber(const double& real, const double& dual);

        // Methods.
        DualNumber operator+(const DualNumber& dualNumber) const;
        DualNumber operator*(const DualNumber& dualNumber) const;
    };
}
