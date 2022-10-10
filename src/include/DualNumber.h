#pragma once

namespace MotionPlanner
{
    /// @brief Dual number.
    class DualNumber {
    public:
        /// @brief Real component.
        double m_real;

        /// @brief Dual component.
        double m_dual;

        /// @brief Default constructor.
        DualNumber();

        /// @brief Constructor with components.
        /// @param real Real component.
        /// @param dual Dual component.
        DualNumber(const double& real, const double& dual);

        /// @brief Add two dual numbers together and return the sum.
        /// @param dualNumber Other dual number to be added to this dual number.
        /// @return Dual number representing sum.
        DualNumber operator+(const DualNumber& dualNumber) const;

        /// @brief Subtract a dual number from this dual number and return the difference.
        /// @param dualNumber Dual number that we are subtracting from this dual number.
        /// @return Dual number represention difference.
        DualNumber operator*(const DualNumber& dualNumber) const;
    };
}
