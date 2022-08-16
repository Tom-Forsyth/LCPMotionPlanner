#include "DualNumber.h"

// Constructors.
DualNumber::DualNumber()
    : m_real(0), m_dual(0) { }

DualNumber::DualNumber(const double& real, const double& dual)
    : m_real(real), m_dual(dual) { }

// Addition.
DualNumber DualNumber::operator+(const DualNumber& dualNumber) const
{
    double realNew = m_real + dualNumber.m_real;
    double dualNew = m_dual + dualNumber.m_dual;
    return DualNumber(realNew, dualNew);
}

// Multiplication.
DualNumber DualNumber::operator*(const DualNumber& dualNumber) const
{
    double realNew = m_real * dualNumber.m_real;
    double dualNew = (m_real * dualNumber.m_dual) + (dualNumber.m_real * m_dual);
    return DualNumber(realNew, dualNew);
}