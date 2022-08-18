#include "DualNumber.h"
#include <iostream>

// Constructors.
DualNumber::DualNumber() {
    this->real = 0;
    this->dual = 0;
}

DualNumber::DualNumber(const double &argReal, const double &argDual) {
    this->real = argReal;
    this->dual = argDual;
}

// Addition.
DualNumber DualNumber::operator+(const DualNumber &D) {
    double sumReal = this->real + D.real;
    double sumDual = this->dual + D.dual;
    DualNumber A {sumReal, sumDual};
    return A;
}

// Multiplication.
DualNumber DualNumber::operator*(const DualNumber &D) {
    double prodReal = this->real * D.real;
    double prodDual = (this->real * D.dual) + (D.real * this->dual);
    DualNumber A {prodReal, prodDual};
    return A;
}