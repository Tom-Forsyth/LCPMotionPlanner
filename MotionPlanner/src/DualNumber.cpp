#include "DualNumber.h"
#include <iostream>

// Constructors.
DualNumber::DualNumber() {
    this->real = 0;
    this->dual = 0;
}

DualNumber::DualNumber(const double &real, const double &dual) {
    this->real = real;
    this->dual = dual;
}

// Addition.
DualNumber DualNumber::operator+(const DualNumber &D) {
    double real = this->real + D.real;
    double dual = this->dual + D.dual;
    DualNumber A {real, dual};
    return A;
}

// Multiplication.
DualNumber DualNumber::operator*(const DualNumber &D) {
    double real = this->real * D.real;
    double dual = (this->real * D.dual) + (D.real * this->dual);
    DualNumber A {real, dual};
    return A;
}