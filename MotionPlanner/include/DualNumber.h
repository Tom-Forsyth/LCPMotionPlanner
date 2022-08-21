#ifndef DUALNUMBER_H_
#define DUALNUMBER_H_

// Dual Number class.
class DualNumber {
    public:
        // Attributes.
        double real;
        double dual;

        // Constructors.
        DualNumber();
        DualNumber(const double &real, const double &dual);

        //Methods.
        DualNumber operator+(const DualNumber &D);
        DualNumber operator*(const DualNumber &D);
};

#endif