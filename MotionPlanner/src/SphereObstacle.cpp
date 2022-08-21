#include "SphereObstacle.h"
#include <vector>
#include <cmath>

// Constructors.
SphereObstacle::SphereObstacle() {
    SphereObstacle::x = 0;
    SphereObstacle::y = 0;
    SphereObstacle::z = 0;
    SphereObstacle::r = 0;
};

SphereObstacle::SphereObstacle(const double &_x, const double &_y, const double &_z, const double &_r) {
    SphereObstacle::x = _x;
    SphereObstacle::y = _y;
    SphereObstacle::z = _z;
    SphereObstacle::r = _r;
};