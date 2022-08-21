#ifndef SPHEREOBSTACLE_H_
#define SPHEREOBSTACLE_H_
#include <vector>

// Sphere obstacle.
class SphereObstacle {
    public:

        // Attributes.
        double x;
        double y;
        double z;
        double r;

        // Constructor.
        SphereObstacle();
        SphereObstacle(const double &_x, const double &_y, const double &_z, const double &_r);
};

#endif