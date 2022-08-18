#pragma once

#include "SpatialManipulator.h"

class FrankaPanda : public SpatialManipulator 
{
    public:
        FrankaPanda();
        FrankaPanda(const Eigen::Vector3d &origin);
};