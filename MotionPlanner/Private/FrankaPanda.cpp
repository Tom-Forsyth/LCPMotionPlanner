#include "FrankaPanda.h"
#include <Eigen/Dense>
#include <vector>
#include <iostream>

FrankaPanda::FrankaPanda()
{

}

FrankaPanda::FrankaPanda(const Eigen::Vector3d &argOrigin)
{
    const int argDoF = 7;

    Eigen::Vector<double, 5> distances = {0.3330, 0.3160, 0.3840, 0.0880, 0.1070};
    Eigen::Vector<double, 8> linkLengths = {0.141, 0.192, 0.194, 0.150426, 0.152869, 0.259, 0.101811, 0.0558};
    Eigen::Vector<std::string, argDoF> argTypeJoints {"R", "R", "R", "R", "R", "R", "R"};
    Eigen::Matrix<double, argDoF, 3> argAxisJoints {
        {0, 0, 1},
        {0, -1, 0},
        {0, 0, 1},
        {0, 1, 0},
        {0, 0, 1},
        {0, 1, 0},
        {0, 0, -1}
    };
    Eigen::Matrix<double, argDoF, 3> argQJoints {
        {0, 0, linkLengths(0)},
        {0, 0, linkLengths(0)+linkLengths(1)},
        {0, 0, linkLengths(0)+linkLengths(1)+linkLengths(2)},
        {distances(3), 0, distances(0)+distances(1)},
        {0, 0, distances(0)+distances(1)+0.125},
        {0, 0, distances(0)+distances(1)+distances(2)},
        {distances(3), 0, distances(0)+distances(1)+distances(2)-0.0512}
    };
    Eigen::Vector<double, argDoF> argJointAngles = Eigen::Vector<double, argDoF>::Zero();

    // Initialize reference frames for each link/frame.
    Eigen::Matrix4d gBase1 {
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    };
    gBase1.block(0, 3, 3, 1) = argQJoints.row(0).transpose();
    Eigen::Matrix4d gBase2 {
        {1, 0, 0, 0},
        {0, -1, 0, 0},
        {0, 0, -1, 0},
        {0, 0, 0, 1}
    };
    gBase2.block(0, 3, 3, 1) = argQJoints.row(1).transpose();
    Eigen::Matrix4d gBase3 {
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    };
    gBase3.block(0, 3, 3, 1) = argQJoints.row(2).transpose();
    Eigen::Matrix4d gBase4 {
        {1, 0, 0, 0},
        {0, 0, 1, 0},
        {0, -1, 0, 0},
        {0, 0, 0, 1}
    };
    gBase4.block(0, 3, 3, 1) = argQJoints.row(3).transpose();
    Eigen::Matrix4d gBase5 {
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    };
    gBase5.block(0, 3, 3, 1) = argQJoints.row(4).transpose();
    Eigen::Matrix4d gBase6 {
        {1, 0, 0, 0},
        {0, 0, 1, 0},
        {0, -1, 0, 0},
        {0, 0, 0, 1}
    };
    gBase6.block(0, 3, 3, 1) = argQJoints.row(5).transpose();
    Eigen::Matrix4d gBase7 {
        {1, 0, 0, linkLengths(3)},
        {0, -1, 0, 0},
        {0, 0, -1, 0},
        {0, 0, 0, 1}
    };
    gBase7.block(0, 3, 3, 1) = argQJoints.row(6).transpose();
    Eigen::Matrix4d gBaseTool {
        {1, 0, 0, distances(3)},
        {0, -1, 0, 0},
        {0, 0, -1, distances(0)+distances(1)+distances(2)-distances(4)},
        {0, 0, 0, 1}
    };
    std::vector<Eigen::Matrix4d> argGBaseFrames0 {gBase1, gBase2, gBase3, gBase4, gBase5, gBase6, gBase7, gBaseTool};

    this->dof = argDoF;
    this->origin = argOrigin;
    this->jointAngles = argJointAngles;
    this->axisJoints = argAxisJoints;
    this->typeJoints = argTypeJoints;
    this->qJoints = argQJoints;
    this->gBaseFrames0 = argGBaseFrames0;
    setGOriginBase();
    update();
}