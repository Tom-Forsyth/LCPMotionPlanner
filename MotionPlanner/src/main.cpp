#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "LCPSolve.h"
#include "SphereObstacle.h"
#include "SpatialManipulator.h"
#include "kinematics.h"
#include "DualQuaternion.h"
#include "DualNumber.h"
#include <chrono>
#include <fstream>


int main () {
    // Round outputs to console.
    std::cout.precision(4);
    
    // Franka Panda Manipulator configuration/constants.
    const int dof {7}; 
    Eigen::Vector3d robotOrigin {0, 0, 0};
    Eigen::Vector<double, 5> distances = {0.3330, 0.3160, 0.3840, 0.0880, 0.1070};
    Eigen::Vector<double, 8> linkLengths = {0.141, 0.192, 0.194, 0.150426, 0.152869, 0.259, 0.101811, 0.0558};
    Eigen::Vector<std::string, dof> typeJoints {"R", "R", "R", "R", "R", "R", "R"};
    Eigen::Matrix<double, dof, 3> axisJoints {
        {0, 0, 1},
        {0, 1, 0},
        {0, 0, 1},
        {0, -1, 0},
        {0, 0, 1},
        {0, -1, 0},
        {0, 0, -1}
    };
    Eigen::Matrix<double, dof, 3> qJoints {
        {0, 0, linkLengths(0)},
        {0, 0, linkLengths(0)+linkLengths(1)},
        {0, 0, linkLengths(0)+linkLengths(1)+linkLengths(2)},
        //{distances(3), 0, distances(0)+distances(1)},
        {0.0825, 0, distances(0)+distances(1)},
        {0, 0, distances(0)+distances(1)+0.125},
        {0, 0, distances(0)+distances(1)+distances(2)},
        {distances(3), 0, distances(0)+distances(1)+distances(2)-0.0512}
    };
    Eigen::Vector<double, dof> jointAngles = Eigen::Vector<double, dof>::Zero();

    // Initialize reference frames for each link/frame.
    Eigen::Matrix4d gBase1 {
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    };
    gBase1.block(0, 3, 3, 1) = qJoints.row(0).transpose();
    Eigen::Matrix4d gBase2 {
        {1, 0, 0, 0},
        {0, -1, 0, 0},
        {0, 0, -1, 0},
        {0, 0, 0, 1}
    };
    gBase2.block(0, 3, 3, 1) = qJoints.row(1).transpose();
    Eigen::Matrix4d gBase3 {
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    };
    gBase3.block(0, 3, 3, 1) = qJoints.row(2).transpose();
    Eigen::Matrix4d gBase4 {
        {1, 0, 0, 0},
        {0, 0, 1, 0},
        {0, -1, 0, 0},
        {0, 0, 0, 1}
    };
    gBase4.block(0, 3, 3, 1) = qJoints.row(3).transpose();
    Eigen::Matrix4d gBase5 {
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    };
    gBase5.block(0, 3, 3, 1) = qJoints.row(4).transpose();
    Eigen::Matrix4d gBase6 {
        {1, 0, 0, 0},
        {0, 0, 1, 0},
        {0, -1, 0, 0},
        {0, 0, 0, 1}
    };
    gBase6.block(0, 3, 3, 1) = qJoints.row(5).transpose();
    Eigen::Matrix4d gBase7 {
        {1, 0, 0, linkLengths(3)},
        {0, -1, 0, 0},
        {0, 0, -1, 0},
        {0, 0, 0, 1}
    };
    gBase7.block(0, 3, 3, 1) = qJoints.row(6).transpose();
    Eigen::Matrix4d gBaseTool {
        {1, 0, 0, distances(3)},
        {0, -1, 0, 0},
        //{0, 0, -1, distances(0)+distances(1)+distances(2)-distances(4)},
        {0, 0, -1, 1.033},
        {0, 0, 0, 1}
    };
    std::vector<Eigen::Matrix4d> gBaseFrames0 {gBase1, gBase2, gBase3, gBase4, gBase5, gBase6, gBase7, gBaseTool};
    

    // Create robot object.
    SpatialManipulator panda {dof, robotOrigin, jointAngles, axisJoints, qJoints, typeJoints, gBaseFrames0};

    /*
    auto attr = panda.gBaseFrames0;
    for (int i = 0; i < attr.size(); i++) {
        std::cout << "Index: " << i << std::endl;
        std::cout << attr.at(i) << std::endl;
    }
    */
    
    // Create obstacles and initial and final configurations.
    SphereObstacle obs1 {0.06 + 0.05 + 0.02, 0.0, 0.649 - 0.313/2, 0.05};
    std::vector<SphereObstacle> obstacles {obs1};
    Eigen::Vector<double, dof> jointAngles1 = {0, 0, 0, 0, 0, 0, 0};
    Eigen::Vector<double, dof> jointAngles2 = {0, M_PI/2, 0, 0, 0, 0, 0};

    // Setup initial and final configuration.
    panda.setObstacles(obstacles);
    panda.jointAngles = jointAngles2;
    panda.update();
    Eigen::Matrix4d gGoal = panda.gBaseFrames.at(dof);
    panda.jointAngles = jointAngles1;
    panda.update();
    Eigen::Matrix4d gStart = panda.gBaseFrames.at(dof);

    // Debug contact points.
    std::cout << panda.linkContactPoints[1][0] << "\n";
 
    // Generate plan.
    auto plan = panda.motionPlan(gGoal); 
    std::cout << "Iterations: " << plan.size() << std::endl << std::endl;
    std::cout << "Start: " << std::endl;
    std::cout << gStart << std::endl;
    std::cout << "Goal: " << std::endl;
    std::cout << gGoal << std::endl;
    std::cout << "Achieved: " << std::endl;
    std::cout << panda.gOriginFrames.at(7) << std::endl;


    /*
    // Write plan to CSV file.
    std::ofstream file("plan.txt");
    if (file.is_open()) {
        for (int i = 0; i < plan.size(); i++) {
            file << plan.at(i) << "\n";
        }
    }

    // Read plan from CSV file.
    int i = 0;
    std::vector<std::vector<double>> myPlan {};
    std::vector<double> angles {};
    std::ifstream myFile("plan.txt");
    std::string myLine;
    if (myFile.is_open()) {
        while (myFile) {
            std::getline(myFile, myLine);
            if (!myLine.empty()) {
                angles.push_back(std::stod(myLine));
                i++;
            }

            if (i == 7) {
                myPlan.push_back(angles);
                angles = {};
                i = 0;
            }
        }
        myFile.close();
    }

    // Junk.
    std::cout << sizeof(panda) << std::endl;
    std::cout << sizeof(gGoal) << std::endl;

    */

    return 0;
}
