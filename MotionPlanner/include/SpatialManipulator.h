#ifndef SPATIALMANIPULATOR_H_
#define SPATIALMANIPULATOR_H_
#include <vector>
#include <Eigen/Dense>
#include "SphereObstacle.h"

// n-Bar Spatial Manipulator.
class SpatialManipulator {
    public:

    // Specified robot geometry.
    int dof;
    Eigen::Vector3d origin;
    Eigen::VectorXd jointAngles;
    Eigen::Matrix<double, Eigen::Dynamic, 3> axisJoints;
    Eigen::Matrix<double, Eigen::Dynamic, 3> qJoints;
    Eigen::Vector<std::string, Eigen::Dynamic> typeJoints;
    std::vector<Eigen::Matrix4d> gBaseFrames0;
    
    // Computed Transformations & Jacobians.
    Eigen::Matrix4d gOriginBase;
    std::vector<Eigen::Matrix4d> gBaseFrames;
    std::vector<Eigen::Matrix4d> gOriginFrames;
    std::vector<Eigen::MatrixXd> spatialJacobians;

    // Computed Contacts.
    std::vector<SphereObstacle> obstacles;
    std::vector<double> obstacleDistances;
    std::vector<Eigen::Vector3d> linkContactPoints;
    std::vector<Eigen::Vector<double, 6>> contactNormals;
    std::vector<Eigen::MatrixXd> contactJacobians;

    // Constructor.
    SpatialManipulator(const int &dof, const Eigen::Vector3d &origin, const Eigen::VectorXd &jointAngles, const Eigen::MatrixXd &axisJoints, const Eigen::MatrixXd &qJoints, const Eigen::Vector<std::string, Eigen::Dynamic> &typeJoints, const std::vector<Eigen::Matrix4d> &gBaseFrames0);

    // Methods.
    void setGOriginBase();
    void updateGBaseFrames();
    void updateGOriginFrames();
    void update();
    void computeJacobians();
    void setObstacles(const std::vector<SphereObstacle> &obstacles);
    void computeContacts();
    std::vector<Eigen::VectorXd> motionPlan(const Eigen::Matrix4d &gf);
};

#endif