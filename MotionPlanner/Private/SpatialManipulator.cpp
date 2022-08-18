#include "SpatialManipulator.h"
#include <vector>
#include "kinematics.h"
#include <iostream>
#include "DualQuaternion.h"
#include "SphereObstacle.h"
#include "LCPSolve.h"


// Constructors.
SpatialManipulator::SpatialManipulator() {

}

SpatialManipulator::SpatialManipulator(const int &dof, const Eigen::Vector3d &origin, const Eigen::VectorXd &jointAngles, const Eigen::MatrixXd &axisJoints, const Eigen::MatrixXd &qJoints, const Eigen::Vector<std::string, Eigen::Dynamic> &typeJoints, const std::vector<Eigen::Matrix4d> &gBaseFrames0) {
    this->dof = dof;
    this->origin = origin;
    this->jointAngles = jointAngles;
    this->typeJoints = typeJoints;
    this->axisJoints = axisJoints;
    this->qJoints = qJoints;
    this->gBaseFrames0 = gBaseFrames0;
    setGOriginBase();
    update();
}


// Compute transformation from origin to robot base.
void SpatialManipulator::setGOriginBase() {
    double x = this->origin[0];
    double y = this->origin[1];
    double z = this->origin[2];
    Eigen::Matrix4d g {
        {1, 0, 0, x},
        {0, 1, 0, y},
        {0, 0, 1, z},
        {0, 0, 0, 1}
    };
    this->gOriginBase = g;
}


// Compute transformations from robot base to each frame.
void SpatialManipulator::updateGBaseFrames() {
    std::vector<Eigen::Matrix4d> gList = kinematics::getGList(this->axisJoints, this->qJoints, this->jointAngles, this->typeJoints);
    std::vector<Eigen::Matrix4d> gBaseFrameList {};
    for (int i {0}; i < this->dof+1; i++) {
        Eigen::Matrix4d g = kinematics::forwardKinematics(i, gList, this->gBaseFrames0);
        gBaseFrameList.push_back(g);
    }
    this->gBaseFrames = gBaseFrameList;
}


// Compute transformations from the origin to each frame.
void SpatialManipulator::updateGOriginFrames() {
    std::vector<Eigen::Matrix4d> gList {};
    for (int i {0}; i < this->dof+1; i++) {
        gList.push_back(this->gOriginBase * this->gBaseFrames.at(i));
    }
    this->gOriginFrames = gList;
}


// Update.
void SpatialManipulator::update() {
    updateGBaseFrames();
    updateGOriginFrames();
    computeJacobians();
    computeContacts();
}


// Spatial manipulator jacobians.
void SpatialManipulator::computeJacobians() {
    // Compute spatial jacobian of last link.
    Eigen::MatrixXd Jst = kinematics::spatialJacobian(this->jointAngles, this->axisJoints, this->qJoints, this->typeJoints);

    // Create list of link spatial jacobians.
    std::vector<Eigen::MatrixXd> jacobians {};
    for (int i {0}; i < this->dof; i++) {
        jacobians.push_back(Jst);
    }
    this->spatialJacobians = jacobians;
}


// Determine list of potential obstacles to robot.
void SpatialManipulator::setObstacles(const std::vector<SphereObstacle> &argObstacles) {
    this->obstacles = argObstacles;
}


// Compute contacts between links and obstacles.
void SpatialManipulator::computeContacts() {
    int nObstacles = this->obstacles.size();

    // For each link, check each obstacle.
    std::vector<double> distances {};
    std::vector<Eigen::Vector3d> contactPoints {};
    std::vector<Eigen::Vector<double, 6>> normals {};
    distances.reserve(this->dof);
    contactPoints.reserve(this->dof);
    normals.reserve(this->dof);
    for (int i = 0; i < this->dof; i++) {
        // Link endpoints.
        Eigen::Vector3d q0 = this->gOriginFrames.at(i).block(0, 3, 3, 1);
        Eigen::Vector3d q1 = this->gOriginFrames.at(i+1).block(0, 3, 3, 1);

        // Compute distance, contact points, and normals for each link-obstacle pair.
        double distance {1000};
        Eigen::Vector3d contactPoint {};
        Eigen::Vector<double, 6> contactNormal {};
        for (int j = 0; j < nObstacles; j++) {
            Eigen::Vector3d centerPoint {this->obstacles.at(j).x, this->obstacles.at(j).y, this->obstacles.at(j).z};
            Eigen::Vector3d linkDisplacement = q1 - q0;
            double h = (((centerPoint - q0).dot(linkDisplacement)) / ((linkDisplacement).norm())) / linkDisplacement.norm();
            h = std::min(1.0, std::max(0.0, h));
            double newDistance = ((centerPoint - q0) - h*(linkDisplacement)).norm() - this->obstacles.at(j).r - 0.2;
            if (newDistance < distance) {
                distance = newDistance;
                contactPoint = (1-h)*q0 + h*q1;
                Eigen::Vector3d normal = contactPoint - centerPoint;
                normal.normalize();
                Eigen::Vector<double, 6> N = Eigen::VectorXd::Zero(6);
                N.head(3) = normal;
                contactNormal = N;
            }
        }

        distances.push_back(distance);
        contactPoints.push_back(contactPoint);
        normals.push_back(contactNormal);
    }

    this->obstacleDistances = distances;
    this->linkContactPoints = contactPoints;
    this->contactNormals = normals;

    // For each link, compute the contact jacobian.
    std::vector<Eigen::MatrixXd> contactJacobianVec {};
    contactJacobianVec.reserve(this->dof);
    for (int i = 0; i < dof; i++) {
        Eigen::MatrixXd jacobian {};

        // Jacobian to transform Js to Jc.
        Eigen::Matrix<double, 6, 6> Jconv = Eigen::Matrix<double, 6, 6>::Identity();
        Eigen::Vector3d pEnv = this->linkContactPoints.at(i);
        Eigen::Vector4d pEnvHomo = {pEnv[0], pEnv[1], pEnv[2], 1};
        Eigen::Vector3d p = (this->gOriginBase.inverse() * pEnvHomo).head(3);
        Eigen::Matrix3d pHat = kinematics::skew(p);
        Jconv.block(0, 3, 3, 3) = -pHat;

        // Compute Jc.
        Eigen::MatrixXd Js = this->spatialJacobians.at(i);
        Eigen::MatrixXd Jc = Jconv * Js;
        jacobian = Jc;

        contactJacobianVec.push_back(jacobian);
    }
    this->contactJacobians = contactJacobianVec;
}


std::vector<Eigen::VectorXd> SpatialManipulator::motionPlan(const Eigen::Matrix4d &gf) {
    // EE position/orientation representations.
    Eigen::Matrix4d g = this->gBaseFrames.at(this->dof);
    Eigen::Vector3d pGoal = gf.block(0, 3, 3, 1);
    DualQuaternion A {g};
    DualQuaternion Af {gf};
    Eigen::Vector<double, 7> gamma = A.toConcat();

    // Motion planning parameters.
    double tau = .01;
    int maxIter = 10000;
    double safetyDistance {.05};
    double h = 0.01;


    // Run stepping until convergence.
    std::vector<Eigen::VectorXd> jointAnglePlan {};
    bool run = true;
    int i = 0;
    while (run && i < maxIter) {
        // Jacobians.
        Eigen::MatrixXd J = this->spatialJacobians.at(this->dof-1);
        Eigen::MatrixXd B = kinematics::BMatrix(J, g);

        // Next goal pose.
        DualQuaternion Anew = A.ScLERP(Af, tau);
        Eigen::Vector<double, 7> gammaNew = Anew.toConcat();

        // Check angle change.
        Eigen::VectorXd angleChange = B * (gammaNew - gamma);
        double maxAngleChange = angleChange.cwiseAbs().maxCoeff();
        double maxAngleAllowed2 = 0.004;
        double scaleFactor2 = std::max(maxAngleChange / maxAngleAllowed2, 1.0); //changed 1.0 to 0.0
        double sf = (1/scaleFactor2); 
        //double k = 0.02;
        //std::cout << k << std::endl;

        // Null space.
        Eigen::MatrixXd Jmod = this->spatialJacobians.at(this->dof-1);
        //Jmod.row(5) = Eigen::VectorXd::Zero(this->dof);
        Eigen::MatrixXd invJ = Jmod.completeOrthogonalDecomposition().pseudoInverse();
        //Eigen::MatrixXd nullSpaceTerm = Eigen::MatrixXd::Identity(this->dof, this->dof) - invJ*J;
        double nullSpaceTerm = 1;

        // Compute LCP inputs and solve.
        Eigen::VectorXd q = Eigen::VectorXd::Zero(this->dof);
        Eigen::MatrixXd M = Eigen::MatrixXd::Identity(this->dof, this->dof);
        for (int j = 0; j < this->dof; j++) {

            // Extract needed components.
            double dist = this->obstacleDistances.at(j);
            Eigen::Vector<double, 6> N = this->contactNormals.at(j);
            Eigen::MatrixXd Jc = this->contactJacobians.at(j);

            // Form qi.
            double product = (N.transpose() * Jc) * (sf * B * (gammaNew - gamma));
            double qi = (dist - safetyDistance) + 1 * product;
            //std::cout << dist - safetyDistance << std::endl;
            q(j) = qi;

            // Form M[i, :].
            for (int k = 0; k < this->dof; k++) {
                Eigen::MatrixXd colJc = this->contactJacobians.at(j);
                Eigen::MatrixXd colJcMod = this->contactJacobians.at(k);
                //colJcMod.row(5) = Eigen::VectorXd::Zero(this->dof);
                Eigen::MatrixXd colJcInv = colJcMod.completeOrthogonalDecomposition().pseudoInverse();
                Eigen::Vector<double, 6> colN = this->contactNormals.at(k);
                double m = (((N.transpose() * Jc) * nullSpaceTerm) * colJcInv) * (colN);  
                M(j, k) = m;
            }
        }
        LCP sol = LCPSolve(M, q);

        // Compute sum of additional joint velocities for each contact.
        Eigen::VectorXd thetaAddSum = Eigen::VectorXd::Zero(this->dof);
        for (int j = 0; j < this->dof; j++) {
            Eigen::MatrixXd JcMod = this->contactJacobians.at(j);
            //JcMod.row(5) = Eigen::VectorXd::Zero(this->dof);
            Eigen::MatrixXd invJc = JcMod.completeOrthogonalDecomposition().pseudoInverse();
            Eigen::Vector<double, 6> Nc = this->contactNormals.at(j);
            Eigen::VectorXd thetaAdd = (invJc * Nc) * (sol.z(j)); // changed sol.z(j) to 0.
            thetaAddSum += thetaAdd;
        }

        // Compute change in joint angles.
        Eigen::VectorXd angleDelta = (sf * B * (gammaNew - gamma)) + nullSpaceTerm*thetaAddSum;
        double maxAngle = angleDelta.cwiseAbs().maxCoeff();
        double maxAngleAllowed = 0.01;
        double scaleFactor = std::max(maxAngle / maxAngleAllowed, 1.0);
        double beta = (1/scaleFactor); 
        
        // Update new angles.
        Eigen::VectorXd anglesNew = this->jointAngles + beta * angleDelta;
        jointAnglePlan.push_back(anglesNew);

        // Forward kinematics.
        //std::cout << "Angle Delta: " << (anglesNew - this->jointAngles).transpose() << std::endl;
        this->jointAngles = anglesNew;
        this->update();
        Eigen::Matrix4d gNew = this->gBaseFrames.at(this->dof);

        // Determine if tau should be adjusted by looking at end-effector displacement.
        Eigen::Vector3d pOld = g.block(0, 3, 3, 1);
        Eigen::Vector3d pNew = gNew.block(0, 3, 3, 1);
        Eigen::Vector3d pRel = pNew - pOld;
    
        /*
        double dist = pRel.norm();
        if (dist < 0.001) {
            tau *= 1.1;
        }
        if (tau > 1) {
            tau = 1;
        }
        */
        
        // Determine if we have converged at the goal.
        Eigen::Vector3d pTGoal = pGoal - pNew;
        double goalDist = pTGoal.norm();
        if (goalDist < .01) {
            run = false;
            std::cout << "Complete" << std::endl;
        }

        // Find actual achieved gamma.
        DualQuaternion correctedPose {gNew};
        Eigen::Vector<double, 7> gammaCorrected = correctedPose.toConcat();
      
        // Store old variables and restart loop.
        A = Anew;
        gamma = gammaCorrected;
        g = gNew;
        i++;
    }
    return jointAnglePlan; 
}

void SpatialManipulator::setJointAngles(const Eigen::VectorXd& newJointAngles)
{
    this->jointAngles = newJointAngles;
    update();
}
