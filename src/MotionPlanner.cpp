#include "MotionPlanner.h"
#include "SpatialManipulator.h"
#include "DualQuaternion.h"
#include "RigidBody.h"
#include "Kinematics.h"
#include "ContactPoint.h"
#include <Eigen/Dense>
#include <vector>
#include <map>

// Constructor.
MotionPlanner::MotionPlanner(SpatialManipulator* pSpatialManipulator, const Eigen::Matrix4d& goalTransform)
	: m_pSpatialManipulator(pSpatialManipulator), m_goalTransform(goalTransform), m_plan(std::vector<Eigen::VectorXd>{}), 
	m_endFrame(pSpatialManipulator->getEndFrame()), m_dof(pSpatialManipulator->getDof()), m_currentTransform(m_endFrame.getCurrentSpatialTransform()),
	m_currentDualQuat(DualQuaternion(m_currentTransform)), m_goalDualQuat(DualQuaternion(m_goalTransform)), m_currentConcat(m_currentDualQuat.toConcat())
{
}

// Get the change in joint displacements before correction using ScLERP.
Eigen::VectorXd MotionPlanner::getJointDisplacementChange() const
{
	// Get the joint displacement change from ScLERP.
	const Eigen::MatrixXd& spatialJacobian = m_endFrame.getSpatialJacobian();
	Eigen::MatrixXd B = Kinematics::BMatrix(spatialJacobian, m_currentTransform);
	DualQuaternion nextDualQuat = m_currentDualQuat.ScLERP(m_goalTransform, m_tau);
	Eigen::Vector<double, 7> nextConcat = nextDualQuat.toConcat();
	Eigen::VectorXd displacementChange = B * (nextConcat - m_currentConcat);

	// If displacement change is too large, scale to within the specified maximum.
	double maxDisplacementChange = displacementChange.cwiseAbs().maxCoeff();
	double scaleFactor = 1 / (std::max(maxDisplacementChange / m_maxDisplacementChangeAllowed, 1.0));
	displacementChange *= scaleFactor;

	return displacementChange;
}

// Get the null space term.
double MotionPlanner::getNullSpaceTerm() const
{
	return 1;
}

// Formulate and solve LCP to get the compensating velocities.
Eigen::VectorXd MotionPlanner::getCompensatingVelocities() const
{
    // Get the active contacts.
    std::map<int, const ContactPoint&> contactPoints;
    const RigidBodyChain& rigidBodyChain = m_pSpatialManipulator->getRigidBodyChain();
    int bodyIndex = 0;
    for (const RigidBody& body : rigidBodyChain.getRigidBodies())
    {
        if (body.isMovable())
        {
            const ContactPoint& contactPoint = body.getContactPoint();
            if (contactPoint.m_isActive)
            {
                contactPoints.insert({bodyIndex, contactPoint});
            }
        }
        bodyIndex++;
    }

    // Compute the inputs to the LCP.
    size_t dim = contactPoints.size();
    Eigen::VectorXd q = Eigen::VectorXd::Zero(dim);
    Eigen::MatrixXd M = Eigen::MatrixXd::Identity(dim, dim);
    for (int row = 0; row < dim; row++)
    {
        // Distance, normal, and contact jacobian.
        double distance = contactPoint[].m_distance;
        Eigen::Vector<double, 6> paddedNormal = Eigen::Vector<double, 6>::Zero();
        paddedNormal.head(3) = contactPoint.m_normal;    v 

        // Form qi.


    }


    for (int j = 0; j < dim; j++)
    {
        double dist = th
        Eigen::Vector<double, 6> N = this->contactNormals.at(j);
        Eigen::MatrixXd Jc = this->contactJacobians.at(j);

        // Form qi.
        double product = (N.transpose() * Jc) * (k * B * (gammaNew - gamma));
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
}

// Generate motion plan.
void MotionPlanner::computePlan()
{
	// Run stepping until convergence or divergence.
	size_t iter = 0;
	bool running = true;
	while (running && iter<m_maxIterations)
	{
		// Get the joint displacement change from ScLERP, scaled to respect linearization.
		Eigen::VectorXd displacementChange = getJointDisplacementChange();

		// Get the null space term.
		double nullSpaceTerm = getNullSpaceTerm();

		// Formulate and solve LCP to get the compensating velocities.
        Eigen::VectorXd compensatingVelocities = getCompensatingVelocities();


	

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
        Eigen::VectorXd angleDelta = (k * B * (gammaNew - gamma)) + nullSpaceTerm * thetaAddSum;
        double maxAngle = angleDelta.cwiseAbs().maxCoeff();
        double maxAngleAllowed = 0.01;
        double scaleFactor = std::max(maxAngle / maxAngleAllowed, 1.0);
        double beta = (1 / scaleFactor);

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
        if (goalDist < .001 && tau == 1) {
            run = false;
            std::cout << "Complete" << std::endl;
        }
        if (goalDist < .05) {
            tau = 1;
        }

        // Find actual achieved gamma.
        DualQuaternion correctedPose{ gNew };
        Eigen::Vector<double, 7> gammaCorrected = correctedPose.toConcat();

        // Store old variables and restart loop.
        A = Anew;
        gamma = gammaCorrected;
        g = gNew;
        i++;


		iter++;
	}


}