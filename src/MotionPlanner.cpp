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
	m_currentDualQuat(DualQuaternion(m_currentTransform)), m_goalDualQuat(DualQuaternion(m_goalTransform)), m_currentConcat(m_currentDualQuat.toConcat()),
    m_goalConcat(m_goalDualQuat.toConcat())
{
    m_plan.reserve(m_maxIterations + 1);
}

// Get the change in joint displacements before correction using ScLERP.
Eigen::VectorXd MotionPlanner::getJointDisplacementChange() const
{
	// Get the joint displacement change from ScLERP.
    Eigen::MatrixXd spatialJacobian = m_endFrame.getSpatialJacobian();
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

// Compute the joint displacement change to avoid collision at the next time step.
Eigen::VectorXd MotionPlanner::getCollisionDisplacementChange(const Eigen::VectorXd& displacementChange) const
{
    // Get the active contacts.
    std::map<int, const ContactPoint&> contactPoints;
    const RigidBodyChain& rigidBodyChain = m_pSpatialManipulator->getRigidBodyChain();
    const std::vector<RigidBody>& rigidBodies = rigidBodyChain.getRigidBodies();
    int bodyIndex = 0;
    for (const RigidBody& body : rigidBodies)
    {
        if (body.isMovable())
        {
            const ContactPoint& contactPoint = body.getContactPoint();
            if (contactPoint.m_isActive)
            {
                contactPoints.insert({ bodyIndex, contactPoint });            
            }
        }
        bodyIndex++;
    }

    // Create empty q vector and M matrix.
    size_t dim = contactPoints.size();
    Eigen::VectorXd q = Eigen::VectorXd::Zero(dim);
    Eigen::MatrixXd M = Eigen::MatrixXd::Identity(dim, dim);

    // Loop over each active rigid body.
    int row = 0;
    for (const auto& pair : contactPoints)
    {
        // Get distance, normal, and row contact jacobian.
        double distance = pair.second.m_distance;
        Eigen::Vector<double, 6> paddedNormal = Eigen::Vector<double, 6>::Zero();
        paddedNormal.head(3) = pair.second.m_normal;
        Eigen::MatrixXd contactJacobian = rigidBodies[pair.first].getContactJacobian();

        // Form element of q.
        q(row) = (distance - m_safetyDistance) + (((paddedNormal.transpose() * contactJacobian) * displacementChange));

        // Loop over again to form entire row of M.
        int col = 0;
        for (const auto& pair : contactPoints)
        {
            Eigen::MatrixXd colContactJacobian = rigidBodies[pair.first].getContactJacobian();
            Eigen::VectorXd colPaddedNormal = Eigen::Vector<double, 6>::Zero();
            colPaddedNormal.head(3) = pair.second.m_normal;
            M(row, col) = (((paddedNormal.transpose() * contactJacobian) * getNullSpaceTerm()) * colContactJacobian.completeOrthogonalDecomposition().pseudoInverse()) * colPaddedNormal;
            col++;
        }
        row++;
    }

    // Solve LCP for compensating velocites.
    LCP solution = LCPSolve(M, q);
    Eigen::VectorXd compensatingVelocities = solution.z;

    // Find the change in displacements based on compensating velocities.
    Eigen::VectorXd collisionDisplacementChange = Eigen::VectorXd::Zero(m_dof);
    int compVelIndex = 0;
    int movableIndex = 0;
    for (const RigidBody& body : rigidBodies)
    {
        // If the body is movable.
        if (body.isMovable())
        {
            // If the body has an active contact.
            if (contactPoints.count(movableIndex))
            {
                Eigen::MatrixXd contactJacobianInv = (body.getContactJacobian()).completeOrthogonalDecomposition().pseudoInverse();
                Eigen::Vector<double, 6> paddedNormal = Eigen::Vector<double, 6>::Zero();
                paddedNormal.head(3) = body.getContactPoint().m_normal;
                collisionDisplacementChange += (contactJacobianInv * paddedNormal) * compensatingVelocities[compVelIndex];
                compVelIndex++;
            }
            movableIndex++;
        }
    }
  
    return collisionDisplacementChange;
}

// Add joint displacements and ensure they respect the linearization assumption.
Eigen::VectorXd MotionPlanner::getTotalDisplacementChange(const Eigen::VectorXd& displacementChange, const Eigen::VectorXd& collisionDisplacementChange)
{
    Eigen::VectorXd totalDisplacementChange = displacementChange + (getNullSpaceTerm() * collisionDisplacementChange);
    double maxDisplacement = totalDisplacementChange.cwiseAbs().maxCoeff();
    double scaleFactor = 1 / (std::max(maxDisplacement / m_maxDisplacementChangeAllowed, 1.0));
    totalDisplacementChange *= scaleFactor;
    return totalDisplacementChange;
}

// Generate motion plan.
void MotionPlanner::computePlan()
{
	// Run stepping until convergence or divergence.
	size_t iter = 0;
	bool running = true;
	while (running && iter<m_maxIterations)
	{
        // Update end frame.
        m_endFrame = m_pSpatialManipulator->getEndFrame();

		// Get the joint displacement change from ScLERP, scaled to respect linearization.
        Eigen::VectorXd displacementChange = getJointDisplacementChange();

		// Formulate and solve LCP to get the compensating velocities and compute joint displacement change.
        Eigen::VectorXd collisionDisplacementChange = getCollisionDisplacementChange(displacementChange);

        // Add joint displacements and ensure they respect the linearization assumption.
        Eigen::VectorXd totalDisplacementChange = getTotalDisplacementChange(displacementChange, collisionDisplacementChange);

        // Update the robot's joint displacements.
        Eigen::VectorXd nextJointDisplacements = m_pSpatialManipulator->getJointDisplacements() + totalDisplacementChange;
        m_plan.emplace_back(nextJointDisplacements);
        m_pSpatialManipulator->setJointDisplacements(nextJointDisplacements);

        // Update end frame.
        m_endFrame = m_pSpatialManipulator->getEndFrame();

        // Get achieved pose after forward kinematics and update variables.
        Eigen::Matrix4d correctedTransform = m_pSpatialManipulator->getEndFrameSpatialTransform();
        DualQuaternion correctedDualQuat(correctedTransform);
        Eigen::Vector<double, 7> correctedConcat = correctedDualQuat.toConcat();

        // Store old variables and restart loop.
        m_currentDualQuat = m_currentDualQuat.ScLERP(m_goalTransform, m_tau);
        //m_currentDualQuat = correctedDualQuat;
        m_currentConcat = correctedConcat;
        m_currentTransform = correctedTransform;

        // Check for convergence.
        double tol = 0.015;
        if ((m_currentConcat - m_goalConcat).norm() < tol)
        {
             running = false;
        }

		iter++;
	}
}