#include "ManipulatorMotionPlanner.h"
#include "SpatialManipulator.h"
#include "DualQuaternion.h"
#include "RigidBody.h"
#include "Kinematics.h"
#include "ContactPoint.h"
#include <Eigen/Dense>
#include <vector>
#include <map>

namespace MotionPlanner
{
    // Constructor.
    ManipulatorMotionPlanner::ManipulatorMotionPlanner(SpatialManipulator* pSpatialManipulator, const Eigen::Matrix4d& goalTransform)
        : m_pSpatialManipulator(pSpatialManipulator), m_goalTransform(goalTransform), m_plan(std::vector<Eigen::VectorXd>{}),
        m_endFrame(pSpatialManipulator->getEndFrame()), m_dof(pSpatialManipulator->getDof()), m_currentTransform(m_endFrame.getCurrentSpatialTransform()),
        m_currentDualQuat(DualQuaternion(m_currentTransform)), m_goalDualQuat(DualQuaternion(m_goalTransform)), m_currentConcat(m_currentDualQuat.toConcat()),
        m_goalConcat(m_goalDualQuat.toConcat())
    {
        m_plan.reserve(m_maxIterations + 1);
    }

    // Get the change in joint displacements before correction using ScLERP.
    Eigen::VectorXd ManipulatorMotionPlanner::getJointDisplacementChange()
    {
        // Get the joint displacement change from ScLERP.
        Eigen::MatrixXd spatialJacobian = m_endFrame.getSpatialJacobian();
        Eigen::MatrixXd B = Kinematics::BMatrix(spatialJacobian, m_currentTransform);
        DualQuaternion nextDualQuat = m_currentDualQuat.ScLERP(m_goalTransform, m_tau);
        Eigen::Vector<double, 7> nextConcat = nextDualQuat.toConcat();
        Eigen::VectorXd displacementChange = B * (nextConcat - m_currentConcat);

        // If displacement change is too large, scale to within the specified maximum.
        double maxDisplacementChange = displacementChange.cwiseAbs().maxCoeff();
        double scaleFactor = 1 / (std::max(maxDisplacementChange / m_maxScLERPDisplacementChange, 1.0));
        displacementChange *= scaleFactor;

        // Determine if we can increase tau.
        if (scaleFactor == 1 && !m_tauIsMax)
        {
            m_tau *= 1.1;
            if (m_tau > 1)
            {
                m_tau = 1;
                m_tauIsMax = true;
            }
        }

        return displacementChange;
    }

    // Get the null space term.
    double ManipulatorMotionPlanner::getNullSpaceTerm() const
    {
        return 1;
    }

    // Compute the joint displacement change to avoid collision at the next time step.
    Eigen::VectorXd ManipulatorMotionPlanner::getCollisionDisplacementChange(const Eigen::VectorXd& displacementChange) const
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
            for (const auto& pair2 : contactPoints)
            {
                Eigen::MatrixXd colContactJacobian = rigidBodies[pair2.first].getContactJacobian();
                Eigen::VectorXd colPaddedNormal = Eigen::Vector<double, 6>::Zero();
                colPaddedNormal.head(3) = pair2.second.m_normal;
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
    Eigen::VectorXd ManipulatorMotionPlanner::getTotalDisplacementChange(const Eigen::VectorXd& displacementChange, const Eigen::VectorXd& collisionDisplacementChange)
    {
        // Adjust total step to respect the maximum collision displacement change.
        double maxCollisionDisplacement = collisionDisplacementChange.cwiseAbs().maxCoeff();
        double collisionScaleFactor = 1 / (std::max(maxCollisionDisplacement / m_maxCollisionDisplacementChange, 1.0));
        Eigen::VectorXd totalDisplacementChange = displacementChange + (getNullSpaceTerm() * collisionDisplacementChange);
        totalDisplacementChange *= collisionScaleFactor;

        // Check that this is within the total displacement change limits.
        double maxTotalDisplacement = totalDisplacementChange.cwiseAbs().maxCoeff();
        double totalScaleFactor = 1 / (std::max(maxTotalDisplacement / m_maxTotalDisplacementChange, 1.0));
        totalDisplacementChange *= totalScaleFactor;

        return totalDisplacementChange;
    }

    // Generate motion plan.
    void ManipulatorMotionPlanner::computePlan()
    {
        // Run stepping until convergence or divergence.
        size_t iter = 0;
        bool running = true;
        while (running && iter < m_maxIterations)
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

            // Get achieved pose after forward kinematics and update variables.
            Eigen::Matrix4d correctedTransform = m_pSpatialManipulator->getEndFrameSpatialTransform();
            DualQuaternion correctedDualQuat(correctedTransform);
            Eigen::Vector<double, 7> correctedConcat = correctedDualQuat.toConcat();

            // Store old variables and restart loop.
            m_currentDualQuat = correctedDualQuat;
            m_currentConcat = correctedConcat;
            m_currentTransform = correctedTransform;

            // Check for convergence.
            Eigen::Vector<double, 7> concatError = m_goalConcat - m_currentConcat;
            if (concatError.head(3).norm() < m_positionTolerance)
            {
                if (concatError.tail(4).norm() < m_quatTolerance)
                {
                    running = false;
                }
            }

            iter++;
        }
    }

    const std::vector<Eigen::VectorXd>& ManipulatorMotionPlanner::getPlan() const
    {
        return m_plan;
    }
}
