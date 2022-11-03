#include "LocalPlanner.h"
#include "SpatialManipulator.h"
#include "DualQuaternion.h"
#include "RigidBody.h"
#include "Kinematics.h"
#include "ContactPoint.h"
#include "LocalPlannerParams.h"
#include "MotionPlanResults.h"
#include <Eigen/Dense>
#include <vector>
#include <map>

namespace MotionPlanner
{
    LocalPlanner::LocalPlanner(SpatialManipulator* pSpatialManipulator, const Eigen::Matrix4d& goalTransform)
        : m_pSpatialManipulator(pSpatialManipulator), m_goalTransform(goalTransform), m_plan(std::vector<Eigen::VectorXd>{}),
        m_dof(pSpatialManipulator->getDof()), m_currentTransform(m_pSpatialManipulator->getEndFrameSpatialTransform()),
        m_currentDualQuat(DualQuaternion(m_currentTransform)), m_goalDualQuat(DualQuaternion(m_goalTransform)), m_currentConcat(m_currentDualQuat.toConcat()),
        m_goalConcat(m_goalDualQuat.toConcat()), m_startTransform(m_pSpatialManipulator->getEndFrameSpatialTransform()), 
        m_startDisplacements(m_pSpatialManipulator->getJointDisplacements())
    {
        m_plan.reserve(m_params.maxIterations + 1);
    }

    void LocalPlanner::computePlan()
    {
        // Include starting joint displacements in plan.
        Eigen::VectorXd startJointDisplacements = m_pSpatialManipulator->getJointDisplacements();
        m_plan.emplace_back(startJointDisplacements);

        // Run stepping until convergence or divergence.
        size_t iter = 0;
        while (m_isRunning)
        {
            // Update spatial jacobian and null space term.
            m_spatialJacobian = m_pSpatialManipulator->getEndFrame().getSpatialJacobian();
            computeNullSpaceTerm();

            // Get the joint displacement change from ScLERP, scaled to respect linearization.
            Eigen::VectorXd displacementChange = getJointDisplacementChange();

            // Formulate and solve LCP to get the compensating velocities and compute joint displacement change.
            Eigen::VectorXd collisionDisplacementChange = getCollisionDisplacementChange(displacementChange);

            // Add joint displacements and ensure they respect the linearization assumption.
            Eigen::VectorXd totalDisplacementChange = getTotalDisplacementChange(displacementChange, collisionDisplacementChange);

            // Update the robot's joint displacements.
            Eigen::VectorXd nextJointDisplacements = m_pSpatialManipulator->getJointDisplacements() + totalDisplacementChange;
            bool displacementsAreValid = m_pSpatialManipulator->setJointDisplacements(nextJointDisplacements);
            
            // Check for joint limit violation.
            if (!displacementsAreValid)
            {
                m_isRunning = false;
                m_exitCodePlanner = LocalPlannerExitCode::JointLimitViolation;
            }

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
            double posError = concatError.head(3).norm();
            double quatError = concatError.tail(4).norm();
            if (posError < m_params.positionTolerance && quatError < m_params.quatTolerance)
            {
                m_isRunning = false;
                m_exitCodePlanner = LocalPlannerExitCode::Success;
            }

            // Determine if there was an LCP error.
            if (m_exitCodeLCP)
            {
                m_isRunning = false;
                m_exitCodePlanner = LocalPlannerExitCode::LCPError;
            }

            // Check for penetration.
            if (isPenetrating())
            {
                m_isRunning = false;
                m_exitCodePlanner = LocalPlannerExitCode::Collision;
            }

            if (iter == m_params.maxIterations - 1)
            {
                m_isRunning = false;
                m_exitCodePlanner = LocalPlannerExitCode::MaxIterationsExceeded;
            }

            // Check if we are near goal to tighten linearization for next iteration.
            checkNearGoal(posError, quatError);

            // If the planner did not terminate with an error, add the joint configuration to the plan.
            if (m_exitCodePlanner == LocalPlannerExitCode::Success || m_exitCodePlanner == LocalPlannerExitCode::Undefined)
            {
                m_plan.emplace_back(nextJointDisplacements);
            }

            iter++;
        }
      
    }

    MotionPlanResults LocalPlanner::getPlanResults() const
    {
        MotionPlanResults planResults;
        planResults.startPose = m_startTransform;
        planResults.startJointDisplacements = m_startDisplacements;
        planResults.goalPose = m_goalTransform;
        planResults.achievedPose = m_currentTransform;
        planResults.achievedJointDisplacements = m_pSpatialManipulator->getJointDisplacements();
        planResults.exitCode = static_cast<int>(m_exitCodePlanner);
        planResults.motionPlan = m_plan;
        return planResults;
    }

    Eigen::VectorXd LocalPlanner::getJointDisplacementChange()
    {
        // Get the joint displacement change from ScLERP.
        Eigen::MatrixXd B = Kinematics::BMatrix(m_spatialJacobian, m_currentTransform);
        DualQuaternion nextDualQuat = m_currentDualQuat.ScLERP(m_goalTransform, m_params.tau);
        Eigen::Vector<double, 7> nextConcat = nextDualQuat.toConcat();
        Eigen::VectorXd displacementChange = B * (nextConcat - m_currentConcat);

        // If displacement change is too large, scale to within the specified maximum.
        double maxDisplacementChange = displacementChange.cwiseAbs().maxCoeff();
        double scaleFactor = 1 / (std::max(maxDisplacementChange / m_params.maxScLERPDisplacementChange, 1.0));
        displacementChange *= scaleFactor;

        // Determine if we can increase tau.
        if (scaleFactor == 1 && !m_params.tauIsMax)
        {
            m_params.tau *= 1.1;
            if (m_params.tau > 1)
            {
                m_params.tau = 1;
                m_params.tauIsMax = true;
            }
        }

        return displacementChange;
    }

    Eigen::VectorXd LocalPlanner::getCollisionDisplacementChange(const Eigen::VectorXd& displacementChange)
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
            q(row) = (distance - m_params.safetyDistance) + (((paddedNormal.transpose() * contactJacobian) * displacementChange));

            // Loop over again to form entire row of M.
            int col = 0;
            for (const auto& pair2 : contactPoints)
            {
                Eigen::MatrixXd colContactJacobian = rigidBodies[pair2.first].getContactJacobian();
                Eigen::VectorXd colPaddedNormal = Eigen::Vector<double, 6>::Zero();
                colPaddedNormal.head(3) = pair2.second.m_normal;
                M(row, col) = (((paddedNormal.transpose() * contactJacobian) * m_nullSpaceTerm) * colContactJacobian.completeOrthogonalDecomposition().pseudoInverse()) * colPaddedNormal;
                col++;
            }
            row++;
        }

        // Solve LCP for compensating velocites.
        LCPSolve::LCP solution = LCPSolve::LCPSolve(M, q);
        Eigen::VectorXd compensatingVelocities = solution.z;
        m_exitCodeLCP = solution.exitCond;

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

    Eigen::VectorXd LocalPlanner::getTotalDisplacementChange(const Eigen::VectorXd& displacementChange, const Eigen::VectorXd& collisionDisplacementChange)
    {
        // Adjust total step to respect the maximum collision displacement change.
        double maxCollisionDisplacement = collisionDisplacementChange.cwiseAbs().maxCoeff();
        double collisionScaleFactor = 1 / (std::max(maxCollisionDisplacement / m_params.maxCollisionDisplacementChange, 1.0));
        Eigen::VectorXd totalDisplacementChange = displacementChange + (m_nullSpaceTerm * collisionDisplacementChange);
        totalDisplacementChange *= collisionScaleFactor;

        // Check that this is within the total displacement change limits.
        double maxTotalDisplacement = totalDisplacementChange.cwiseAbs().maxCoeff();
        double totalScaleFactor = 1 / (std::max(maxTotalDisplacement / m_params.maxTotalDisplacementChange, 1.0));
        totalDisplacementChange *= totalScaleFactor;

        return totalDisplacementChange;
    }

    bool LocalPlanner::isPenetrating()
    {
        return m_pSpatialManipulator->isColliding();
    }

    void LocalPlanner::checkNearGoal(double posError, double quatError)
    {
        if (!m_isNearGoal)
        {
            // Determine if near the goal.
            double thresh = 0.03;
            if ((posError < thresh) && (quatError < thresh))
            {
                // Enforce stricter linearization assumptions.
                m_isNearGoal = true;
                double maxAngleChange = 0.001;
                m_params.maxScLERPDisplacementChange = maxAngleChange;
                m_params.maxCollisionDisplacementChange = maxAngleChange;
                m_params.maxTotalDisplacementChange = maxAngleChange;
            }
        }
    }

    void LocalPlanner::computeNullSpaceTerm()
    {
        const Eigen::MatrixXd spatialJacobianInv = m_spatialJacobian.completeOrthogonalDecomposition().pseudoInverse();
        m_nullSpaceTerm = Eigen::MatrixXd::Identity(m_dof, m_dof) - (spatialJacobianInv * m_spatialJacobian);
    }
}
