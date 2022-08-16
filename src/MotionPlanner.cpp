#include "MotionPlanner.h"
#include "SpatialManipulator.h"
#include "DualQuaternion.h"
#include "RigidBody.h"
#include "Kinematics.h"
#include <Eigen/Dense>
#include <vector>

// Constructor.
MotionPlanner::MotionPlanner(SpatialManipulator* pSpatialManipulator)
	: m_pSpatialManipulator(pSpatialManipulator), m_plan(std::vector<Eigen::VectorXd>{})
{
	init();
}

// Initialize.
void MotionPlanner::init()
{

}

// Generate motion plan.
void MotionPlanner::computePlan(const Eigen::Matrix4d& goalTransform)
{
	// Setup dual quaternion and quaternion-position representations of start and goal.
	Eigen::Matrix4d currentTransform = m_pSpatialManipulator->getEndFrameSpatialTransform();
	DualQuaternion currentDualQuat(currentTransform);
	DualQuaternion goalDualQuat(goalTransform);
	Eigen::Vector<double, 7> currentGamma = currentDualQuat.toConcat();

	// Run stepping until convergence or divergence.
	size_t iter = 0;
	bool running = true;
	while (running && iter<m_maxIterations)
	{
		// Get end frame information.
		RigidBody endFrame = m_pSpatialManipulator->getEndFrame();
		const Eigen::MatrixXd& spatialJacobian = endFrame.getSpatialJacobian();
		const Eigen::Matrix4d& transform = endFrame.getCurrentSpatialTransform();

		// Get next intermediate pose.
		Eigen::MatrixXd B = Kinematics::BMatrix(spatialJacobian, transform);
		DualQuaternion nextDualQuat = currentDualQuat.ScLERP(goalTransform, m_tau);
		Eigen::Vector<double, 7> nextGamma = nextDualQuat.toConcat();

		// Check if angle change is too large.
		Eigen::VectorXd angleChange = B * (nextGamma - currentGamma);



	}


}