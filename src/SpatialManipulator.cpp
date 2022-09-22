#include "SpatialManipulator.h"
#include "RigidBodyChain.h"
#include "MotionPlanner.h"
#include "PhysicsScene.h"
#include <Eigen/Dense>

namespace CollisionAvoidance
{
	// Default constructor.
	SpatialManipulator::SpatialManipulator()
	{
		setBaseTransform(Eigen::Matrix4d::Identity());
	}

	// Base transform constructor.
	SpatialManipulator::SpatialManipulator(const Eigen::Matrix4d& baseTransform)
	{
		setBaseTransform(baseTransform);
	}

	// Set base transform.
	void SpatialManipulator::setBaseTransform(const Eigen::Matrix4d& baseTransform)
	{
		m_rigidBodyChain.setBaseTransform(baseTransform);
	}

	// Set the physics scene.
	void SpatialManipulator::setPhysicsScene(PhysicsScene* physicsScene)
	{
		m_physicsScene = physicsScene;
	}

	// Set joint displacements.
	void SpatialManipulator::setJointDisplacements(const Eigen::VectorXd& jointDisplacements)
	{
		m_rigidBodyChain.setJointDisplacements(jointDisplacements);
		if (m_physicsScene)
		{
			m_physicsScene->generateContacts();
		}
		m_rigidBodyChain.updateContactJacobians();
	}

	// Get transform of end frame.
	Eigen::Matrix4d SpatialManipulator::getEndFrameSpatialTransform() const
	{
		return m_rigidBodyChain.getEndFrameSpatialTransform();
	}

	// Return the last rigid body of the chain.
	RigidBody SpatialManipulator::getEndFrame() const
	{
		return m_rigidBodyChain.getEndFrame();
	}

	// Get DoF/nMovableBodies of manipulator.
	int SpatialManipulator::getDof() const
	{
		return m_rigidBodyChain.getNMovableBodies();
	}

	// Get const reference to the rigid body chain.
	const RigidBodyChain& SpatialManipulator::getRigidBodyChain() const
	{
		return m_rigidBodyChain;
	}

	// Get the current joint displacements.
	Eigen::VectorXd SpatialManipulator::getJointDisplacements() const
	{
		return m_rigidBodyChain.getJointDisplacements();
	}

	// Generate motion plan.
	void SpatialManipulator::motionPlan(const Eigen::Matrix4d& goalTransform)
	{
		// Setup planner.
		MotionPlanner planner(this, goalTransform);

		// Generate plan.
		planner.computePlan();
	}
}
