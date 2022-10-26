#include "SpatialManipulator.h"
#include "RigidBodyChain.h"
#include "LocalPlanner.h"
#include "PhysicsScene.h"
#include "ContactPoint.h"
#include <Eigen/Dense>
#include <vector>
#include <map>
#include <string>

namespace MotionPlanner
{
	SpatialManipulator::SpatialManipulator()
	{
		setBaseTransform(Eigen::Matrix4d::Identity());
	}

	SpatialManipulator::SpatialManipulator(const Eigen::Matrix4d& baseTransform)
	{
		setBaseTransform(baseTransform);
	}

	void SpatialManipulator::setBaseTransform(const Eigen::Matrix4d& baseTransform)
	{
		m_rigidBodyChain.setBaseTransform(baseTransform);
	}

	void SpatialManipulator::setPhysicsScene(PhysicsScene* physicsScene)
	{
		m_physicsScene = physicsScene;
	}

	void SpatialManipulator::setJointDisplacements(const Eigen::VectorXd& jointDisplacements)
	{
		m_rigidBodyChain.setJointDisplacements(jointDisplacements);
		if (m_physicsScene)
		{
			m_rigidBodyChain.deactivateContacts();
			const std::map<std::string, ContactPoint>& contactPoints = m_physicsScene->generateContacts();
			m_rigidBodyChain.updateContactPoints(contactPoints);
		}
		m_rigidBodyChain.updateContactJacobians();
	}

	Eigen::VectorXd SpatialManipulator::getJointDisplacements() const
	{
		return m_rigidBodyChain.getJointDisplacements();
	}

	Eigen::Matrix4d SpatialManipulator::getEndFrameSpatialTransform() const
	{
		return m_rigidBodyChain.getEndFrameSpatialTransform();
	}

	RigidBody SpatialManipulator::getEndFrame() const
	{
		return m_rigidBodyChain.getEndFrame();
	}

	int SpatialManipulator::getDof() const
	{
		return m_rigidBodyChain.getNMovableBodies();
	}

	const RigidBodyChain& SpatialManipulator::getRigidBodyChain() const
	{
		return m_rigidBodyChain;
	}

	std::vector<Eigen::VectorXd> SpatialManipulator::motionPlan(const Eigen::Matrix4d& goalTransform)
	{
		// Setup planner.
		LocalPlanner planner(this, goalTransform);

		// Generate plan.
		planner.computePlan();
		return planner.getPlan();
	}
}
