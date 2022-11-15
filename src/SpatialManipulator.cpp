#include "SpatialManipulator.h"
#include "RigidBodyChain.h"
#include "LocalPlanner.h"
#include "PhysicsScene.h"
#include "ContactPoint.h"
#include "GlobalPlanner.h"
#include "MotionPlanResults.h"
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

	Eigen::Matrix4d SpatialManipulator::getBaseTransform() const
	{
		return m_rigidBodyChain.getBaseTransform();
	}

	void SpatialManipulator::setPhysicsScene(PhysicsScene* physicsScene)
	{
		m_physicsScene = physicsScene;
	}

	bool SpatialManipulator::setJointDisplacements(const Eigen::VectorXd& jointDisplacements)
	{
		// If the joint displacements are valid and not violating joint limits, update physics and kinematics.
		bool validDisplacements = m_rigidBodyChain.setJointDisplacements(jointDisplacements);
		if (validDisplacements)
		{
			if (m_physicsScene)
			{
				m_rigidBodyChain.deactivateContacts();
				const std::map<std::string, ContactPoint>& contactPoints = m_physicsScene->generateContacts();
				m_rigidBodyChain.updateContactPoints(contactPoints);
			}
			m_rigidBodyChain.updateContactJacobians();
		}
		return validDisplacements;
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

	void SpatialManipulator::setMaxReach(double maxReach)
	{
		m_maxReach = maxReach;
	}

	double SpatialManipulator::getMaxReach() const
	{
		return m_maxReach;
	}

	bool SpatialManipulator::isColliding() const
	{
		for (const auto& body : getRigidBodyChain().getRigidBodies())
		{
			const ContactPoint& contactPoint = body.getContactPoint();
			if (contactPoint.m_distance <= 0 && contactPoint.m_isActive)
			{
				return true;
			}
		}
		return false;
	}

	std::vector<std::pair<double, double>> SpatialManipulator::getJointLimits() const
	{
		return m_rigidBodyChain.getJointLimits();
	}

	MotionPlanResults SpatialManipulator::motionPlan(const Eigen::Matrix4d& goalTransform)
	{
		// Intialize global planner.
		GlobalPlanner globalPlanner(this, goalTransform);

		// Generate plan.
		globalPlanner.computePlan();
		return globalPlanner.getPlanResults();
	}
}
