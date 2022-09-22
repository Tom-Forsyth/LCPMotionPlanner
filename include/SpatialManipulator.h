#pragma once

#include "RigidBodyChain.h"
#include <Eigen/Dense>

namespace CollisionAvoidance
{
	class PhysicsScene;

	class SpatialManipulator
	{
	protected:
		RigidBodyChain m_rigidBodyChain;
		PhysicsScene* m_physicsScene = nullptr;

	public:
		// Constructors.
		SpatialManipulator();
		SpatialManipulator(const Eigen::Matrix4d& baseTransform);

		// Set base transform.
		void setBaseTransform(const Eigen::Matrix4d& baseTransform);

		// Set the physics scene.
		void setPhysicsScene(PhysicsScene* physicsScene);

		// Set joint displacements.
		void setJointDisplacements(const Eigen::VectorXd& jointDisplacements);

		// Get transform of end frame.
		Eigen::Matrix4d getEndFrameSpatialTransform() const;

		// Return the last rigid body of the chain.
		RigidBody getEndFrame() const;

		// Get DoF/nMovableBodies of manipulator.
		int getDof() const;

		// Get const reference to the rigid body chain.
		const RigidBodyChain& getRigidBodyChain() const;

		// Get the current joint displacements.
		Eigen::VectorXd getJointDisplacements() const;

		// Generate motion plan.
		void motionPlan(const Eigen::Matrix4d& goalTransform);
	};
}
