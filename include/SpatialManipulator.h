#pragma once

#include "RigidBodyChain.h"
#include <Eigen/Dense>

namespace CollisionAvoidance
{
	class SpatialManipulator
	{
	protected:
		RigidBodyChain m_rigidBodyChain;

	public:
		// Constructors.
		SpatialManipulator();
		SpatialManipulator(const Eigen::Matrix4d& baseTransform);

		// Set base transform.
		void setBaseTransform(const Eigen::Matrix4d& baseTransform);

		// Set joint displacements.
		void setJointDisplacements(const Eigen::VectorXd& jointDisplacements);

		// Setup simulation environment.
		void setupSimulationEnvironment();

		// Run simulation to generate contacts.
		void generateContacts();

		// Generate motion plan.
		void motionPlan(const Eigen::Matrix4d& goalTransform);

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
	};
}
