#pragma once

#include <vector>
#include "RigidBody.h"

namespace CollisionAvoidance
{
	class RigidBodyChain
	{
	private:
		std::vector<RigidBody> m_rigidBodies;
		Eigen::Matrix4d m_baseTransform;
		size_t m_nBodies;
		size_t m_nMovableBodies;

	public:
		// Default constructor.
		RigidBodyChain();

		// Rigid body vector constructor.
		RigidBodyChain(const std::vector<RigidBody>& rigidBodies);

		// Set base transform.
		void setBaseTransform(const Eigen::Matrix4d& baseTransform);

		// Add rigid body.
		void addBody(const RigidBody& rigidBody);

		// Called to finish initialization once chain is created.
		void postInit();

		// Forward kinematics.
		void forwardKinematics();

		// Set joint displacements.
		void setJointDisplacements(const Eigen::VectorXd& jointDisplacements);

		// Get reference to rigid bodies.
		const std::vector<RigidBody>& getRigidBodies() const;

		// Get number of total bodies.
		size_t getNBodies() const;

		// Get number of movable bodies.
		size_t getNMovableBodies() const;

		// Take closest contact of all the collision aggregates to be this bodies contact.
		void condenseContacts();

		// Update spatial jacobian for each body.
		void updateSpatialJacobians();

		// Update contact jacobians for the active contacts.
		void updateContactJacobians();

		// Get end frame transform.
		Eigen::Matrix4d getEndFrameSpatialTransform() const;

		// Get end frame.
		RigidBody getEndFrame() const;

		// Get the current joint displacements.
		Eigen::VectorXd getJointDisplacements() const;
	};
}
