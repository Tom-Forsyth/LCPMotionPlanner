#pragma once

#include <vector>
#include "RigidBody.h"

class RigidBodyChain
{
private:
	std::vector<RigidBody> m_rigidBodies;
	int m_nBodies;
	Eigen::Matrix4d m_baseTransform;

public:
	// Default constructor.
	RigidBodyChain();

	// Rigid body vector constructor.
	RigidBodyChain(const std::vector<RigidBody>& rigidBodies);

	// Set base transform.
	void setBaseTransform(const Eigen::Matrix4d& baseTransform);

	// Add rigid body.
	void addBody(const RigidBody& rigidBody);

	// Forward kinematics.
	void forwardKinematics();

	// Set joint displacements.
	void setJointDisplacements(const std::vector<double>& jointDisplacements);

	// Get reference to rigid bodies.
	const std::vector<RigidBody>& getRigidBodies() const;

	// Get number of bodies.
	int getNBodies() const;

	// Set maximum number of bodies to reserve block of memory.
	void setMaxBodies(size_t nBodies);

	// Take closest contact of all the collision aggregates to be this bodies contact.
	void condenseContacts();

	// Update contact jacobians for the active contacts.
	void updateContactJacobians();


};