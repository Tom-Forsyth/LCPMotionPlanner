#include "RigidBodyChain.h"
#include "RigidBody.h"
#include "Joint.h"
#include <vector>
#include <Eigen/Dense>
#include <iostream>

// Default constructor.
RigidBodyChain::RigidBodyChain()
	: m_rigidBodies(std::vector<RigidBody> {}), m_nBodies(0),
	m_baseTransform(Eigen::Matrix4d::Identity()) { }

// Rigid body vector constructor.
RigidBodyChain::RigidBodyChain(const std::vector<RigidBody>& rigidBodies)
	: m_rigidBodies(rigidBodies), m_nBodies(0),
	m_baseTransform(Eigen::Matrix4d::Identity()) { }

// Set base transform.
void RigidBodyChain::setBaseTransform(const Eigen::Matrix4d& baseTransform)
{
	m_baseTransform = baseTransform;
	forwardKinematics();
}

// Add rigid body.
void RigidBodyChain::addBody(const RigidBody& rigidBody)
{
	m_rigidBodies.emplace_back(rigidBody);
	m_nBodies++;
}

// Forward kinematics.
void RigidBodyChain::forwardKinematics()
{
	// Keep running product of displacements rather than storing each.
	Eigen::Matrix4d displacementProduct = Eigen::Matrix4d::Identity();

	// Loop over each body in the chain.
	for (RigidBody& body : m_rigidBodies)
	{
		displacementProduct *= body.getRelativeTransformation();
		Eigen::Matrix4d spatialTransform = displacementProduct * body.getReferenceSpatialTransform();
		Eigen::Matrix4d worldTransform = m_baseTransform * spatialTransform;
		body.setCurrentSpatialTransform(spatialTransform);
		body.setCurrentWorldTransform(worldTransform);
	}
}

// Set joint displacements.
void RigidBodyChain::setJointDisplacements(const std::vector<double>& jointDisplacements)
{
	// Set displacement for each body's joint.
	int i = 0;
	for (RigidBody& body : m_rigidBodies)
	{
		if (body.getJointType() != Joint::FIXED)
		{
			body.setJointDisplacement(jointDisplacements[i]);
			i++;
		}
	}

	// Update spatial and world transforms with new joint displacement.
	forwardKinematics();
}

// Get reference to rigid bodies.
const std::vector<RigidBody>& RigidBodyChain::getRigidBodies() const
{
	return m_rigidBodies;
}

// Get number of bodies.
int RigidBodyChain::getNBodies() const
{
	return m_nBodies;
}

// Set maximum number of bodies to reserve block of memory.
void RigidBodyChain::setMaxBodies(size_t nBodies)
{
	m_rigidBodies.reserve(nBodies);
}

// Take closest contact of all the collision aggregates to be this bodies contact.
void RigidBodyChain::condenseContacts()
{

	for (RigidBody& body : m_rigidBodies)
	{
		body.condenseContacts();
	}
}

// Update contact jacobians for the active contacts.
void RigidBodyChain::updateContactJacobians()
{
	for (RigidBody& body : m_rigidBodies)
	{
		const ContactPoint& contactPoint = body.getContactPoint();
		if (contactPoint.m_isActive)
		{

		}
	}
}
