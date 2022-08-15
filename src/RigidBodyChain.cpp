#include "RigidBodyChain.h"
#include "RigidBody.h"
#include "Joint.h"
#include "Kinematics.h"
#include <vector>
#include <Eigen/Dense>
#include <iostream>

// Default constructor.
RigidBodyChain::RigidBodyChain()
	: m_rigidBodies(std::vector<RigidBody> {}), m_baseTransform(Eigen::Matrix4d::Identity()),
	m_nBodies(0), m_nMovableBodies(0) { }

// Rigid body vector constructor.
RigidBodyChain::RigidBodyChain(const std::vector<RigidBody>& rigidBodies)
	: m_rigidBodies(rigidBodies), m_baseTransform(Eigen::Matrix4d::Identity()),
	m_nBodies(0), m_nMovableBodies(0) { }

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

	// Increment body sizes.
	m_nBodies++;
	if (rigidBody.getJointType() != Joint::FIXED)
	{
		m_nMovableBodies++;
	}
}

// Called to finish initialization once chain is created.
void RigidBodyChain::postInit()
{
	// Initialize jacobians with zero matrices.
	for (RigidBody& body : m_rigidBodies)
	{
		body.setSpatialJacobian(Eigen::MatrixXd::Zero(6, m_nMovableBodies));
		body.setContactJacobian(Eigen::MatrixXd::Zero(6, m_nMovableBodies));
	}

	// Update transforms, jacobians, etc.
	forwardKinematics();
	updateSpatialJacobians();
	
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

	// Update spatial jacobians.
	updateSpatialJacobians();
}

// Get reference to rigid bodies.
const std::vector<RigidBody>& RigidBodyChain::getRigidBodies() const
{
	return m_rigidBodies;
}

// Get number of bodies.
size_t RigidBodyChain::getNBodies() const
{
	return m_nBodies;
}

// Get number of movable bodies.
size_t RigidBodyChain::getNMovableBodies() const
{
	return m_nMovableBodies;
}

// Take closest contact of all the collision aggregates to be this bodies contact.
void RigidBodyChain::condenseContacts()
{

	for (RigidBody& body : m_rigidBodies)
	{
		body.condenseContacts();
	}
}

// Update spatial jacobian for each body.
void RigidBodyChain::updateSpatialJacobians()
{
	// First, we compute the end-frame spatial jacobian.
	Eigen::MatrixXd spatialJacobian = Eigen::MatrixXd::Zero(6, m_nMovableBodies);
	Eigen::Matrix4d displacementProduct = Eigen::Matrix4d::Identity();
	int bodyIndex = 0;
	for (const RigidBody& body : m_rigidBodies)
	{
		if (body.isMovable())
		{
			// Compute column of jacobian.
			Eigen::Vector<double, 6> twistCoord = body.getJointTwistCoord();
			if (bodyIndex == 0)
			{
				spatialJacobian.col(bodyIndex) = twistCoord;
			}
			else
			{
				spatialJacobian.col(bodyIndex) = Kinematics::adjoint(displacementProduct) * twistCoord;
			}

			// Compute next iteration's product of i-1.
			displacementProduct *= body.getRelativeTransformation();
			bodyIndex++;
		}
	}

	// Set the jacobian of the last frame.
	m_rigidBodies[m_nBodies - 1].setSpatialJacobian(spatialJacobian);

	// Now, we can set the jacobian of the other bodies by zero padding backwards.
	bodyIndex = static_cast<int>(m_nBodies - 2);
	int columnIndex = static_cast<int>(m_nMovableBodies - 1);
	while (bodyIndex >= 0)
	{
		if (m_rigidBodies[bodyIndex].isMovable())
		{
			spatialJacobian.col(columnIndex) = Eigen::Vector<double, 6>::Zero();
			m_rigidBodies[bodyIndex].setSpatialJacobian(spatialJacobian);
			columnIndex--;
		}
		else
		{
		}
		bodyIndex--;
	}

	// Check my logic is correct. Will remove this after I write tests.
	assert(bodyIndex == -1);
	assert(columnIndex == -1);
}

// Update contact jacobians for the active contacts.
void RigidBodyChain::updateContactJacobians()
{
	for (RigidBody& body : m_rigidBodies)
	{
		std::cout << body.getSpatialJacobian() << std::endl << std::endl;
		const ContactPoint& contactPoint = body.getContactPoint();
		if (contactPoint.m_isActive)
		{
			body.updateContactJacobian();
		}
	}
}
