#include "RigidBodyChain.h"
#include "RigidBody.h"
#include "Joint.h"
#include "Kinematics.h"
#include <vector>
#include <Eigen/Dense>

namespace CollisionAvoidance
{
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
		if (rigidBody.getJointType() != JointType::Fixed)
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
		updateColliderTransforms();

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
	void RigidBodyChain::setJointDisplacements(const Eigen::VectorXd& jointDisplacements)
	{
		// Set displacement for each body's joint.
		int i = 0;
		for (RigidBody& body : m_rigidBodies)
		{
			if (body.getJointType() != JointType::Fixed)
			{
				body.setJointDisplacement(jointDisplacements[i]);
				i++;
			}
		}

		// Update spatial and world transforms with new joint displacement.
		forwardKinematics();

		// Update spatial jacobians.
		updateSpatialJacobians();

		// Update the world transforms of the collision primatives.
		updateColliderTransforms();
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
		// Loop over each body, adding columns for movable joints and updating the member.
		Eigen::MatrixXd spatialJacobian = Eigen::MatrixXd::Zero(6, m_nMovableBodies);
		Eigen::Matrix4d displacementProduct = Eigen::Matrix4d::Identity();
		int bodyIndex = 0;
		for (RigidBody& body : m_rigidBodies)
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

			// Set the body's spatial jacobian with zero padding.
			body.setSpatialJacobian(spatialJacobian);
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
				body.updateContactJacobian();
			}
		}
	}

	// Get end frame transform.
	Eigen::Matrix4d RigidBodyChain::getEndFrameSpatialTransform() const
	{
		return m_rigidBodies[m_nBodies - 1].getCurrentSpatialTransform();
	}

	// Get end frame.
	RigidBody RigidBodyChain::getEndFrame() const
	{
		return m_rigidBodies[m_nBodies - 1];
	}

	// Get the current joint displacements.
	Eigen::VectorXd RigidBodyChain::getJointDisplacements() const
	{
		Eigen::VectorXd jointDisplacements = Eigen::VectorXd::Zero(m_nMovableBodies);
		int index = 0;
		for (const RigidBody& body : m_rigidBodies)
		{
			if (body.isMovable())
			{
				jointDisplacements[index] = body.getJointDisplacement();
				index++;
			}
		}

		return jointDisplacements;
	}

	void RigidBodyChain::updateColliderTransforms()
	{
		for (RigidBody& rigidBody : m_rigidBodies)
		{
			rigidBody.updateColliderTransforms();
		}
	}
}
