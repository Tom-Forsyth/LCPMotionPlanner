#include "RigidBodyChain.h"
#include "RigidBody.h"
#include "Joint.h"
#include "Kinematics.h"
#include <vector>
#include <Eigen/Dense>

namespace MotionPlanner
{
	RigidBodyChain::RigidBodyChain()
		: m_rigidBodies(std::vector<RigidBody> {}), m_baseTransform(Eigen::Matrix4d::Identity()),
		m_nBodies(0), m_nMovableBodies(0) { }

	void RigidBodyChain::setBaseTransform(const Eigen::Matrix4d& baseTransform)
	{
		m_baseTransform = baseTransform;
		forwardKinematics();
	}

	Eigen::Matrix4d RigidBodyChain::getBaseTransform() const
	{
		return m_baseTransform;
	}

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

	bool RigidBodyChain::setJointDisplacements(const Eigen::VectorXd& jointDisplacements)
	{
		// Set displacement for each body's joint.
		int displacementIndex = 0;
		for (RigidBody& body : m_rigidBodies)
		{
			// Attempt to set displacement for the joint.
			bool withinJointLimits = true;
			if (body.getJointType() != JointType::Fixed)
			{
				withinJointLimits = body.setJointDisplacement(jointDisplacements[displacementIndex]);
				displacementIndex++;
			}

			// If there is a limit violation, exit and return a failure.
			if (!withinJointLimits)
			{
				return false;
			}
		}

		// Update spatial and world transforms with new joint displacement.
		forwardKinematics();

		// Update spatial jacobians.
		updateSpatialJacobians();

		// Update the world transforms of the collision primatives.
		updateColliderTransforms();

		return true;
	}

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

	const std::vector<RigidBody>& RigidBodyChain::getRigidBodies() const
	{
		return m_rigidBodies;
	}

	size_t RigidBodyChain::getNBodies() const
	{
		return m_nBodies;
	}

	size_t RigidBodyChain::getNMovableBodies() const
	{
		return m_nMovableBodies;
	}

	Eigen::Matrix4d RigidBodyChain::getEndFrameSpatialTransform() const
	{
		return m_rigidBodies[m_nBodies - 1].getSpatialTransform();
	}

	RigidBody RigidBodyChain::getEndFrame() const
	{
		return m_rigidBodies[m_nBodies - 1];
	}

	void RigidBodyChain::deactivateContacts()
	{
		for (RigidBody& rigidBody : m_rigidBodies)
		{
			rigidBody.deactivateContactPoint();
		}
	}

	void RigidBodyChain::updateContactPoints(const std::map<std::string, ContactPoint>& contactPoints)
	{
		int rigidBodyIndex = 0;
		for (const std::pair<std::string, ContactPoint> contactPoint : contactPoints)
		{
			// Advance location in rigid body chain until we find the body of the contact point or hit a the max.
			std::string rigidBodyName = m_rigidBodies[rigidBodyIndex].getName();
			while ((contactPoint.first != rigidBodyName) && (rigidBodyIndex < m_nBodies))
			{
				rigidBodyIndex++;
				rigidBodyName = m_rigidBodies[rigidBodyIndex].getName();
			}

			// If we have not hit the max, we can assign the contact point.
			//if (rigidBodyIndex != m_nBodies - 1)
			//{
				m_rigidBodies[rigidBodyIndex].setContactPoint(contactPoint.second);
			//}
		}
	}

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
			body.setSpatialTransform(spatialTransform);
			body.setWorldTransform(worldTransform);
		}
	}

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

	void RigidBodyChain::updateColliderTransforms()
	{
		for (RigidBody& rigidBody : m_rigidBodies)
		{
			rigidBody.updateColliderTransforms();
		}
	}
}
