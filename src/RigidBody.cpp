#include "RigidBody.h"
#include <Eigen/Dense>
#include "CollisionAggregate.h"
#include "Sphere.h"
#include "Capsule.h"
#include "Box.h"
#include <vector>
#include "Kinematics.h"

namespace CollisionAvoidance
{
	// Constructor.
	RigidBody::RigidBody(const Joint& joint, const Eigen::Matrix4d& referenceSpatialTransform, const std::string& name)
		: m_joint(joint), m_referenceSpatialTransform(referenceSpatialTransform), m_name(name), 
		m_collisionAggregate(CollisionAggregate()), m_currentSpatialTransform(Eigen::Matrix4d::Identity()), 
		m_currentWorldTransform(Eigen::Matrix4d::Identity()), m_contactPoint()
	{
		if (joint.getType() == JointType::Fixed)
		{
			m_isMovableBody = false;
		}
		else
		{
			m_isMovableBody = true;
		}

		// Set the parent body names of the colliders as this body's name.
		m_collisionAggregate.setParentBodyName(m_name);
	}

	// Add sphere collider.
	void RigidBody::addCollider(Sphere& sphere)
	{
		sphere.setParentBodyName(this->getName());
		m_collisionAggregate.addShape(sphere);
	}


	// Add capsule collider.
	void RigidBody::addCollider(Capsule& capsule)
	{
		capsule.setParentBodyName(this->getName());
		m_collisionAggregate.addShape(capsule);
	}

	// Add box collider.
	void RigidBody::addCollider(Box& box)
	{
		box.setParentBodyName(this->getName());
		m_collisionAggregate.addShape(box);
	}

	// Get relative transformation from the joint.
	Eigen::Matrix4d RigidBody::getRelativeTransformation() const
	{
		return m_joint.getRelativeTransformation();
	}

	// Get reference spatial transform.
	Eigen::Matrix4d RigidBody::getReferenceSpatialTransform() const
	{
		return m_referenceSpatialTransform;
	}

	// Set current spatial transform.
	void RigidBody::setCurrentSpatialTransform(const Eigen::Matrix4d& spatialTransform)
	{
		m_currentSpatialTransform = spatialTransform;
	}

	// Set current world transform.
	void RigidBody::setCurrentWorldTransform(const Eigen::Matrix4d& worldTransform)
	{
		m_currentWorldTransform = worldTransform;
	}

	// Set joint displacement.
	void RigidBody::setJointDisplacement(const double& displacement)
	{
		m_joint.setDisplacement(displacement);
	}

	// Get reference to collision aggregate.
	const CollisionAggregate& RigidBody::getCollisionAggregate() const
	{
		return m_collisionAggregate;
	}

	// Get current world transform.
	Eigen::Matrix4d RigidBody::getCurrentWorldTransform() const
	{
		return m_currentWorldTransform;
	}

	// Get joint type.
	JointType RigidBody::getJointType() const
	{
		return m_joint.getType();
	}

	// Get contact point.
	const ContactPoint& RigidBody::getContactPoint() const
	{
		return m_contactPoint;
	}

	// Update spatial jacobian.
	void RigidBody::setSpatialJacobian(const Eigen::MatrixXd& spatialJacobian)
	{
		m_spatialJacobian = spatialJacobian;
	}

	// Get joint twist coordinate.
	Eigen::Vector<double, 6> RigidBody::getJointTwistCoord() const
	{
		return m_joint.getTwistCoord();
	}

	// Determine if the body is movable.
	bool RigidBody::isMovable() const
	{
		return m_isMovableBody;
	}

	// Set the contact jacobian.
	void RigidBody::setContactJacobian(const Eigen::MatrixXd& contactJacobian)
	{
		m_contactJacobian = contactJacobian;
	}

	// Get the spatial jacobian.
	Eigen::MatrixXd RigidBody::getSpatialJacobian() const
	{
		return m_spatialJacobian;
	}

	// Get the contact jacobian.
	Eigen::MatrixXd RigidBody::getContactJacobian() const
	{
		return m_contactJacobian;
	}

	// Update the contact jacobian.
	void RigidBody::updateContactJacobian()
	{
		// Extract contact point in world frame and convert to spatial frame.
		const Eigen::Vector3d& contactPointWorld = m_contactPoint.m_point;
		Eigen::Vector4d contactPointWorldHomo{ contactPointWorld[0], contactPointWorld[1], contactPointWorld[2], 1 };
		Eigen::Vector3d contactPointSpatial = (m_currentSpatialTransform * contactPointWorldHomo).head(3);

		// Get analytic jacobian of the contact point (contact jacobian) using the point and this frames spatial jacobian.
		m_contactJacobian = Kinematics::spatialToAnalyticJacobian(m_spatialJacobian, contactPointSpatial);
	}

	// Get the current spatial transform.
	Eigen::Matrix4d RigidBody::getCurrentSpatialTransform() const
	{
		return m_currentSpatialTransform;
	}

	// Get joint displacement.
	double RigidBody::getJointDisplacement() const
	{
		return m_joint.getDisplacement();
	}

	void RigidBody::updateColliderTransforms()
	{
		m_collisionAggregate.updateColliderTransforms(m_currentWorldTransform);
	}

	void RigidBody::deactivateContactPoint()
	{
		m_contactPoint.m_isActive = false;
	}

	std::string RigidBody::getName() const
	{
		return m_name;
	}

	void RigidBody::setContactPoint(const ContactPoint& contactPoint)
	{
		m_contactPoint = contactPoint;
	}

}
