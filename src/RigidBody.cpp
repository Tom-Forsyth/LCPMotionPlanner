#include "RigidBody.h"
#include <Eigen/Dense>
#include "CollisionAggregate.h"
#include "Sphere.h"
#include "Capsule.h"
#include "Box.h"
#include <vector>
#include "Kinematics.h"

namespace MotionPlanner
{
	RigidBody::RigidBody(const Joint& joint, const Eigen::Matrix4d& referenceSpatialTransform, const std::string& name)
		: m_joint(joint), m_referenceSpatialTransform(referenceSpatialTransform), m_name(name)
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

	std::string RigidBody::getName() const
	{
		return m_name;
	}

	void RigidBody::addCollider(Shape& shape)
	{
		shape.setParentBodyName(getName());
		m_collisionAggregate.addShape(shape);
	}

	void RigidBody::updateColliderTransforms()
	{
		m_collisionAggregate.updateColliderTransforms(m_worldTransform);
	}

	const CollisionAggregate& RigidBody::getCollisionAggregate() const
	{
		return m_collisionAggregate;
	}

	Eigen::Matrix4d RigidBody::getRelativeTransformation() const
	{
		return m_joint.getRelativeTransformation();
	}

	Eigen::Matrix4d RigidBody::getReferenceSpatialTransform() const
	{
		return m_referenceSpatialTransform;
	}

	Eigen::Matrix4d RigidBody::getSpatialTransform() const
	{
		return m_spatialTransform;
	}

	void RigidBody::setSpatialTransform(const Eigen::Matrix4d& spatialTransform)
	{
		m_spatialTransform = spatialTransform;
	}

	Eigen::Matrix4d RigidBody::getWorldTransform() const
	{
		return m_worldTransform;
	}

	void RigidBody::setWorldTransform(const Eigen::Matrix4d& worldTransform)
	{
		m_worldTransform = worldTransform;
	}

	double RigidBody::getJointDisplacement() const
	{
		return m_joint.getDisplacement();
	}

	bool RigidBody::setJointDisplacement(const double& displacement)
	{
		return m_joint.setDisplacement(displacement);
	}

	JointType RigidBody::getJointType() const
	{
		return m_joint.getType();
	}

	bool RigidBody::isMovable() const
	{
		return m_isMovableBody;
	}

	Eigen::Vector<double, 6> RigidBody::getJointTwistCoord() const
	{
		return m_joint.getTwistCoord();
	}

	const ContactPoint& RigidBody::getContactPoint() const
	{
		return m_contactPoint;
	}

	void RigidBody::setContactPoint(const ContactPoint& contactPoint)
	{
		m_contactPoint = contactPoint;
	}

	void RigidBody::deactivateContactPoint()
	{
		m_contactPoint.m_isActive = false;
	}

	Eigen::MatrixXd RigidBody::getContactJacobian() const
	{
		return m_contactJacobian;
	}

	void RigidBody::setContactJacobian(const Eigen::MatrixXd& contactJacobian)
	{
		m_contactJacobian = contactJacobian;
	}

	void RigidBody::updateContactJacobian()
	{
		// Extract contact point in world frame and convert to spatial frame.
		const Eigen::Vector3d& contactPointWorld = m_contactPoint.m_point;
		Eigen::Vector4d contactPointWorldHomo{ contactPointWorld[0], contactPointWorld[1], contactPointWorld[2], 1 };
		Eigen::Vector3d contactPointSpatial = (m_spatialTransform * contactPointWorldHomo).head(3);

		// Get analytic jacobian of the contact point (contact jacobian) using the point and this frames spatial jacobian.
		m_contactJacobian = Kinematics::spatialToAnalyticJacobian(m_spatialJacobian, contactPointSpatial);
	}

	Eigen::MatrixXd RigidBody::getSpatialJacobian() const
	{
		return m_spatialJacobian;
	}

	void RigidBody::setSpatialJacobian(const Eigen::MatrixXd& spatialJacobian)
	{
		m_spatialJacobian = spatialJacobian;
	}
}
