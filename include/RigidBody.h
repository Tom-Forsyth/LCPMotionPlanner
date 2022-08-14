#pragma once

#include <Eigen/Dense>
#include "Joint.h"
#include "CollisionAggregate.h"
#include "Sphere.h"
#include "Capsule.h"
#include "Box.h"
#include "ContactPoint.h"


class RigidBody
{
private:
	Joint m_joint;
	Eigen::Matrix4d m_referenceSpatialTransform;
	CollisionAggregate m_collisionAggregate;
	Eigen::MatrixXd m_spatialJacobian;
	Eigen::Matrix4d m_currentSpatialTransform;
	Eigen::Matrix4d m_currentWorldTransform;
	ContactPoint m_contactPoint;
	Eigen::MatrixXd m_contactJacobian;

public:
	// Constructor.
	RigidBody(const Joint& joint, const Eigen::Matrix4d& referenceSpatialTransform);

	// Add colliders to collision aggregate member.
	void addCollider(const Sphere& sphere);
	void addCollider(const Capsule& capsule);
	void addCollider(const Box& box);

	// Get relative transformation from the joint.
	Eigen::Matrix4d getRelativeTransformation() const;

	// Get reference spatial transform.
	Eigen::Matrix4d getReferenceSpatialTransform() const;

	// Set spatial and world transformation.
	void setCurrentSpatialTransform(const Eigen::Matrix4d& spatialTransform);
	void setCurrentWorldTransform(const Eigen::Matrix4d& worldTransform);

	// Set joint displacement.
	void setJointDisplacement(const double& displacement);

	// Get reference to collision aggregate.
	const CollisionAggregate& getCollisionAggregate() const;

	// Get current world transform.
	Eigen::Matrix4d getCurrentWorldTransform() const;

	// Get joint type.
	Joint::Type getJointType() const;

	// Take closest contact in collision aggregate as the body's contact.
	void condenseContacts();

	// Get contact point.
	const ContactPoint& getContactPoint() const;

	// Update spatial jacobian.
	void updateSpatialJacobian();

	// Update

};

