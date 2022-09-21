#pragma once

#include <Eigen/Dense>
#include "Joint.h"
#include "CollisionAggregate.h"
#include "Sphere.h"
#include "Capsule.h"
#include "Box.h"
#include "ContactPoint.h"

namespace CollisionAvoidance
{
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
		bool m_isMovableBody;

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
		JointType getJointType() const;

		// Take closest contact in collision aggregate as the body's contact.
		void condenseContacts();

		// Get contact point.
		const ContactPoint& getContactPoint() const;

		// Update spatial jacobian.
		void setSpatialJacobian(const Eigen::MatrixXd& spatialJacobian);

		// Get joint twist coordinate.
		Eigen::Vector<double, 6> getJointTwistCoord() const;

		// Determine if the body is movable.
		bool isMovable() const;

		// Set the contact jacobian.
		void setContactJacobian(const Eigen::MatrixXd& contactJacobian);

		// Get the spatial jacobian.
		Eigen::MatrixXd getSpatialJacobian() const;

		// Get the contact jacobian.
		Eigen::MatrixXd getContactJacobian() const;

		// Update the contact jacobian.
		void updateContactJacobian();

		// Get the current spatial transform.
		Eigen::Matrix4d getCurrentSpatialTransform() const;

		// Get joint displacement.
		double getJointDisplacement() const;

		// Update the transforms of the collision actors.
		void updateColliderTransforms();
	};
}
