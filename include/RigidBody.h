#pragma once

#include <Eigen/Dense>
#include "Joint.h"
#include "CollisionAggregate.h"
#include "Sphere.h"
#include "Capsule.h"
#include "Box.h"
#include "ContactPoint.h"
#include <string>

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
		
		// Name of the rigid body.
		std::string m_name;

	public:
		// Constructor.
		RigidBody(const Joint& joint, const Eigen::Matrix4d& referenceSpatialTransform, const std::string& name);

		// Add colliders to collision aggregate member.
		void addCollider(Sphere& sphere);
		void addCollider(Capsule& capsule);
		void addCollider(Box& box);

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

		// Deactivate the contact point.
		void deactivateContactPoint();

		// Get name.
		std::string getName() const;

		// Set contact point.
		void setContactPoint(const ContactPoint& contactPoint);
	};
}
