#pragma once

#include <Eigen/Dense>
#include "Joint.h"
#include "CollisionAggregate.h"
#include "Sphere.h"
#include "Capsule.h"
#include "Box.h"
#include "ContactPoint.h"
#include <string>

namespace MotionPlanner
{
	/// @brief Rigid body representing a robot link containing transforms, Joint, jacobains, and contact point.
	class RigidBody
	{
	private:
		/// @brief Joint.
		Joint m_joint;

		/// @brief Spatial transform when all joint displacements of the robot are 0.
		Eigen::Matrix4d m_referenceSpatialTransform = Eigen::Matrix4d::Identity();

		/// @brief Aggregate of shape actors for collision of this link.
		CollisionAggregate m_collisionAggregate;

		/// @brief Spatial jacobian of the link.
		Eigen::MatrixXd m_spatialJacobian;

		/// @brief Spatial transform at the current joint state of the robot.
		Eigen::Matrix4d m_spatialTransform = Eigen::Matrix4d::Identity();

		/// @brief World transform at the current joint state of the robot.
		Eigen::Matrix4d m_worldTransform = Eigen::Matrix4d::Identity();

		/// @brief Closest contact point between this link and the environment.
		ContactPoint m_contactPoint;

		/// @brief Analytic jacobian at the contact point.
		Eigen::MatrixXd m_contactJacobian;

		/// @brief Flag to determine if the joint of the body is fixed or not.
		bool m_isMovableBody;
		
		/// @brief Name of the link.
		std::string m_name;

	public:
		/// @brief Constructor.
		/// @param joint Joint.
		/// @param referenceSpatialTransform Spatial transform when all joint displacements of the robot are 0.
		/// @param name Name of link.
		RigidBody(const Joint& joint, const Eigen::Matrix4d& referenceSpatialTransform, const std::string& name);

		/// @brief Get the name of the link.
		/// @return Name.
		std::string getName() const;

		/// @brief Add shape actor to the body's collision aggregate.
		/// Shape transforms should be local to the body's origin.
		/// Once added, the shape's transform will be updated to the correct world transform based on the body's transform.
		/// @param shape Shape actor.
		void addCollider(Shape& shape);

		/// @brief Update the transforms of the collision shape actors based on the link's transform.
		void updateColliderTransforms();

		/// @brief Get the collision aggregate.
		/// @return Collision aggregate.
		const CollisionAggregate& getCollisionAggregate() const;

		/// @brief Get the joint's relative transformation/displacement.
		/// @return Relative transformation.
		Eigen::Matrix4d getRelativeTransformation() const;

		/// @brief Get the reference spatial transform.
		/// @return Reference spatial transform.
		Eigen::Matrix4d getReferenceSpatialTransform() const;

		/// @brief Get the spatial transform of the link.
		/// @return Spatial transform.
		Eigen::Matrix4d getSpatialTransform() const;

		/// @brief Set the spatial transformation of the link.
		/// @param spatialTransform Spatial transformation.
		void setSpatialTransform(const Eigen::Matrix4d& spatialTransform);

		/// @brief Get the current world transformation.
		/// @return World transformation.
		Eigen::Matrix4d getWorldTransform() const;

		/// @brief Set the world transformation of the link.
		/// @param worldTransform World transformation.
		void setWorldTransform(const Eigen::Matrix4d& worldTransform);

		/// @brief Get the joint's displacement.
		/// @return Joint displacement.
		double getJointDisplacement() const;

		/// @brief Set the displacement of the joint.
		/// @param displacement Displacement.
		/// @return If setting the joint displacement was successful.
		bool setJointDisplacement(const double& displacement);

		/// @brief Get the type of the joint.
		/// @return Joint type.
		/// @bug Redundant. See RigidBody::isMovable().
		JointType getJointType() const;

		/// @brief Determine if the link's joint is movable (IE: Revolute or Prismatic).
		/// @return True if the joint is movable.
		bool isMovable() const;

		/// @brief Get the twist coordinate of the joint.
		/// @return Twist coordinate.
		Eigen::Vector<double, 6> getJointTwistCoord() const;

		/// @brief Get the closest contact point of the link to the environment.
		/// @return Contact point.
		const ContactPoint& getContactPoint() const;

		/// @brief Set the contact point of the link.
		/// @param contactPoint Contact point.
		void setContactPoint(const ContactPoint& contactPoint);

		/// @brief Deactivate the link's contact point.
		void deactivateContactPoint();

		/// @brief Get the contact jacobian of the link.
		/// @return Contact jacobian.
		Eigen::MatrixXd getContactJacobian() const;

		/// @brief Set the contact jacobian of the link.
		/// Used for intialization as dimension of matrix is determined on full construction of robot.
		/// @param contactJacobian Contact jacobian.
		void setContactJacobian(const Eigen::MatrixXd& contactJacobian);

		/// @brief Update the contact jacobian based on the current contact point of the link.
		/// @bug Should check if the contact point is active before computing the new contact jacobian.
		/// @bug Should have this method called when a contact point is set, not in rigid body chain.
		void updateContactJacobian();

		/// @brief Get the spatial jacobian of the link.
		/// @return Spatial jacobian.
		Eigen::MatrixXd getSpatialJacobian() const;

		/// @brief Set the spatial jacobian of link.
		/// @param spatialJacobian Spatial jacobian.
		void setSpatialJacobian(const Eigen::MatrixXd& spatialJacobian);
	};
}
