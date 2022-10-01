#pragma once

#include <vector>
#include "RigidBody.h"

namespace MotionPlanner
{
	/// @brief Kinematic chain to represent a spatial maniulator.
	class RigidBodyChain
	{
	private:
		/// @brief Rigid bodies/links in the chain.
		std::vector<RigidBody> m_rigidBodies;

		/// @brief World transform of the base of the manipulator.
		/// Spatial base transform is always identity, and kinematics are done in spatial frame.
		/// This world transform is used to get the world transforms of the robot components after forward kinematics.
		Eigen::Matrix4d m_baseTransform;

		/// @brief Number of bodies in the kinematic chain.
		size_t m_nBodies;

		/// @brief Number of movable bodies in the kinematic chain. IE: Joint is not fixed.
		size_t m_nMovableBodies;

		/// @brief Compute the forward kinematics of the chain.
		void forwardKinematics();

		/// @brief Update the spatial jacobian for each body.
		void updateSpatialJacobians();



		/// @brief Update the transforms of the collision actors in each body.
		/// @bug Delete this and move functionality to the rigid body.
		void updateColliderTransforms();



	public:
		/// @brief Constructor.
		RigidBodyChain();

		/// @brief Set the base transform of the kinematic chain.
		/// @param baseTransform Base transform.
		void setBaseTransform(const Eigen::Matrix4d& baseTransform);

		/// @brief Add a link.body to the kinematic chain.
		/// @param rigidBody Robot link.
		void addBody(const RigidBody& rigidBody);

		/// @brief Set the joint displacements of the chain.
		/// @param jointDisplacements Joint displacements.
		void setJointDisplacements(const Eigen::VectorXd& jointDisplacements);

		/// @brief Get a vector of rigid bodies in the chain.
		/// @return Vector of rigid bodies.
		const std::vector<RigidBody>& getRigidBodies() const;

		/// @brief Get the number of bodies in the chain.
		/// @return Number of bodies.
		size_t getNBodies() const;

		/// @brief Get the number of bodies in the chain that do not have a fixed joint.
		/// @return Number of movable bodies in chain.
		size_t getNMovableBodies() const;

		/// @brief Get the spatial transform of the end-effector.
		/// @return End effector spatial transform.
		Eigen::Matrix4d getEndFrameSpatialTransform() const;

		/// @brief Get the end effector body.
		/// @return End effector.
		RigidBody getEndFrame() const;

		/// @brief Get the joint displacements of the kinematic chain.
		/// @return Joint displacements.
		Eigen::VectorXd getJointDisplacements() const;

		/// @brief Deactivate the contacts in each rigid body.
		/// @bug Move functionality to the rigid body.
		void deactivateContacts();

		/// @brief Update the contact points of each body in the chain.
		/// @param contactPoints Contact point and body name pairs.
		void updateContactPoints(const std::map<std::string, ContactPoint>& contactPoints);
		
		/// @brief Update contact jacobians for each body.
		/// @bug Delete this, and move functionality to RigidBody::setContactPoint().
		void updateContactJacobians();

		/// @brief Perform post initialization tasks such as determining jacobian dimensions and forward kinematics.
		void postInit();
	};
}
