#pragma once

#include "RigidBodyChain.h"
#include <Eigen/Dense>
#include <vector>

namespace MotionPlanner
{
	class PhysicsScene;

	/// @brief Base interface for a manipulator to create motion plans.
	class SpatialManipulator
	{
	protected:
		/// @brief Kinematic chain of the manipulator.
		RigidBodyChain m_rigidBodyChain;

		/// @brief Physics scene that the manipulator belongs to.
		PhysicsScene* m_physicsScene = nullptr;

	public:
		/// @brief Default constructor.
		SpatialManipulator();

		/// @brief Constructor at a location.
		/// @param baseTransform Pose of base of robot.
		SpatialManipulator(const Eigen::Matrix4d& baseTransform);

		/// @brief Set the base transform of the robot.
		/// @param baseTransform Robot base transform.
		void setBaseTransform(const Eigen::Matrix4d& baseTransform);

		/// @brief Set the physics scene pointer.
		/// @param physicsScene Pointer to physics scene.
		void setPhysicsScene(PhysicsScene* physicsScene);

		/// @brief Set the joint displacements and compute forward kinematics.
		/// @param jointDisplacements Joint displacements.
		void setJointDisplacements(const Eigen::VectorXd& jointDisplacements);

		/// @brief Get the spatial transform of the end effector.
		/// @return End effector spatial transform.
		Eigen::Matrix4d getEndFrameSpatialTransform() const;

		/// @brief Get the last rigid body in the kinematic chain.
		/// @return Last rigid body in kinematic chain.
		RigidBody getEndFrame() const;

		/// @brief Get the DoF of the manipulator.
		/// @return DoF.
		int getDof() const;

		/// @brief Get the rigid body chain.
		/// @return Rigid body chain.
		const RigidBodyChain& getRigidBodyChain() const;

		/// @brief Get the current joint displacements.
		/// @return Joint displacements.
		Eigen::VectorXd getJointDisplacements() const;

		/// @brief Generate a motion plan to the specfied pose.
		/// @param goalTransform Goal pose.
		/// @return Sequence of joint displacements.
		std::vector<Eigen::VectorXd> motionPlan(const Eigen::Matrix4d& goalTransform);
	};
}
