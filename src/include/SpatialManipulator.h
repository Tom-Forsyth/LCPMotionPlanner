#pragma once

#include "RigidBodyChain.h"
#include "MotionPlanResults.h"
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

		/// @brief Maximum reach of the manipulator.
		double m_maxReach = 0;

	public:
		/// @brief Default constructor.
		SpatialManipulator();

		/// @brief Constructor at a location.
		/// @param baseTransform Pose of base of robot.
		SpatialManipulator(const Eigen::Matrix4d& baseTransform);

		/// @brief Set the base transform of the robot.
		/// @param baseTransform Robot base transform.
		void setBaseTransform(const Eigen::Matrix4d& baseTransform);

		/// @brief Get the base transform of the manipulator.
		/// @return Base transform.
		Eigen::Matrix4d getBaseTransform() const;

		/// @brief Set the physics scene pointer.
		/// @param physicsScene Pointer to physics scene.
		void setPhysicsScene(PhysicsScene* physicsScene);

		/// @brief Set the joint displacements and compute forward kinematics.
		/// @param jointDisplacements Joint displacements.
		/// @return If the joint displacements were successfully set.
		bool setJointDisplacements(const Eigen::VectorXd& jointDisplacements);

		/// @brief Get the current joint displacements.
		/// @return Joint displacements.
		Eigen::VectorXd getJointDisplacements() const;

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

		/// @brief Set maximum reach of the manipulator.
		void setMaxReach(double maxReach);

		/// @brief Get the maximum reach of the manipulator.
		/// @return Maximum reach.
		double getMaxReach() const;

		/// @brief Check if the robot is colliding with any obstacles.
		/// @return If the robot is colliding with the environment.
		bool isColliding() const;

		/// @brief Get the joint limits of the manipulator.
		/// @return Joint limits.
		std::vector<std::pair<double, double>> getJointLimits() const;

		/// @brief Generate a motion plan to the specfied pose.
		/// @param goalTransform Goal pose.
		/// @return Motion plan results.
		MotionPlanResults motionPlan(const Eigen::Matrix4d& goalTransform);
	};
}
