#pragma once

#include "DualQuaternion.h"
#include "RigidBody.h"
#include "LCPSolve.h"
#include "MotionPlanningParameters.h"
#include "MotionPlanResults.h"
#include "PlannerExitCodes.h"
#include <Eigen/Dense>
#include <vector>

namespace MotionPlanner
{
	class SpatialManipulator;

	/// @brief ScLERP motion planner with LCP collision avoidance. 
	class LocalPlanner
	{
	private:
		/// @brief Pointer to the spatial manipulator to generate motion plan with.
		SpatialManipulator* m_pSpatialManipulator = nullptr;

		/// @brief Motion plan.
		std::vector<Eigen::VectorXd> m_plan;

		/// @brief Struct with motion planning parameters.
		MotionPlanningParameters m_params;

		/// @brief Flag to determine if the plan is still computing.
		bool m_isRunning = true;

		/// @brief Enforce stricter linearization near goal.
		bool m_isNearGoal = false;

		/// @brief Exit code for the LCP solver.
		int m_exitCodeLCP = -1;

		/// @brief Exit code for the local motion planner.
		LocalPlannerExitCode m_exitCodePlanner = LocalPlannerExitCode::Undefined;

		/// @brief Degree of freedom of manipulator.
		int m_dof;

		/// @brief Initial end-effector pose.
		const Eigen::Matrix4d m_startTransform;

		/// @brief Initial joint space configuration.
		const Eigen::MatrixXd m_startDisplacements;

		/// @brief Goal pose transform.
		const Eigen::Matrix4d m_goalTransform;

		/// @brief Goal pose dual quaternion.
		DualQuaternion m_goalDualQuat;

		/// @brief Goal pose position/quaternion.
		Eigen::Vector<double, 7> m_goalConcat;

		/// @brief Current pose transform.
		Eigen::Matrix4d m_currentTransform;

		/// @brief Current pose dual quaternion.
		DualQuaternion m_currentDualQuat;
		
		/// @brief Current pose position/quaternion.
		Eigen::Vector<double, 7> m_currentConcat;

		/// @brief Manipulator spatial jacobian.
		Eigen::MatrixXd m_spatialJacobian;

		/// @brief Null space matrix.
		Eigen::MatrixXd m_nullSpaceTerm;

		/// @brief Compute the change in joint displacements due to ScLERP.
		/// @return Joint displacements.
		Eigen::VectorXd getJointDisplacementChange();

		/// @brief Formulate and solve LCP to get the joint displacement change to avoid obstacles.
		Eigen::VectorXd getCollisionDisplacementChange(const Eigen::VectorXd& displacementChange);

		/// @brief Add the ScLERP and collision avoidance joint displacements and ensure they respect linearization.
		/// @param displacementChange Change in displacements due to ScLERP.
		/// @param collisionDisplacementChange Change in displacements due to collision avoidance.
		/// @return Total joint displacement change for the current step.
		Eigen::VectorXd getTotalDisplacementChange(const Eigen::VectorXd& displacementChange, const Eigen::VectorXd& collisionDisplacementChange);

		/// @brief Ensure that the robot is not colliding with obstacles.
		/// @return Penetration condition.
		bool isPenetrating();

		/// @brief Check if the robot is near the goal to change the linearization mode.
		void checkNearGoal(double posError, double quatError);

		/// @brief Compute the null space term.
		void computeNullSpaceTerm();

	public:
		/// @brief Constructor.
		/// @param pSpatialManipulator Pointer to the spatial manipulator to generate the plan with.
		/// @param goalTransform Goal transform of the end-effector.
		LocalPlanner(SpatialManipulator* pSpatialManipulator, const Eigen::Matrix4d& goalTransform);

		/// @brief Generate motion plan.
		void computePlan();

		/// @brief Get the motion plan.
		/// @return Vector of joint displacements.
		MotionPlanResults getPlanResults() const;
	};
}
