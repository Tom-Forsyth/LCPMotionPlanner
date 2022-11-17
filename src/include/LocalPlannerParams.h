#pragma once

namespace MotionPlanner
{
	/// @brief Default motion planning parameters.
	struct LocalPlannerParams
	{
		/// @brief Max iterations before termination.
		const size_t maxIterations = 2000;

		/// @brief Distance at which to start avoiding obstacles.
		const double safetyDistance = 0.01;

		/// @brief Max displacement change of any of the joints due to ScLERP.
		double maxScLERPDisplacementChange = 0.005;

		/// @brief Max displacement change of any of the joints due to collision avoidance.
		double maxCollisionDisplacementChange = 0.005;

		/// @brief Max displacement change of any of the joints.
		double maxTotalDisplacementChange = 0.01;

		/// @brief Convergence criteria for position.
		const double positionTolerance = 0.005;

		/// @brief Convergence criteria for orientation.
		const double quatTolerance = 0.01;

		/// @brief ScLERP interpolation factor.
		double tau = 0.01;

		/// @brief Flag to determine if tau has been increased to 1 and we should stop trying to increase.
		bool tauIsMax = false;

		/// @brief Minimum change in end-effector position and quaternion concatination vector norm to be considered not stuck at a local minimum.
		const double concatDisplacementThreshold = 0.000001;

		/// @brief Time step for motion plan.
		/// @note Unused, since the time step factors out in the LCP, and we are clamping step size with joint displacement change.
		const double timeStep = 0.01;
	};
}
