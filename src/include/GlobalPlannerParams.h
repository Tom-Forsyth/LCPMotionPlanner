#pragma once

namespace MotionPlanner
{
	/// @brief Parameters for the RRT global planner.
	struct GlobalPlannerParams
	{
		/// @brief Max iterations.
		const int maxIterations = 1000;

		/// @brief Connection distance between nodes in meters.
		const double maxConnectionDistance = 0.05;

		/// @brief Probability the goal pose will be sampled when an SE3 sample is requested.
		const double sampleGoalProbability = 0.25;

		/// @brief Determines if the sampled pose should use the previous orientation or a new one.
		const bool sampleOrientation = false;
	};
}
