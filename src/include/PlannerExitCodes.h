#pragma once

namespace MotionPlanner
{
	/// @brief Global planner exit codes.
	enum class GlobalPlannerExitCode
	{
		Success,
		NoPathFound,
		MaxIterationsExceeded,
		Undefined
	};

	/// @brief Local planner exit codes.
	enum class LocalPlannerExitCode
	{
		Success,
		Collision,
		LCPError,
		MaxIterationsExceeded,
		JointLimitViolation,
		StuckAtLocalMinimum,
		Undefined
	};
}
