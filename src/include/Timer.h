#pragma once

#include <chrono>

namespace MotionPlanner
{
	/// @brief General purpose code timer.
	class Timer
	{
		/// @brief Start time.
		std::chrono::steady_clock::time_point m_start;

		/// @brief Stop time.
		std::chrono::steady_clock::time_point m_stop;

	public:
		/// @brief Start timer.
		void start();

		/// @brief Stop timer.
		void stop();

		/// @brief Get the elapsed time in milliseconds.
		/// @return Elapsed time.
		double getTimeMilli() const;

		/// @brief Get the elapsed time in microseconds.
		/// @return Elapsed time.
		double getTimeMicro() const;
	};
}
