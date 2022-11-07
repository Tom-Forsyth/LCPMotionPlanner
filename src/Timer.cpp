#include "Timer.h"
#include <chrono>

namespace MotionPlanner
{
	void Timer::start()
	{
		m_start = std::chrono::steady_clock::now();
	}

	void Timer::stop()
	{
		m_stop = std::chrono::steady_clock::now();
	}

	double Timer::getTimeMilli() const
	{
		return std::chrono::duration_cast<std::chrono::milliseconds>(m_stop - m_start).count();
	}

	double Timer::getTimeMicro() const
	{
		return std::chrono::duration_cast<std::chrono::microseconds>(m_stop - m_start).count();
	}
}
