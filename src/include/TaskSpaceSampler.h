#pragma once

#include <Eigen/Dense>
#include <random>

namespace MotionPlanner
{
	/// @brief Generates pose samples in the task space.
	class TaskSpaceSampler
	{
		/// @brief Origin of the robot.
		const Eigen::Vector3d m_robotOrigin;

		/// @brief Radius of the semi-sphere from which to draw samples in R3.
		const double m_samplingSphereRadius;

		/// @brief Generates seed for MT engine.
		std::random_device m_randomDevice;

		/// @brief MT engine to generate random numbers.
		std::mt19937 m_engine;

		/// @brief Uniform distribution to draw from.
		std::uniform_real_distribution<double> m_uniformDist;

		/// @brief Generate random number between -1 and 1.
		/// @return Random number.
		double drawNum();

	public:
		/// @brief Constructor.
		/// @param robotOrigin Origin of the robot.
		/// @param samplingSphereRadius Radius of the semi-sphere centered at the robot's origin to sample positions from.
		TaskSpaceSampler(const Eigen::Vector3d& robotOrigin, double samplingSphereRadius);

		/// @brief Draw a position sample from R3.
		/// @return R3 sample.
		Eigen::Vector3d drawR3Sample();

		/// @brief Draw an orientation sample  from SO3.
		/// @return SO3 sample.
		Eigen::Quaterniond drawSO3Sample();

		/// @brief Draw a pose sample from SE3
		/// @return SE3 sample.
		Eigen::Matrix4d drawSE3Sample();
	};
}