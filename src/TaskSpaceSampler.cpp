#include "TaskSpaceSampler.h"
#include <Eigen/Dense>
#include <random>
#include <math.h>

namespace MotionPlanner
{
	TaskSpaceSampler::TaskSpaceSampler(const Eigen::Vector3d& robotOrigin, double samplingSphereRadius)
		: m_robotOrigin(robotOrigin), m_samplingSphereRadius(samplingSphereRadius*2),
		m_engine(std::mt19937(m_randomDevice())), 
		m_uniformDist(std::uniform_real_distribution<double>(-1, 1))
	{
		
	}

	Eigen::Vector3d TaskSpaceSampler::drawR3Sample()
	{
		// Draw displacement sample within the hemi-sphere.
		Eigen::Vector3d sample(drawNormalReal(), drawNormalReal(), abs(drawNormalReal()));
		sample *= m_samplingSphereRadius;

		// Add to the robot origin and return.
		return sample + m_robotOrigin;
	}

	Eigen::Quaterniond TaskSpaceSampler::drawSO3Sample()
	{
		// Draw a random quaternion and normalize.
		Eigen::Quaterniond sample(drawUniformReal(), drawUniformReal(), drawUniformReal(), drawUniformReal());
		sample.normalize();
		return sample;
	}

	Eigen::Matrix4d TaskSpaceSampler::drawSE3Sample()
	{
		// Draw position and orientation sample.
		Eigen::Vector3d position = drawR3Sample();
		Eigen::Quaterniond orientation = drawSO3Sample();

		// Create transformation matrix.
		Eigen::Matrix4d sample = Eigen::Matrix4d::Identity();
		sample.block(0, 0, 3, 3) = orientation.toRotationMatrix();
		sample.block(0, 3, 3, 1) = position;
		return sample;
	}

	double TaskSpaceSampler::drawUniformReal()
	{
		return m_uniformDist(m_engine);
	}

	double TaskSpaceSampler::drawNormalReal()
	{
		return (m_normalDist(m_engine) * 2) - 1;
	}
}