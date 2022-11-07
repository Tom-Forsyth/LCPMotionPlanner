#include "TaskSpaceSampler.h"
#include "SpatialManipulator.h"
#include <Eigen/Dense>
#include <random>
#include <math.h>

namespace MotionPlanner
{
	TaskSpaceSampler::TaskSpaceSampler(const Eigen::Vector3d& robotOrigin, double samplingSphereRadius)
		: m_robotOrigin(0.5, 0, 0.25), m_samplingSphereRadius(samplingSphereRadius),
		m_engine(std::mt19937(m_randomDevice())), 
		m_uniformDist(std::uniform_real_distribution<double>(-1, 1))
	{
		
	}

	Eigen::Vector3d TaskSpaceSampler::drawR3Sample()
	{
		// Draw displacement sample within the hemi-sphere.
		Eigen::Vector3d sample(drawUniformReal(), drawUniformReal(), abs(drawUniformReal()));
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

	Eigen::VectorXd TaskSpaceSampler::drawJointSpaceSample(const SpatialManipulator* robot)
	{
		// Get the joint angle limits.
		std::vector<std::pair<double, double>> jointLimits = robot->getJointLimits();

		// Draw a uniform sample in the range of the joint for each joint.
		size_t dim = jointLimits.size();
		Eigen::VectorXd sample = Eigen::VectorXd::Zero(dim);
		for (int i = 0; i < dim; i++)
		{
			// Sample interpolation factor between 0 and 1 and linear interpolate.
			double interpFactor = abs(drawUniformReal());
			const std::pair<double, double>& jointLimit = jointLimits[i];
			sample[i] = jointLimit.first + interpFactor * jointLimit.second;
		}

		return sample;
	}
}