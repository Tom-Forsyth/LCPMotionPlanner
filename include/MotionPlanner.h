#pragma once

#include <Eigen/Dense>
#include <vector>

class SpatialManipulator;

class MotionPlanner
{
private:
	SpatialManipulator* m_pSpatialManipulator = nullptr;
	std::vector<Eigen::VectorXd> m_plan;

	// Plan parameters.
	size_t m_maxIterations = 1000;
	double m_tau = 0.01;
	double m_timeStep = 0.01;
	double m_safetyDistance = 0.01;

public:
	// Constructor.
	MotionPlanner(SpatialManipulator* pSpatialManipulator);

	// Initialize.
	void init();

	// Generate motion plan.
	void computePlan(const Eigen::Matrix4d& goalTransform);

};