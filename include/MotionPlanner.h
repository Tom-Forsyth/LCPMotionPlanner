#pragma once

#include "DualQuaternion.h"
#include <Eigen/Dense>
#include <vector>
#include "RigidBody.h"
#include "LCPSolve.h"

class SpatialManipulator;

class MotionPlanner
{
private:
	SpatialManipulator* m_pSpatialManipulator = nullptr;
	std::vector<Eigen::VectorXd> m_plan;

	// Plan parameters.
	bool m_isRunning = true;
	size_t m_maxIterations = 1;
	double m_tau = 0.01;
	double m_timeStep = 0.01;
	double m_safetyDistance = 0.01;
	double m_maxDisplacementChangeAllowed = 0.005;

	// Manipulator information.
	RigidBody m_endFrame;
	int m_dof;
	Eigen::Matrix4d m_goalTransform;
	Eigen::Matrix4d m_currentTransform;
	DualQuaternion m_currentDualQuat;
	DualQuaternion m_goalDualQuat;
	Eigen::Vector<double, 7> m_currentConcat;

public:
	// Constructor.
	MotionPlanner(SpatialManipulator* pSpatialManipulator, const Eigen::Matrix4d& goalTransform);

	// Get the change in joint displacements before correction using ScLERP.
	Eigen::VectorXd getJointDisplacementChange() const;
	
	// Get the null space term.
	double getNullSpaceTerm() const;

	// Formulate and solve LCP to get the compensating velocities and get joint displacements.
	Eigen::VectorXd getCollisionDisplacementChange(const Eigen::VectorXd& displacementChange) const;

	// Add joint displacements and ensure they respect the linearization assumption.
	Eigen::VectorXd getTotalDisplacementChange(const Eigen::VectorXd& displacementChange, const Eigen::VectorXd& collisionDisplacementChange);

	// Generate motion plan.
	void computePlan();

};