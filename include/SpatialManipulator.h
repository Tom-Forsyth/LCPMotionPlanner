#pragma once

#include "RigidBodyChain.h"
#include <Eigen/Dense>
#include <vector>
#include "PhysXEnv.h"

class SpatialManipulator
{
protected:
	RigidBodyChain m_rigidBodyChain;
	PhysXEnv m_simulationEnvironment;

public:
	// Constructors.
	SpatialManipulator();
	SpatialManipulator(const Eigen::Matrix4d& baseTransform);

	// Set base transform.
	void setBaseTransform(const Eigen::Matrix4d& baseTransform);

	// Set joint displacements.
	void setJointDisplacements(const Eigen::VectorXd& jointDisplacements);

	// Setup simulation environment.
	void setupSimulationEnvironment();

	// Run simulation to generate contacts.
	void generateContacts();

	// Add obstacles.
	void addObstacle(const Sphere& sphere);
	void addObstacle(const Capsule& capsule);
	void addObstacle(const Box& box);

	// Generate motion plan.
	void motionPlan(const Eigen::Matrix4d& goalTransform);

	// Get transform of end frame.
	Eigen::Matrix4d getEndFrameSpatialTransform() const;

	// Return the last rigid body of the chain.
	RigidBody getEndFrame() const;
};