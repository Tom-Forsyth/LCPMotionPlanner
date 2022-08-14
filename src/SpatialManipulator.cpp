#include "SpatialManipulator.h"
#include "RigidBodyChain.h"
#include <Eigen/Dense>
#include "PhysXEnv.h"

// Default constructor.
SpatialManipulator::SpatialManipulator()
{
	setBaseTransform(Eigen::Matrix4d::Identity());
}

// Base transform constructor.
SpatialManipulator::SpatialManipulator(const Eigen::Matrix4d& baseTransform)
{
	setBaseTransform(baseTransform);
}

// Set base transform.
void SpatialManipulator::setBaseTransform(const Eigen::Matrix4d& baseTransform)
{
	m_rigidBodyChain.setBaseTransform(baseTransform);
}

// Set joint displacements.
void SpatialManipulator::setJointDisplacements(const std::vector<double>& jointDisplacements)
{
	m_rigidBodyChain.setJointDisplacements(jointDisplacements);
	generateContacts();
}

// Give simulation environment a pointer to the chain.
void SpatialManipulator::setupSimulationEnvironment()
{
	m_simulationEnvironment.setRigidBodyChain(&m_rigidBodyChain);
	m_simulationEnvironment.initColliders();
}

// Run simulation to generate contacts.
void SpatialManipulator::generateContacts()
{
	m_simulationEnvironment.updateTransforms();
	m_simulationEnvironment.simulate();
	m_rigidBodyChain.condenseContacts();
	m_rigidBodyChain.updateContactJacobians();
}

// Add sphere obstacle.
void SpatialManipulator::addObstacle(const Sphere& sphere)
{
	m_simulationEnvironment.createObstacleActor(sphere);
}

// Add capsule obstacle.
void SpatialManipulator::addObstacle(const Capsule& capsule)
{
	m_simulationEnvironment.createObstacleActor(capsule);
}

// Add box obstacle.
void SpatialManipulator::addObstacle(const Box& box)
{
	m_simulationEnvironment.createObstacleActor(box);
}
