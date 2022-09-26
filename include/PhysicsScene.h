#pragma once

#include "PhysicsCore.h"
#include "ContactReportCallback.h"
#include "Shape.h"
#include "RigidBody.h"
#include "RigidBodyChain.h"
#include "SpatialManipulator.h"
#include "ContactManager.h"
#include "ContactPoint.h"
#include "PxPhysicsAPI.h"
#include <string>
#include <vector>
#include <map>

namespace CollisionAvoidance
{
	// Physics scene to manage simulation scene.
	class PhysicsScene
	{
	private:
		// Scene name.
		std::string m_name;

		// PhysX scene.
		physx::PxScene* m_scene = nullptr;

		// Physics factory class.
		physx::PxPhysics* m_physics = nullptr;

		// Contact reporting callback class.
		physx::PxSimulationEventCallback* m_simulationEventCallback = nullptr;

		// Default actor material.
		physx::PxMaterial* m_material = nullptr;

		// Offset in meters at which to start generating contacts.
		double m_contactGenOffset = 0.1;

		// Simulation time step.
		double m_timeStep = 0.01;

		// Obstacle actors.
		std::map<std::string, physx::PxRigidStatic*> m_obstacleActors;

		// Robot geometry actors.
		std::map<std::string, physx::PxRigidDynamic*> m_robotGeometryActors;

		// Scene contact manager.
		ContactManager m_contactManager;

		// Add a robot geometry actor to the scene.
		void addCollider(const Shape& shape);

		// Add a robot rigid body to the scene.
		void addCollisionAggregate(const CollisionAggregate& collisionAggregate);

		// Add all of the bodies of a RigidBodyChain to the scene.
		void addRigidBodyChain(const RigidBodyChain& rigidBodyChain);

		// Sync the transforms of the PhysX actors with the collision actors.
		void syncTransforms();

	public:
		// Constructor.
		PhysicsScene(const std::string& sceneName, physx::PxPhysics* const physics, physx::PxCpuDispatcher* const cpuDispatcher);

		// Destructor.
		~PhysicsScene();

		// Connect scene to PVD client.
		void connectToPVD() const;

		// Add an obstacle to the scene.
		void addObstacle(const Shape& shape);

		// Add a spatial manipulator to the scene and give it a pointer to the scene.
		void addSpatialManipulator(SpatialManipulator& spatialManipulator);

		// Add a contact to be processed by the contact manager.
		void addContact(const std::string& linkName, const ContactPoint& contactPoint);

		// Set the contact generation offset to be slightly larger than safety distance.
		void setContactOffsets(double manipulatorSafetyDistance, double fingerSafetyDistance);

		// Simulate to generate contacts.
		const std::map<std::string, ContactPoint>& generateContacts();
	};
}