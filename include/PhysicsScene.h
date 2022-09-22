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
		void addContact(const std::string& colliderName, const ContactPoint& contactPoint);

		// Set the contact generation offset to be slightly larger than safety distance.
		void setContactOffsets(double manipulatorSafetyDistance, double fingerSafetyDistance);

		// Simulate to generate contacts.
		void generateContacts();


		/*
			Use case:
			- User adds object to scene.
				- PxActor created at exact pose.
				- Store pointer to PxActor in scene in one vector.
			- User adds object to robot.
				- PxActor created at exact pose.
				- Store pointer to PxActor in scene in vector of vectors.
				- Call updateScene() method of MotionPlanner to loop through rigidBodies and update the PxActor locations.
				- updateScene() will be called every iteration.
			* How do we want to structure the PxActors to represent the robot?
				- Currently using vector of vectors (link, shapes of link).
				- Could use a map of vectors (link, shapes of link).
			* How do we want to access the PxActors?
				- updateScene() will be called by MotionPlanner or RigidBodyChain? or PhysicsScene?
				- Do we want to modify Shape to have an absolute transform or keep the local?
				- Probably want to keep local so displacements are easier to calculate.
				- Also consider in the case of a gripper where the one PhysicsScene has many RigidBodyChains.

			** Solution: PhysicsScene will take in a pointer/reference of the RigidBodyChain
			* PhysicsScene must hold actors in a structure that can handle multiple robots/chains, but one set of obstacles.
				- PhysicsScene will hold a vector of pointers to RigidBodyChains.
				- PhysicsScene will hold a map of <std::string, PxActor*> shape name, PxActor pairs.
				- PhysicsScene::updateTransforms() will loop through this vector.
					- For each RigidBodyChain, loop through each rigid body, and each shape.
					- Find the shape's PxActor with the name of the shape. (Throw error if shape is created with existing name)
					- Update PxActor's transform.
		*/
	};
}