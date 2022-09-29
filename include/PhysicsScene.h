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

namespace MotionPlanner
{
	/// @brief Physics scene to manage simulation scene.
	class PhysicsScene
	{
	private:
		/// @brief Scene name.
		std::string m_name;

		/// @brief Pointer to PhysX scene.
		physx::PxScene* m_scene = nullptr;

		/// @brief Pointer to PhysX factory class.
		physx::PxPhysics* m_physics = nullptr;

		/// @brief Contact reporting simulation callback class.
		physx::PxSimulationEventCallback* m_simulationEventCallback = nullptr;

		/// @brief Default actor material.
		physx::PxMaterial* m_material = nullptr;

		/// @brief Offset in meters at which to start generating contacts.
		double m_contactGenOffset = 0.1;

		/// @brief Simulation time step.
		double m_timeStep = 0.01;

		/// @brief Environment obstacle actors.
		/// @bug Convert to vector? Do we need a map here?
		std::map<std::string, physx::PxRigidStatic*> m_obstacleActors;

		/// @brief Robot geometry actors.
		/// @bug Convert to vector? Do we need a map here?
		std::map<std::string, physx::PxRigidDynamic*> m_robotGeometryActors;

		/// @brief Contact manager that recieves all contacts and reduces to one per robot link.
		ContactManager m_contactManager;

		/// @brief Add a robot geometry actor to the scene.
		/// @param shape Shape actor.
		void addCollider(const Shape& shape);

		/// @brief Add an aggregate of robot geometry actors to the scene.
		/// @param collisionAggregate Aggregate of shape actors.
		void addCollisionAggregate(const CollisionAggregate& collisionAggregate);

		/// @brief Add all of the rigid bodies of a chain to the scene.
		/// @param rigidBodyChain Chain of rigid bodies.
		void addRigidBodyChain(const RigidBodyChain& rigidBodyChain);

		/// @brief Update the PhysX actor transforms with the tranforms of the robot geometry shape actors.
		void syncTransforms();

	public:
		/// @brief Constructor.
		/// @param sceneName Name of scene.
		/// @param physics Pointer to the PhysX factory class.
		/// @param cpuDispatcher Pointer to the PhysX CPU dispatcher.
		PhysicsScene(const std::string& sceneName, physx::PxPhysics* const physics, physx::PxCpuDispatcher* const cpuDispatcher);

		/// @brief Destructor.
		~PhysicsScene();

		/// @brief Connect the scene to the PVD client.
		void connectToPVD() const;

		/// @brief Add an obstacle to the scene.
		/// @param shape Obstacle actor.
		void addObstacle(const Shape& shape);

		/// @brief Add the robot geometry actors of a manipulator to the scene and provide the manipulator with a pointer to the scene.
		/// @param spatialManipulator Manipulator to add to scene.
		void addSpatialManipulator(SpatialManipulator& spatialManipulator);

		/// @brief Add a contact to be processed by the contact manager.
		/// @param linkName Name of rigid body that the the contact occured on.
		/// @param contactPoint Position, normal, separation, and status of contact.
		void addContact(const std::string& linkName, const ContactPoint& contactPoint);

		/// @brief Set the contact generation offset to be slightly larger than safety distance.
		/// @param manipulatorSafetyDistance Desired safety distance of manipulator.
		/// @param fingerSafetyDistance Desired safety distance of the gripper fingers.
		void setContactOffsets(double manipulatorSafetyDistance, double fingerSafetyDistance);

		/// @brief Run PhysX simulation to generate contacts and run contact manager to reduce contacts.
		/// @return Sorted and reduced contacts to be given to the manipulator.
		const std::map<std::string, ContactPoint>& generateContacts();
	};
}