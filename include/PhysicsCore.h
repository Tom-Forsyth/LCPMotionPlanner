#pragma once

#include "PxPhysicsAPI.h"
#include <map>
#include <string>

namespace CollisionAvoidance
{
	class PhysicsScene;

	// Class to create core PhysX instance.
	class PhysicsCore
	{
	private:
		// Allocator.
		physx::PxAllocatorCallback* m_allocator = nullptr;

		// Error callback.
		physx::PxErrorCallback* m_errorCallback = nullptr;

		// Tolerance scale.
		physx::PxTolerancesScale* m_toleranceScale = nullptr;

		// Foundation.
		physx::PxFoundation* m_foundation = nullptr;

		// Physics factory class.
		physx::PxPhysics* m_physics = nullptr;

		// CPU dispatcher.
		physx::PxCpuDispatcher* m_dispatcher = nullptr;

		// PhysX Visual Debugger instance.
		physx::PxPvd* m_pvd = nullptr;

		// Number of threads to give to PhysX.
		physx::PxU32 m_numThreads = 1;

		// Bool to determine if we own the PhysX instance or if external software does.
		bool bOwnPhysicsInstance = false;

		// Bool to determine if PhysicsCore has been initialized.
		bool bIsInitialized = false;

		// Bool to determine if we want to setup PVD.
		bool bEnablePVD = false;

		// Map of the created scenes.
		std::map<std::string, PhysicsScene*> m_scenes;

	public:
		// Constructor.
		PhysicsCore();

		// Destructor.
		~PhysicsCore();

		// Create PhysX instance.
		void createPhysicsCore();

		// TODO: Connect to Unreal Engine's PhysX instance.
		void connectPhysicsCore();

		// Create a Physics Scene.
		PhysicsScene* createPhysicsScene(const std::string& sceneName);
	};
}