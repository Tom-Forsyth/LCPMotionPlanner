#pragma once

#include "PxPhysicsAPI.h"
#include <map>
#include <string>

namespace MotionPlanner
{
	class PhysicsScene;

	/// @brief Class to create core PhysX instance.
	class PhysicsCore
	{
	private:
		/// @brief PhysX memory allocator.
		physx::PxAllocatorCallback* m_allocator = nullptr;

		/// @brief PhysX error callback.
		physx::PxErrorCallback* m_errorCallback = nullptr;

		/// @brief PhysX tolerance scale.
		physx::PxTolerancesScale* m_toleranceScale = nullptr;

		/// @brief PhysX foundation.
		physx::PxFoundation* m_foundation = nullptr;

		/// @brief PhysX physics factory class.
		physx::PxPhysics* m_physics = nullptr;

		/// @brief PhysX CPU dispatcher.
		physx::PxCpuDispatcher* m_dispatcher = nullptr;

		/// @brief PhysX visual debugger instance.
		physx::PxPvd* m_pvd = nullptr;

		/// @brief Number of threads to run PhysX simulation on.
		physx::PxU32 m_numThreads = 1;

		/// @brief Flag to determine if this class created the PhysX instance, or if we are simply connecting to an applications PhysX instance.
		bool bOwnPhysicsInstance = false;

		/// @brief Flag to determine if PhysX has been initialized.
		bool bIsInitialized = false;

		/// @brief Flag to determine if PVD should be connected to.
		bool bEnablePVD = false;

		/// @brief Scenes belonging to this PhysX instance.
		std::map<std::string, PhysicsScene*> m_scenes;

	public:
		/// @brief Constructor.
		PhysicsCore();

		/// @brief Destructor.
		~PhysicsCore();

		/// @brief Create PhysX instance.
		void createPhysicsCore();

		/// @brief Connect to Unreal Engine's PhysX instance.
		/// @bug Not yet implemented.
		void connectPhysicsCore();

		/// @brief Create a physics scene.
		/// @param sceneName Name of physics scene.
		/// @return Pointer to physics scene.
		PhysicsScene* createPhysicsScene(const std::string& sceneName);
	};
}