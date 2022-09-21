#include "PhysicsCore.h"
#include "PxPhysicsAPI.h"
#include "PhysicsScene.h"

namespace CollisionAvoidance
{
	PhysicsCore::PhysicsCore()
	{
#ifndef NDEBUG
		bEnablePVD = true;
#endif
	}

	PhysicsCore::~PhysicsCore()
	{
		// Only free memory if we allocated it, so we don't free memory managed by Unreal Engine.
		if (bOwnPhysicsInstance)
		{
			// Free PhysX core objects.
			PxCloseExtensions(); 
			m_physics->release();
			m_foundation->release();

			// Free allocated members.
			delete m_allocator;
			delete m_errorCallback;
			delete m_toleranceScale;
		}
	}

	void PhysicsCore::createPhysicsCore()
	{
		if (!bIsInitialized)
		{
			// Create default allocator callback.
			m_allocator = new physx::PxDefaultAllocator;

			// Create default error callback.
			m_errorCallback = new physx::PxDefaultErrorCallback;

			// Create foundation.
			m_foundation = PxCreateFoundation(PX_PHYSICS_VERSION, *m_allocator, *m_errorCallback);

			// Create and connect to PVD if in debug mode since it is time/resource intensive.
			bool bTrackAllocations = false;
			if (bEnablePVD)
			{
				m_pvd = physx::PxCreatePvd(*m_foundation);
				physx::PxPvdTransport* transport = physx::PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 10);
				m_pvd->connect(*transport, physx::PxPvdInstrumentationFlag::eALL);
				bTrackAllocations = true;
			}

			// Create tolerances scale.
			m_toleranceScale = new physx::PxTolerancesScale;
			m_toleranceScale->length = 1.0f;
			m_toleranceScale->speed = m_toleranceScale->length * 9.81f;

			// Create PhysX factory class.
			m_physics = PxCreatePhysics(PX_PHYSICS_VERSION, *m_foundation, *m_toleranceScale, bTrackAllocations, m_pvd);
			PxInitExtensions(*m_physics, m_pvd);

			// Create dispatcher.
			m_dispatcher = physx::PxDefaultCpuDispatcherCreate(m_numThreads == 0 ? 0 : m_numThreads - 1);

			// Change flags.
			bOwnPhysicsInstance = true;
			bIsInitialized = true;
		}
	}

	void PhysicsCore::connectPhysicsCore()
	{
		// Not yet implemented.
	}

	PhysicsScene* PhysicsCore::createPhysicsScene(const std::string& sceneName)
	{
		// Create scene.
		PhysicsScene* physicsScene = new PhysicsScene(sceneName, m_physics, m_dispatcher);
		m_scenes.insert({ sceneName, physicsScene });

		// Add scene to PVD if enabled.
		if (bEnablePVD)
		{
			physicsScene->connectToPVD();
		}

		return physicsScene;
	}
}