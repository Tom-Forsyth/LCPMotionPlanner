#include "PhysicsCore.h"
#include "PxPhysicsAPI.h"
#include "PhysicsScene.h"

namespace MotionPlanner
{
	PhysicsCore::PhysicsCore()
	{
#ifndef NDEBUG
		bEnablePVD = true;
#endif
		bEnablePVD = true;
	}

	PhysicsCore::~PhysicsCore()
	{
		// Only free memory if we allocated it, so we don't free memory managed by Unreal Engine.
		if (bOwnPhysicsInstance)
		{
			// Free PhysX core objects.
			PxCloseExtensions(); 
			if (m_physics)
			{
				m_physics->release();
			}
			if (m_foundation)
			{
				m_foundation->release();
			}

			// Free allocated members.
			if (m_allocator)
			{
				delete m_allocator;
			}
			if (m_errorCallback)
			{
				delete m_errorCallback;
			}
			if (m_toleranceScale)
			{
				delete m_toleranceScale;
			}
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
				if (m_foundation)
				{
					m_pvd = physx::PxCreatePvd(*m_foundation);
					physx::PxPvdTransport* transport = physx::PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 100);
					m_pvd->connect(*transport, physx::PxPvdInstrumentationFlag::eALL);
					bTrackAllocations = true;
				}
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