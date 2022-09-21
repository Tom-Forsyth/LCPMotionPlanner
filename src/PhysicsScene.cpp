#include "PhysicsScene.h"
#include "ContactReportFilterShader.h"
#include "ContactReportCallback.h"
#include "Shape.h"
#include "PhysXHelperFunctions.h"
#include "ObjectType.h"
#include "RigidBody.h"
#include "CollisionAggregate.h"
#include "PxPhysicsAPI.h"
#include <string>
#include <memory>
#include <exception>
#include <vector>

namespace CollisionAvoidance
{
	PhysicsScene::PhysicsScene(const std::string& sceneName, physx::PxPhysics* physics, physx::PxCpuDispatcher* cpuDispatcher)
		: m_name(sceneName), m_physics(physics)
	{
		// Assign simulation event callback.
		m_simulationEventCallback = new ContactReportCallback();

		// Create scene description.
		physx::PxSceneDesc sceneDesc(m_physics->getTolerancesScale());
		sceneDesc.cpuDispatcher = cpuDispatcher;
		sceneDesc.filterShader = contactReportFilterShader;
		sceneDesc.kineKineFilteringMode = physx::PxPairFilteringMode::eKEEP;
		sceneDesc.staticKineFilteringMode = physx::PxPairFilteringMode::eKEEP;
		sceneDesc.simulationEventCallback = m_simulationEventCallback;
		sceneDesc.flags |= physx::PxSceneFlag::eENABLE_PCM;
		m_scene = m_physics->createScene(sceneDesc);

		// Add scene to PVD.
		physx::PxPvdSceneClient* pvdClient = m_scene->getScenePvdClient();
		if (pvdClient)
		{
			pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		}

		// Setup default material.
		m_material = m_physics->createMaterial(0.0f, 0.0f, 0.0f);
	}

	PhysicsScene::~PhysicsScene()
	{
		delete m_simulationEventCallback;
	}

	void PhysicsScene::connectToPVD() const
	{
		physx::PxPvdSceneClient* pvdClient = m_scene->getScenePvdClient();
		if (pvdClient)
		{
			pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		}
	}

	void PhysicsScene::addObstacle(const Shape& shape)
	{
		if (shape.getObjectType() == ObjectType::Obstacle)
		{
			// Get PxTransform.
			const Eigen::Matrix4d eigenTransform = shape.getTransform();
			const physx::PxTransform transform = eigenMatrixToPxTransform(eigenTransform);

			// Create RigidStatic actor of the obstacle.
			physx::PxRigidStatic* actor = m_physics->createRigidStatic(transform);

			// Create a PxShape from the CollisionAvoidance::Shape.
			const std::unique_ptr<physx::PxGeometry> pxGeomPtr = getPxGeometry(shape);
			const physx::PxGeometry& pxGeom = *(pxGeomPtr.get());
			physx::PxShape* pxShape = m_physics->createShape(pxGeom, *m_material, true);

			// Set the object type as an obstacle for collision filtering.
			physx::PxFilterData filterData;
			filterData.word0 = static_cast<uint32_t>(ObjectType::Obstacle);
			pxShape->setSimulationFilterData(filterData);

			// Set the shape and actor name.
			const std::string shapeName = shape.getName();
			const char* shapeNameChar = shapeName.c_str();
			actor->setName(shapeNameChar);
			pxShape->setName(shapeNameChar);

			// Store pointer to the original shape in the PxShape and PxActor.
			// Note that we must use const_cast here since the PhysX API only accepts a non-const void*.
			// My internal use of the shape is read only so there is no concern.
			const void* shapePointerConst = &shape;
			void* shapePointer = const_cast<void*>(shapePointerConst);
			actor->userData = shapePointer;
			pxShape->userData = shapePointer;

			// Attach the shape to the actor and release.
			actor->attachShape(*pxShape);
			pxShape->release();

			// Add the new actor to scene.
			m_obstacleActors.emplace(shapeName, actor);
			m_scene->addActor(*actor);
		}
		else
		{
			throw std::invalid_argument("Shape must be an obstacle.");
		}
	}

	void PhysicsScene::addCollider(const Shape& shape)
	{
		if (shape.getObjectType() == ObjectType::RobotGeometry)
		{
			// Get PxTransform.
			const Eigen::Matrix4d eigenTransform = shape.getTransform();
			const physx::PxTransform transform = eigenMatrixToPxTransform(eigenTransform);

			// Create RigidDynamic kinematic actor.
			physx::PxRigidDynamic* actor = m_physics->createRigidDynamic(transform);
			actor->setRigidBodyFlag(physx::PxRigidBodyFlag::eKINEMATIC, true);

			// Create a PxShape from the CollisionAvoidance::Shape.
			const std::unique_ptr<physx::PxGeometry> pxGeomPtr = getPxGeometry(shape);
			const physx::PxGeometry& pxGeom = *(pxGeomPtr.get());
			physx::PxShape* pxShape = m_physics->createShape(pxGeom, *m_material, true);

			// Set the contact offset for the shape.
			pxShape->setContactOffset(static_cast<physx::PxReal>(m_contactGenOffset));

			// Set the object type as robot geometry for collision filtering.
			physx::PxFilterData filterData;
			filterData.word0 = static_cast<uint32_t>(ObjectType::RobotGeometry);
			pxShape->setSimulationFilterData(filterData);

			// Set the shape and actor name.
			const std::string shapeName = shape.getName();
			const char* shapeNameChar = shapeName.c_str();
			actor->setName(shapeNameChar);
			pxShape->setName(shapeNameChar);

			// Store pointer to the original shape in the PxShape and PxActor.
			// Note that we must use const_cast here since the PhysX API only accepts a non-const void*.
			// My internal use of the shape is read only so there is no concern.
			const void* shapePointerConst = &shape;
			void* shapePointer = const_cast<void*>(shapePointerConst);
			actor->userData = shapePointer;
			pxShape->userData = shapePointer;

			// Attach the shape to the actor and release.
			actor->attachShape(*pxShape);
			pxShape->release();

			// Add new actor to scene.
			m_robotGeometryActors.emplace(shapeName, actor);
			m_scene->addActor(*actor);
		}
		else
		{
			throw std::invalid_argument("Shape must be a robot geometry shape.");
		}
	}

	void PhysicsScene::addCollisionAggregate(const CollisionAggregate& collisionAggregate)
	{
		// Vector of colliders inside the collision aggregate.
		const std::vector<Shape*> colliders = collisionAggregate.getColliders();

		// Add each collider to the scene.
		for (const Shape* collider : colliders)
		{
			addCollider(*collider);
		}
	}

	void PhysicsScene::addRigidBodyChain(const RigidBodyChain& rigidBodyChain)
	{
		// For each rigid body in the chain, add the collision aggregate to the scene.
		for (const RigidBody& rigidBody : rigidBodyChain.getRigidBodies())
		{
			addCollisionAggregate(rigidBody.getCollisionAggregate());
		}
	}

	void PhysicsScene::addSpatialManipulator(const SpatialManipulator& spatialManipulator)
	{
		addRigidBodyChain(spatialManipulator.getRigidBodyChain());
	}

	void PhysicsScene::syncTransforms()
	{
		// For each PxActor of a collider, update the pose and target to the collider's current pose.
		for (std::pair<const std::string, physx::PxRigidDynamic*> colliderActor : m_robotGeometryActors)
		{
			physx::PxRigidDynamic*& pxActor = colliderActor.second;
			if (pxActor->userData)
			{
				const Shape* shapeActor = static_cast<const Shape*>(pxActor->userData);
				const Eigen::Matrix4d newTransform = shapeActor->getTransform();
				const physx::PxTransform newPxTransform = eigenMatrixToPxTransform(newTransform);
				pxActor->setGlobalPose(newPxTransform);
				pxActor->setKinematicTarget(newPxTransform);
			}
		}
	}

	void PhysicsScene::setContactOffsets(double manipulatorSafetyDistance, double fingerSafetyDistance)
	{
		/* Not yet implemented */
	}

	void PhysicsScene::simulate()
	{
		m_scene->simulate(0.01f);
		m_scene->fetchResults(true);
	}
}