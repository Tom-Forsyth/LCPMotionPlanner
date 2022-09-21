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

	void PhysicsScene::addRobotGeometry(const Shape& shape, const Eigen::Matrix4d& worldTransform)
	{
		if (shape.getObjectType() == ObjectType::RobotGeometry)
		{
			// Get PxTransform.
			const Eigen::Matrix4d eigenTransform = shape.getTransform();
			const physx::PxTransform transform = eigenMatrixToPxTransform(worldTransform * eigenTransform);

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

			// Attach the shape to the actor and release.
			actor->attachShape(*pxShape);
			pxShape->release();

			// Add new actor to scene.
			m_robotGeometryActors.emplace(shapeName, actor);
			m_scene->addActor(*actor);
		}
		else
		{
			throw std::invalid_argument("Shape must be an robot geometry shape.");
		}
	}

	void addRobotBody(const RigidBody& rigidBody)
	{
		// TODO: Refactor CollisionAggregate for polymorphism.
		
		// Get spheres, capsules, and boxes.
		const CollisionAggregate collisionAggregate = rigidBody.getCollisionAggregate();
		const std::vector<Sphere> spheres = collisionAggregate.getSpheres();
		const std::vector<Capsule> capsules = collisionAggregate.getCapsules();
		const std::vector<Box> boxes = collisionAggregate.getBoxes();
		/*
		// Create RigidDynamic kinematic actor vector.
		std::vector<physx::PxRigidDynamic*> rigidDynamicActors;
		std::vector<Eigen::Matrix4d> actorLocalTransforms;
		size_t nActors = spheres.size() + capsules.size() + boxes.size();
		rigidDynamicActors.reserve(nActors);
		actorLocalTransforms.reserve(nActors);

		// Add actors to scene. Transform to world frame before creation.
		Eigen::Matrix4d worldTransform = rigidBody.getCurrentWorldTransform();
		for (Sphere& sphere : spheres)
		{
			physx::PxRigidDynamic* actor = createCollisionActor(sphere, worldTransform);
			rigidDynamicActors.emplace_back(actor);
			actorLocalTransforms.emplace_back(sphere.getTransform());
		}

		for (Capsule& capsule : capsules)
		{
			physx::PxRigidDynamic* actor = createCollisionActor(capsule, worldTransform);
			rigidDynamicActors.emplace_back(actor);
			actorLocalTransforms.emplace_back(capsule.getTransform());
		}

		for (Box& box : boxes)
		{
			physx::PxRigidDynamic* actor = createCollisionActor(box, worldTransform);
			rigidDynamicActors.emplace_back(actor);
			actorLocalTransforms.emplace_back(box.getTransform());
		}

		// Add vector of actors to parent vector.
		m_collisionActors.emplace_back(rigidDynamicActors);
		m_collisionActorLocalTransforms.emplace_back(actorLocalTransforms);
		*/
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