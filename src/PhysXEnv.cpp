#include "PhysXEnv.h"
#include "PxPhysicsAPI.h"
#include "ContactReportCallback.h"
#include "ContactReportFilterShader.h"

#include "Sphere.h"
#include "Capsule.h"
#include "Box.h"
#include <Eigen/Dense>
#include <vector>
#include "SpatialManipulator.h"
#include "RigidBodyChain.h"
#include "CollisionAggregate.h"

// Default constructor.
PhysXEnv::PhysXEnv()
{
	init();
}


// Initialize PhysX.
void PhysXEnv::init()
{
	// Create foundation.
	m_foundation = PxCreateFoundation(PX_PHYSICS_VERSION, m_allocator, m_errorCallback);

	// Initialize PVD.
	#ifndef NDEBUG
	m_pvd = physx::PxCreatePvd(*m_foundation);
	physx::PxPvdTransport* transport = physx::PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 10);
	m_pvd->connect(*transport, physx::PxPvdInstrumentationFlag::eALL);
	#endif

	// Create physics factory class.
	m_toleranceScale.length = 0.1f;
	m_toleranceScale.speed = m_toleranceScale.length * 9.81f;
	m_physics = PxCreatePhysics(PX_PHYSICS_VERSION, *m_foundation, m_toleranceScale, true, m_pvd);
	PxInitExtensions(*m_physics, m_pvd);

	// Multithreading.
	physx::PxU32 numCores = 1;
	m_dispatcher = physx::PxDefaultCpuDispatcherCreate(numCores == 0 ? 0 : numCores - 1);

	// Create scene.
	physx::PxSceneDesc sceneDesc(m_physics->getTolerancesScale());
	sceneDesc.cpuDispatcher = m_dispatcher;
	sceneDesc.filterShader = contactReportFilterShader; // Must use this filter.
	sceneDesc.staticKineFilteringMode = physx::PxPairFilteringMode::eKEEP;
	sceneDesc.simulationEventCallback = &m_contactReportCallback; // Contact callback.
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


// Set rigid body chain and add colliders.
void PhysXEnv::setRigidBodyChain(RigidBodyChain* rigidBodyChain)
{
	m_rigidBodyChain = rigidBodyChain;
}

// Load chain's colliders into the scene.
void PhysXEnv::initColliders()
{
	// Reserve space in vector of pointers.
	size_t nBodies = m_rigidBodyChain->getNBodies();
	m_collisionActors.reserve(nBodies);
	m_collisionActorLocalTransforms.reserve(nBodies);

	// Add collision aggregates into scene.
	for (const RigidBody& body : m_rigidBodyChain->getRigidBodies())
	{
		addCollisionAggregate(body);
	}
}

// Update the pose of the colliders.
void PhysXEnv::updateTransforms()
{
	// Get rigid body vector to read the world transform of each body.
	std::vector<RigidBody> rigidBodies = m_rigidBodyChain->getRigidBodies();
	for (size_t linkIndex = 0; linkIndex < m_collisionActors.size(); linkIndex++)
	{
		for (size_t actorIndex = 0; actorIndex < m_collisionActors[linkIndex].size(); actorIndex++)
		{
			physx::PxRigidDynamic*& pActor = m_collisionActors[linkIndex][actorIndex]; // not sure if i need reference here to change value...
			Eigen::Matrix4d newEigenTransform = rigidBodies[linkIndex].getCurrentWorldTransform() * m_collisionActorLocalTransforms[linkIndex][actorIndex];
			physx::PxTransform newTransform = eigenMatrixToPxTransform(newEigenTransform);
			pActor->setGlobalPose(newTransform);
			pActor->setKinematicTarget(newTransform);
		}
	}
}

// Add robot's collision aggregate to the scene.
void PhysXEnv::addCollisionAggregate(const RigidBody& rigidBody)
{
	// Get spheres, capsules, and boxes.
	CollisionAggregate collisionAggregate = rigidBody.getCollisionAggregate();
	std::vector<Sphere> spheres = collisionAggregate.getSpheres();
	std::vector<Capsule> capsules = collisionAggregate.getCapsules();
	std::vector<Box> boxes = collisionAggregate.getBoxes();

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

}

// Add sphere collider to scene.
physx::PxRigidDynamic* PhysXEnv::createCollisionActor(Sphere& sphere, const Eigen::Matrix4d& worldTransform)
{
	// Get PxTransform.
	Eigen::Matrix4d eigenTransform = sphere.getTransform();
	physx::PxTransform transform = eigenMatrixToPxTransform(worldTransform * eigenTransform);

	// Create RigidDynamic kinematic actor.
	physx::PxRigidDynamic* actor = m_physics->createRigidDynamic(transform);
	actor->setRigidBodyFlag(physx::PxRigidBodyFlag::eKINEMATIC, true);
	//actor->setName(sphere.getName());
	physx::PxShape* shape = m_physics->createShape(getPxGeometry(sphere), *m_material, true);
	shape->setContactOffset(static_cast<physx::PxReal>(m_safetyDistance / 2));
	actor->attachShape(*shape);
	shape->release();

	// Add ContactPoint pointer in actor userData.
	actor->userData = sphere.m_contactPoint;

	// Add new actor to scene.
	m_scene->addActor(*actor);
	return actor;
}

// Add capsule collider to scene.
physx::PxRigidDynamic* PhysXEnv::createCollisionActor(Capsule& capsule, const Eigen::Matrix4d& worldTransform)
{
	// Get PxTransform.
	Eigen::Matrix4d eigenTransform = capsule.getTransform();
	physx::PxTransform transform = eigenMatrixToPxTransform(worldTransform * eigenTransform);

	// Create RigidDynamic kinematic actor.
	physx::PxRigidDynamic* actor = m_physics->createRigidDynamic(transform);
	actor->setRigidBodyFlag(physx::PxRigidBodyFlag::eKINEMATIC, true);
	//actor->setName(sphere.getName());
	physx::PxShape* shape = m_physics->createShape(getPxGeometry(capsule), *m_material, true);
	shape->setContactOffset(static_cast<physx::PxReal>(m_safetyDistance / 2));
	actor->attachShape(*shape);
	shape->release();

	// Add ContactPoint pointer in actor userData.
	actor->userData = capsule.m_contactPoint;

	// Add new actor to scene.
	m_scene->addActor(*actor);
	return actor;
}

// Add box collider to scene.
physx::PxRigidDynamic* PhysXEnv::createCollisionActor(Box& box, const Eigen::Matrix4d& worldTransform)
{
	// Get PxTransform.
	Eigen::Matrix4d eigenTransform = box.getTransform();
	physx::PxTransform transform = eigenMatrixToPxTransform(worldTransform * eigenTransform);

	// Create RigidDynamic kinematic actor.
	physx::PxRigidDynamic* actor = m_physics->createRigidDynamic(transform);
	actor->setRigidBodyFlag(physx::PxRigidBodyFlag::eKINEMATIC, true);
	//actor->setName(sphere.getName());
	physx::PxShape* shape = m_physics->createShape(getPxGeometry(box), *m_material, true);
	shape->setContactOffset(static_cast<physx::PxReal>(m_safetyDistance / 2));
	actor->attachShape(*shape);
	shape->release();

	// Add ContactPoint pointer in actor userData.
	actor->userData = box.m_contactPoint;

	// Add new actor to scene.
	m_scene->addActor(*actor);
	return actor;
}

// Add sphere obstacle to scene.
void PhysXEnv::createObstacleActor(const Sphere& sphere)
{
	// Get PxTransform.
	Eigen::Matrix4d eigenTransform = sphere.getTransform();
	physx::PxTransform transform = eigenMatrixToPxTransform(eigenTransform);

	// Create RigidStatic actor.
	physx::PxRigidStatic* actor = m_physics->createRigidStatic(transform);
	physx::PxShape* shape = m_physics->createShape(getPxGeometry(sphere), *m_material, true);
	shape->setContactOffset(static_cast<physx::PxReal>(m_safetyDistance / 2));
	actor->attachShape(*shape);
	shape->release();

	// Add new actor to scene.
	m_scene->addActor(*actor);
	m_obstacleActors.push_back(actor);
}

// Add capsule obstacle to scene.
void PhysXEnv::createObstacleActor(const Capsule& capsule)
{
	// Get PxTransform.
	Eigen::Matrix4d eigenTransform = capsule.getTransform();
	physx::PxTransform transform = eigenMatrixToPxTransform(eigenTransform);

	// Create RigidStatic actor.
	physx::PxRigidStatic* actor = m_physics->createRigidStatic(transform);
	physx::PxShape* shape = m_physics->createShape(getPxGeometry(capsule), *m_material, true);
	shape->setContactOffset(static_cast<physx::PxReal>(m_safetyDistance / 2));
	actor->attachShape(*shape);
	shape->release();

	// Add new actor to scene.
	m_scene->addActor(*actor);
	m_obstacleActors.push_back(actor);
}

// Add box obstacle to scene.
void PhysXEnv::createObstacleActor(const Box& box)
{
	// Get PxTransform.
	Eigen::Matrix4d eigenTransform = box.getTransform();
	physx::PxTransform transform = eigenMatrixToPxTransform(eigenTransform);

	// Create RigidStatic actor.
	physx::PxRigidStatic* actor = m_physics->createRigidStatic(transform);
	physx::PxShape* shape = m_physics->createShape(getPxGeometry(box), *m_material, true);
	shape->setContactOffset(static_cast<physx::PxReal>(m_safetyDistance / 2));
	actor->attachShape(*shape);
	shape->release();

	// Add new actor to scene.
	m_scene->addActor(*actor);
	m_obstacleActors.push_back(actor);
}

// Convert custom geometry types to PxGeometry types.
physx::PxSphereGeometry PhysXEnv::getPxGeometry(const Sphere& sphere)
{
	return physx::PxSphereGeometry(static_cast<physx::PxReal>(sphere.m_radius));
}

physx::PxCapsuleGeometry PhysXEnv::getPxGeometry(const Capsule& capsule)
{
	return physx::PxCapsuleGeometry(static_cast<physx::PxReal>(capsule.m_radius),
		static_cast<physx::PxReal>(capsule.m_halfHeight));
}

physx::PxBoxGeometry PhysXEnv::getPxGeometry(const Box& box)
{
	return physx::PxBoxGeometry(static_cast<physx::PxReal>(box.m_radii(0)),
		static_cast<physx::PxReal>(box.m_radii(1)),
		static_cast<physx::PxReal>(box.m_radii(2)));
}

// Convert Eigen::Matrix4d to PxTransform.
physx::PxTransform PhysXEnv::eigenMatrixToPxTransform(const Eigen::Matrix4d& eigenMatrix)
{
	float values[16];
	for (size_t i = 0; i < 16; i++)
	{
		values[i] = static_cast<float>(eigenMatrix(i));
	}
	physx::PxMat44 physxMatrix(values);
	return physx::PxTransform(physxMatrix);
}

// Run simulation to generate contacts.
void PhysXEnv::simulate()
{
	m_scene->simulate(0.01f);
	m_scene->fetchResults(true);
}
