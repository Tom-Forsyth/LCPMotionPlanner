#pragma once

#include "PxPhysicsAPI.h"
#include "ContactReportCallback.h"

#include "Sphere.h"
#include "Capsule.h"
#include "Box.h"
#include "CollisionAggregate.h"
#include "RigidBodyChain.h"

#include <vector>
#include <Eigen/Dense>

class RigidBodyChain;

class PhysXEnv
{
private:
	// PhysX parameters.
	physx::PxDefaultAllocator m_allocator;
	physx::PxDefaultErrorCallback m_errorCallback;
	physx::PxTolerancesScale m_toleranceScale;
	physx::PxFoundation* m_foundation = NULL;
	physx::PxPhysics* m_physics = NULL;
	physx::PxDefaultCpuDispatcher* m_dispatcher = NULL;
	physx::PxScene* m_scene = NULL;
	physx::PxMaterial* m_material = NULL;
	physx::PxPvd* m_pvd = NULL;

	// Contact reporting callback.
	ContactReportCallback m_contactReportCallback;

	// Saftey distance to start generating collisions.
	const double m_safetyDistance = 0.05;

	// Rigid body chain pointer.
	RigidBodyChain* m_rigidBodyChain;

	// Collision actors in each rigid body of the chain.
	std::vector<std::vector<physx::PxRigidDynamic*>> m_collisionActors;

	// Local transforms of all the collision actors.
	std::vector<std::vector<Eigen::Matrix4d>> m_collisionActorLocalTransforms;

	// Obstacle actors.
	std::vector<physx::PxRigidStatic*> m_obstacleActors;



public:
	// Constructor.
	PhysXEnv();

	// Setup PhysXScene
	void init();

	// Set rigid body chain pointer.
	void setRigidBodyChain(RigidBodyChain* rigidBodyChain);

	// Load colliders of the rigid body chain into the scene.
	void initColliders();

	// Update the pose of the colliders.
	void updateTransforms();

	// Run simulation to generate contacts.
	void simulate();

	// Add robot's collision aggregate to the scene.
	void addCollisionAggregate(const RigidBody& rigidBody);

	// Convert custom geometry types to PxGeometry types.
	physx::PxSphereGeometry getPxGeometry(const Sphere& sphere);
	physx::PxCapsuleGeometry getPxGeometry(const Capsule& capsule);
	physx::PxBoxGeometry getPxGeometry(const Box& box);

	// Add robot geometry as RigidDynamic kinematic actors.
	physx::PxRigidDynamic* createCollisionActor(Sphere& sphere, const Eigen::Matrix4d& worldTransform);
	physx::PxRigidDynamic* createCollisionActor(Capsule& capsule, const Eigen::Matrix4d& worldTransform);
	physx::PxRigidDynamic* createCollisionActor(Box& box, const Eigen::Matrix4d& worldTransform);

	// Add obstacles as RigidStatic actors.
	void createObstacleActor(const Sphere& sphere);
	void createObstacleActor(const Capsule& capsule);
	void createObstacleActor(const Box& box);

	// Helper conversion function.
	physx::PxTransform eigenMatrixToPxTransform(const Eigen::Matrix4d& eigenMatrix);
};
