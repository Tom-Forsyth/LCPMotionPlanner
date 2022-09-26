#include <iostream>
#include <Eigen/Dense>
#include "PxPhysicsAPI.h"
#include <chrono>

#include "Sphere.h"
#include "Capsule.h"
#include "Box.h"

#include "FrankaPanda.h"
#include "ObjectType.h"

#include "PhysicsCore.h"
#include "PhysicsScene.h"

using namespace CollisionAvoidance;

void testMotionPlan();
void testPhysics();
void testPhysics2();

int main()
{
	// Round outputs to console.
	std::cout.precision(4);

	// Reset seed.
	srand(static_cast<unsigned int>(time(nullptr)));
	rand();

	// Test planner.
	//testMotionPlan();
	//testPhysics();
	testPhysics2();

	return 0;
}

void testMotionPlan()
{
	/*
	constexpr double pi = 3.14159265358979323846;

	// Create robot.
	Eigen::Matrix4d pandaBaseTransform{
		{1, 0, 0, 1},
		{0, 1, 0, 1},
		{0, 0, 1, 0},
		{0, 0, 0, 1}
	};
	FrankaPanda panda(pandaBaseTransform);

	// Table obstacle parameters.
	Eigen::Vector3d tableOrigin(1.5, 1, 0.25);
	Eigen::Vector3d tableOffsets(0.3, 0.5, 0.02);
	double legLength = tableOrigin(2) - tableOffsets(2);
	Eigen::Vector3d legOffsets(0.03, 0.03, legLength/2);
	double legZVal = legLength / 2;

	// Table top.
	ObjectType tableObjectType = ObjectType::Obstacle;
	Box tableTop(tableOrigin, Eigen::Vector3d(0, 0, 0), tableOffsets, "Table Top", tableObjectType);
	panda.addObstacle(tableTop);

	// Table legs.
	Box tableLeg1(Eigen::Vector3d(1.5 - 0.3 + legOffsets(0), 1 - 0.5 + legOffsets(1), legZVal), Eigen::Vector3d(0, 0, 0), legOffsets, "Table Leg 1", tableObjectType);
	panda.addObstacle(tableLeg1);
	Box tableLeg2(Eigen::Vector3d(1.5 - 0.3 + legOffsets(0), 1 + 0.5 - legOffsets(1), legZVal), Eigen::Vector3d(0, 0, 0), legOffsets, "Table Leg 2", tableObjectType);
	panda.addObstacle(tableLeg2);
	Box tableLeg3(Eigen::Vector3d(1.5 + 0.3 - legOffsets(0), 1 - 0.5 + legOffsets(1), legZVal), Eigen::Vector3d(0, 0, 0), legOffsets, "Table Leg 3", tableObjectType);
	panda.addObstacle(tableLeg3);
	Box tableLeg4(Eigen::Vector3d(1.5 + 0.3 - legOffsets(0), 1 + 0.5 - legOffsets(1), legZVal), Eigen::Vector3d(0, 0, 0), legOffsets, "Table Leg 4", tableObjectType);
	panda.addObstacle(tableLeg4);

	// Obstacle on table.
	Box boxObstacle(Eigen::Vector3d(1.5, 1, 0.35), Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0.1, 0.1, 0.1), "Box Obstacle", tableObjectType);
	panda.addObstacle(boxObstacle);

	// Objects on table to pickup.
	Sphere object1(Eigen::Vector3d(1.65, 0.65, 0.32), Eigen::Vector3d(0, 0, 0), 0.05, "Object 1", tableObjectType);
	panda.addObstacle(object1);
	Sphere object2(Eigen::Vector3d(1.65, 1.35, 0.32), Eigen::Vector3d(0, 0, 0), 0.05, "Object 2", tableObjectType);
	panda.addObstacle(object2);

	// Setup start joint angles and transform.
	Eigen::Vector<double, 7> startAngles(0, 0, 0, -pi / 2, 0, pi / 2, 0);
	panda.setJointDisplacements(startAngles);
	Eigen::Matrix4d startTransform = panda.getEndFrameSpatialTransform();

	// Setup goal poses.
	Eigen::Matrix4d goalTransform1(startTransform);
	Eigen::Matrix4d goalTransform2(startTransform);
	const int plan = 0;

	// Pure translation.
	if (plan == 0)
	{
		goalTransform1.block(0, 3, 3, 1) = Eigen::Vector3d(0.65, -0.35, 0.4);
		goalTransform2.block(0, 3, 3, 1) = Eigen::Vector3d(0.65, 0.35, 0.4);
	}

	// Pure rotation.
	if (plan == 1)
	{
		goalTransform1.block(0, 0, 3, 3) = Eigen::Matrix3d{
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1}
		};
		goalTransform2.block(0, 0, 3, 3) = Eigen::Matrix3d{
			{1, 0,  0},
			{0, 0, -1},
			{0, 1,  0}
		};
	}

	// Generate motion plan.
	auto start = std::chrono::steady_clock::now();
	panda.motionPlan(goalTransform1);
	Eigen::Matrix4d achievedTransform1 = panda.getEndFrameSpatialTransform();
	panda.motionPlan(goalTransform2);
	Eigen::Matrix4d achievedTransform2 = panda.getEndFrameSpatialTransform();
	auto stop = std::chrono::steady_clock::now();

	std::cout << "Start Transform: \n" << startTransform << "\n\n";
	std::cout << "Goal Transform 1: \n" << goalTransform1 << "\n\n";
	std::cout << "Acheived Transform 1: \n" << achievedTransform1 << "\n\n";
	std::cout << "Goal Transform 2: \n" << goalTransform2 << "\n\n";
	std::cout << "Acheived Transform 2: \n" << achievedTransform2 << "\n\n";

	// Elapsed time.
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();
	std::cout << "Elapsed Time: " << elapsedTime << " ms" << std::endl;
	*/
}

void testPhysics()
{
	/*
	// Create physics core.
	PhysicsCore physics;
	physics.createPhysicsCore();

	auto start = std::chrono::steady_clock::now();

	// Test time to create scene.
	PhysicsScene* physicsScene = physics.createPhysicsScene("MyTestScene");

	// Add obstacle.
	Sphere mySphere(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), 0.5, "MySphere", ObjectType::Obstacle);
	Capsule myCapsule(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(1, 1, 1), 0.5, 0.5, "MyCapsule", ObjectType::Obstacle);
	Box myBox(Eigen::Vector3d(2, 2, 2), Eigen::Vector3d(2, 2, 2), Eigen::Vector3d(0.5, 0.5, 0.5), "MyBox", ObjectType::Obstacle);
	physicsScene->addObstacle(mySphere);
	physicsScene->addObstacle(myCapsule);
	physicsScene->addObstacle(myBox);

	auto stop = std::chrono::steady_clock::now();

	// Simulate.
	const int iterations = 1000;
	for (int i = 0; i < iterations; i++)
	{
		physicsScene->simulate();
	}

	// Elapsed time.
	auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count();
	std::cout << "Elapsed Time: " << elapsedTime << " us" << std::endl;
	*/
}

void testPhysics2()
{
	constexpr double pi = 3.14159265358979323846;

	// Create physics core and scene.
	PhysicsCore physics;
	physics.createPhysicsCore();
	PhysicsScene* physicsScene = physics.createPhysicsScene("MyTestScene");

	// Table obstacle parameters.
	Eigen::Vector3d tableOrigin(1.5, 1, 0.25);
	Eigen::Vector3d tableOffsets(0.3, 0.5, 0.02);
	double legLength = tableOrigin(2) - tableOffsets(2);
	Eigen::Vector3d legOffsets(0.03, 0.03, legLength / 2);
	double legZVal = legLength / 2;
	ObjectType tableObjectType = ObjectType::Obstacle;

	// Create table top and legs.
	Box tableTop(tableOrigin, Eigen::Vector3d(0, 0, 0), tableOffsets, "Table Top", tableObjectType);
	Box tableLeg1(Eigen::Vector3d(1.5 - 0.3 + legOffsets(0), 1 - 0.5 + legOffsets(1), legZVal), Eigen::Vector3d(0, 0, 0), legOffsets, "Table Leg 1", tableObjectType);
	Box tableLeg2(Eigen::Vector3d(1.5 - 0.3 + legOffsets(0), 1 + 0.5 - legOffsets(1), legZVal), Eigen::Vector3d(0, 0, 0), legOffsets, "Table Leg 2", tableObjectType);
	Box tableLeg3(Eigen::Vector3d(1.5 + 0.3 - legOffsets(0), 1 - 0.5 + legOffsets(1), legZVal), Eigen::Vector3d(0, 0, 0), legOffsets, "Table Leg 3", tableObjectType);
	Box tableLeg4(Eigen::Vector3d(1.5 + 0.3 - legOffsets(0), 1 + 0.5 - legOffsets(1), legZVal), Eigen::Vector3d(0, 0, 0), legOffsets, "Table Leg 4", tableObjectType);

	// Add table to the scene.
	physicsScene->addObstacle(tableTop);
	physicsScene->addObstacle(tableLeg1);
	physicsScene->addObstacle(tableLeg2);
	physicsScene->addObstacle(tableLeg3);
	physicsScene->addObstacle(tableLeg4);

	// Obstacles on table.
	Box boxObstacle(Eigen::Vector3d(1.5, 1, 0.35), Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0.1, 0.1, 0.1), "Box Obstacle", tableObjectType);
	Sphere object1(Eigen::Vector3d(1.65, 0.65, 0.32), Eigen::Vector3d(0, 0, 0), 0.05, "Object 1", tableObjectType);
	Sphere object2(Eigen::Vector3d(1.65, 1.35, 0.32), Eigen::Vector3d(0, 0, 0), 0.05, "Object 2", tableObjectType);
	physicsScene->addObstacle(boxObstacle);
	physicsScene->addObstacle(object1);
	physicsScene->addObstacle(object2);

	// Create robot.
	Eigen::Matrix4d pandaBaseTransform{
		{1, 0, 0, 1},
		{0, 1, 0, 1},
		{0, 0, 1, 0},
		{0, 0, 0, 1}
	};
	FrankaPanda panda(pandaBaseTransform);

	// Add robot to the scene.
	physicsScene->addSpatialManipulator(panda);

	// Setup start joint angles and transform.
	Eigen::Vector<double, 7> startAngles(0, 0, 0, -pi / 2, 0, pi / 2, 0);
	panda.setJointDisplacements(startAngles);
	Eigen::Matrix4d startTransform = panda.getEndFrameSpatialTransform();

	// Setup goal poses.
	Eigen::Matrix4d goalTransform1(startTransform);
	Eigen::Matrix4d goalTransform2(startTransform);
	const int plan = 0;

	// Pure translation.
	if (plan == 0)
	{
		goalTransform1.block(0, 3, 3, 1) = Eigen::Vector3d(0.65, -0.35, 0.4);
		goalTransform2.block(0, 3, 3, 1) = Eigen::Vector3d(0.65, 0.35, 0.4);
	}

	// Pure rotation.
	if (plan == 1)
	{
		goalTransform1.block(0, 0, 3, 3) = Eigen::Matrix3d{
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1}
		};
		goalTransform2.block(0, 0, 3, 3) = Eigen::Matrix3d{
			{1, 0,  0},
			{0, 0, -1},
			{0, 1,  0}
		};
	}

	// Generate motion plan.
	auto start = std::chrono::steady_clock::now();
	panda.motionPlan(goalTransform1);
	Eigen::Matrix4d achievedTransform1 = panda.getEndFrameSpatialTransform();
	panda.motionPlan(goalTransform2);
	Eigen::Matrix4d achievedTransform2 = panda.getEndFrameSpatialTransform();
	auto stop = std::chrono::steady_clock::now();

	std::cout << "Start Transform: \n" << startTransform << "\n\n";
	std::cout << "Goal Transform 1: \n" << goalTransform1 << "\n\n";
	std::cout << "Acheived Transform 1: \n" << achievedTransform1 << "\n\n";
	std::cout << "Goal Transform 2: \n" << goalTransform2 << "\n\n";
	std::cout << "Acheived Transform 2: \n" << achievedTransform2 << "\n\n";

	// Elapsed time.
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();
	std::cout << "Elapsed Time: " << elapsedTime << " ms\n";
}


// TODO: 
	// Contacts need to be sorted by movable rigid bodies, not just rigid bodies.

// TODO: Add logic to demote contact points of rigid bodies to the previous body in the chain.
			// A comparison of distances similar to the contact manager must be made.
			// Must handle edge cases. IE: First body is always fixed, and has no previous body.



/*
* Actor Storage:
*	Obstacles: Store in regular vector, or in name-pointer map.
*	Robot Geometry: Store in shape name and pointer map.
*		Shape name should be prefixed with the robot name and link name.
*			Ex: "panda_link_6_sphere_2"
*		RigidBody name should be the robot name and link name.
*			Ex: "panda_link_6"
* 
* Contacts:
*	Create a ContactManager class that will have a map that stores ContactPoint objects with a name key that belongs to the actor/shape.
*	This allows us to remove ContactPoint from Shape/RigidBody.
*	Also allows us to perform sorting of ContactPoints from this class.
*	ContactManager will be a member of SpatialManipulator.
*	Each PxShape will contain a pointer to ContactManager.
*	In ContactReportCallback::onContact(), 
*		ContactManager* contactManager = pxShape.userData... 
*		if (contactManager)
*		{
*			std::string shapeName = shape.getName();
*			ContactPoint contactPoint = ....
*			contactManager->addContact(shapeName, contactPoint);
*		}
*	Then, we can write logic inside of ContactManager to sort and select contact points to give to the RigidBody.
*
* Shape actor transforms:
*	Obstacle actors: No problem.
*	Robot geometry actors: Yes problem.
*		Shape's transforms should be managed by my code independant of PhysX.
*		PhysicsScene will have a map that holds pointers to the original shapes.
*		PhysicsScene::updateTransforms() will update the PhysX actor transforms with my shape transforms using above map.
*		Will also need to change robot collision shapes from having local transforms to world/spatial transforms.
*			In RigidBody::addCollider(), we assume the shapes being added are relative to the body. We can maintain this.
*			Create CollisionAggregate::updateWorldTransforms(const Eigen::Matrix4d& worldTransform).
*				This will multiply each Shape's transform by the supplied worldTransform and store it in a map inside CollisionAggregate.
*				PhysicsScene::addRobotGeometry() will store a pointer to this world transform in the PxShape's user data.
*
* Getters:
*	Getters for the structure of SpatialManipulator should be const references.
*		Ex: getRigidBodyChain() should ensure that we...
*			1. Only can read the data.
*			2. Do not have to copy the data.
*	Any manipulation of the data inside should be controlled by the classes themselves.
* 
* Logging:
*	Replace the exceptions/print statements with a log.
*/
