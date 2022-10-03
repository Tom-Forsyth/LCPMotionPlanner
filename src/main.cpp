#include "Sphere.h"
#include "Capsule.h"
#include "Box.h"
#include "FrankaPanda.h"
#include "ObjectType.h"
#include "PhysicsCore.h"
#include "PhysicsScene.h"
#include <iostream>
#include <Eigen/Dense>
#include "PxPhysicsAPI.h"
#include <chrono>

void testFrankaPanda();
void testCMake();

int main()
{
	// Round outputs to console.
	std::cout.precision(4);

	// Reset seed.
	srand(static_cast<unsigned int>(time(nullptr)));
	rand();

	// Test planner.
	testFrankaPanda();
	//testCMake();

	return 0;
}

void testCMake()
{
	MotionPlanner::Box tableTop(Eigen::Vector3d::Random(), Eigen::Vector3d::Random(), Eigen::Vector3d::Random(), "MyBox", MotionPlanner::ObjectType::Visual);
	std::cout << tableTop.getTransform() << "\n";

	physx::PxVec3 testVec = {1, 2, 3};
	std::cout << testVec[0] + testVec[1] + testVec[2] << "\n";
}

void testFrankaPanda()
{
	constexpr double pi = 3.14159265358979323846;

	// Create physics core and scene.
	MotionPlanner::PhysicsCore physics;
	physics.createPhysicsCore();
	MotionPlanner::PhysicsScene* physicsScene = physics.createPhysicsScene("MyTestScene");

	// Table obstacle parameters.
	Eigen::Vector3d tableOrigin(1.5, 1, 0.25);
	Eigen::Vector3d tableOffsets(0.3, 0.5, 0.02);
	double legLength = tableOrigin(2) - tableOffsets(2);
	Eigen::Vector3d legOffsets(0.03, 0.03, legLength / 2);
	double legZVal = legLength / 2;
	MotionPlanner::ObjectType tableObjectType = MotionPlanner::ObjectType::Obstacle;

	// Create table top and legs.
	MotionPlanner::Box tableTop(tableOrigin, Eigen::Vector3d(0, 0, 0), tableOffsets, "Table Top", tableObjectType);
	MotionPlanner::Box tableLeg1(Eigen::Vector3d(1.5 - 0.3 + legOffsets(0), 1 - 0.5 + legOffsets(1), legZVal), Eigen::Vector3d(0, 0, 0), legOffsets, "Table Leg 1", tableObjectType);
	MotionPlanner::Box tableLeg2(Eigen::Vector3d(1.5 - 0.3 + legOffsets(0), 1 + 0.5 - legOffsets(1), legZVal), Eigen::Vector3d(0, 0, 0), legOffsets, "Table Leg 2", tableObjectType);
	MotionPlanner::Box tableLeg3(Eigen::Vector3d(1.5 + 0.3 - legOffsets(0), 1 - 0.5 + legOffsets(1), legZVal), Eigen::Vector3d(0, 0, 0), legOffsets, "Table Leg 3", tableObjectType);
	MotionPlanner::Box tableLeg4(Eigen::Vector3d(1.5 + 0.3 - legOffsets(0), 1 + 0.5 - legOffsets(1), legZVal), Eigen::Vector3d(0, 0, 0), legOffsets, "Table Leg 4", tableObjectType);

	// Add table to the scene.
	physicsScene->addObstacle(tableTop);
	physicsScene->addObstacle(tableLeg1);
	physicsScene->addObstacle(tableLeg2);
	physicsScene->addObstacle(tableLeg3);
	physicsScene->addObstacle(tableLeg4);

	// Obstacles on table.
	MotionPlanner::Box boxObstacle(Eigen::Vector3d(1.5, 1, 0.35), Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0.1, 0.1, 0.1), "Box Obstacle", tableObjectType);
	MotionPlanner::Sphere object1(Eigen::Vector3d(1.65, 0.65, 0.32), Eigen::Vector3d(0, 0, 0), 0.05, "Object 1", tableObjectType);
	MotionPlanner::Sphere object2(Eigen::Vector3d(1.65, 1.35, 0.32), Eigen::Vector3d(0, 0, 0), 0.05, "Object 2", tableObjectType);
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
	MotionPlanner::FrankaPanda panda(pandaBaseTransform);

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
