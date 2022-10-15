#include "Sphere.h"
#include "Capsule.h"
#include "Box.h"
#include "FrankaPanda.h"
#include "ObjectType.h"
#include "PhysicsCore.h"
#include "PhysicsScene.h"
#include <iostream>
#include <Eigen/Dense>
#include <chrono>

void testFrankaPanda();
void testFrankaPanda2();

int main()
{
	// Round outputs to console.
	std::cout.precision(4);

	// Reset seed.
	srand(static_cast<unsigned int>(time(nullptr)));
	rand();

	// Test planner.
	//testFrankaPanda();
	testFrankaPanda2();

	return 0;
}

void testFrankaPanda()
{
	constexpr double pi = 3.14159265358979323846;

	// Create physics core and scene.
	MotionPlanner::PhysicsCore physics;
	physics.createPhysicsCore();
	MotionPlanner::PhysicsScene* physicsScene = physics.createPhysicsScene("MyTestScene");

	// Obstacles.
	MotionPlanner::ObjectType objectType = MotionPlanner::ObjectType::Obstacle;
	Eigen::Vector3d position(-1.70, 0.00, 0.352104);
	Eigen::Vector3d orientation = Eigen::Vector3d::Zero();
	Eigen::Vector3d halfExtents(0.615405, 0.615405, 0.351040);
	MotionPlanner::Box myBox(position, orientation, halfExtents, "MyBox", objectType);
	physicsScene->addObstacle(myBox);

	// Create robot.
	Eigen::Matrix4d pandaBaseTransform{
		{1, 0, 0, -2.581200},
		{0, 1, 0,  0.000000},
		{0, 0, 1,  0.383670},
		{0, 0, 0,  1.000000}
	};
	MotionPlanner::FrankaPanda panda(pandaBaseTransform);

	// Add robot to the scene.
	physicsScene->addSpatialManipulator(panda);

	// Setup start joint angles and transform.
	Eigen::Vector<double, 7> startAngles(0, 0, 0, -pi / 2, 0, pi / 2, 0);
	panda.setJointDisplacements(startAngles);
	Eigen::Matrix4d startTransform = panda.getEndFrameSpatialTransform();

	// Setup goal poses.
	Eigen::Matrix4d goalTransform1{
		{1,  0,  0,  0.081200},
		{0, -1,  0, -0.700000},
		{0,  0, -1,  0.516330},
		{0,  0,  0,  1.000000}
	};
	Eigen::Matrix4d goalTransform2{
		{1,  0,  0,  0.681200},
		{0, -1,  0,  0.000000},
		{0,  0, -1,  0.616330},
		{0,  0,  0,  1.000000}
	};
	Eigen::Matrix4d goalTransform3{
		{1,  0,  0,  0.081200},
		{0, -1,  0,  0.700000},
		{0,  0, -1,  0.516330},
		{0,  0,  0,  1.000000}
	};

	// Generate motion plan.
	auto start = std::chrono::steady_clock::now();
	panda.motionPlan(goalTransform1);
	Eigen::Matrix4d achievedTransform1 = panda.getEndFrameSpatialTransform();
	panda.motionPlan(goalTransform2);
	Eigen::Matrix4d achievedTransform2 = panda.getEndFrameSpatialTransform();
	panda.motionPlan(goalTransform3);
	Eigen::Matrix4d achievedTransform3 = panda.getEndFrameSpatialTransform();
	auto stop = std::chrono::steady_clock::now();

	std::cout << "Start Transform 1: \n" << startTransform << "\n\n";
	std::cout << "Goal Transform 1: \n" << goalTransform1 << "\n\n";
	std::cout << "Acheived Transform 1: \n" << achievedTransform1 << "\n\n\n";

	std::cout << "Start Transform 2: \n" << achievedTransform1 << "\n\n";
	std::cout << "Goal Transform 2: \n" << goalTransform2 << "\n\n";
	std::cout << "Acheived Transform 2: \n" << achievedTransform2 << "\n\n\n";

	std::cout << "Start Transform 3: \n" << achievedTransform2 << "\n\n";
	std::cout << "Goal Transform 3: \n" << goalTransform3 << "\n\n";
	std::cout << "Acheived Transform 3: \n" << achievedTransform3 << "\n\n\n";

	// Elapsed time.
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();
	std::cout << "Elapsed Time: " << elapsedTime << " ms\n";
}

void testFrankaPanda2()
{
	constexpr double pi = 3.14159265358979323846;

	// Create physics core and scene.
	MotionPlanner::PhysicsCore physics;
	physics.createPhysicsCore();
	MotionPlanner::PhysicsScene* physicsScene = physics.createPhysicsScene("MyTestScene");

	// Obstacles.
	MotionPlanner::ObjectType objectType = MotionPlanner::ObjectType::Obstacle;
	//Eigen::Vector3d position(0.35, -0.2, 0.70); //overflow/underflow
	//Eigen::Vector3d position(0.18, -0.2, 0.60); // overflow/underflow
	//Eigen::Vector3d position(0.35, -0.25, 0.55); // overflow/underflow
	Eigen::Vector3d position(-0.15, 0, 0.63); // good one
	Eigen::Vector3d orientation = Eigen::Vector3d::Zero();
	Eigen::Vector3d halfExtents(0.05, 0.05, 0.05);
	MotionPlanner::Box myBox(position, orientation, halfExtents, "MyBox", objectType);
	MotionPlanner::Sphere mySphere(position, orientation, 0.05, "MySphere", objectType);
	physicsScene->addObstacle(mySphere);
	//physicsScene->addObstacle(myBox);

	// Create robot.
	Eigen::Matrix4d pandaBaseTransform{
		{1, 0, 0, -0},
		{0, 1, 0,  0.000000},
		{0, 0, 1,  0},
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
	Eigen::Matrix4d goalTransform1{
		{1,  0,  0,  0.00000},
		{0, -1,  0, -0.70000},
		{0,  0, -1,  0.4},
		{0,  0,  0,  1.00000}
	};
	Eigen::Matrix4d goalTransform2{
		{1,  0,  0,  0.58120},
		{0,  0,  1,  0.30000},
		{0, -1,  0,  0.51633},
		{0,  0,  0,  1.00000}
	};
	Eigen::Matrix4d goalTransform3{
		{0,  1,  0,  0.63120},
		{1,  0,  0, -0.00000},
		{0,  0,  1,  0.56633},
		{0,  0,  0,  1.00000}
	};

	// Generate motion plan.
	auto start = std::chrono::steady_clock::now();
	panda.motionPlan(goalTransform1);
	Eigen::Matrix4d achievedTransform1 = panda.getEndFrameSpatialTransform();
	//panda.motionPlan(goalTransform2);
	Eigen::Matrix4d achievedTransform2 = panda.getEndFrameSpatialTransform();
	//panda.motionPlan(goalTransform3);
	Eigen::Matrix4d achievedTransform3 = panda.getEndFrameSpatialTransform();
	auto stop = std::chrono::steady_clock::now();

	std::cout << "Start Transform 1: \n" << startTransform << "\n\n";
	std::cout << "Goal Transform 1: \n" << goalTransform1 << "\n\n";
	std::cout << "Acheived Transform 1: \n" << achievedTransform1 << "\n\n\n";

	std::cout << "Start Transform 2: \n" << achievedTransform1 << "\n\n";
	std::cout << "Goal Transform 2: \n" << goalTransform2 << "\n\n";
	std::cout << "Acheived Transform 2: \n" << achievedTransform2 << "\n\n\n";

	std::cout << "Start Transform 3: \n" << achievedTransform2 << "\n\n";
	std::cout << "Goal Transform 3: \n" << goalTransform3 << "\n\n";
	std::cout << "Acheived Transform 3: \n" << achievedTransform3 << "\n\n\n";

	// Elapsed time.
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();
	std::cout << "Elapsed Time: " << elapsedTime << " ms\n";
}
