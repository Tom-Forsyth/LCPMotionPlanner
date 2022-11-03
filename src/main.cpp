#include "Sphere.h"
#include "Capsule.h"
#include "Box.h"
#include "FrankaPanda.h"
#include "ObjectType.h"
#include "PhysicsCore.h"
#include "PhysicsScene.h"
#include "MotionPlanResults.h"
#include <iostream>
#include <Eigen/Dense>
#include <chrono>

void testFrankaPanda();
void testFrankaPanda2();
void displayPlanResults(const MotionPlanner::MotionPlanResults& planResults, int n);

int main()
{
	// Round outputs to console.
	std::cout.precision(4);

	// Reset seed.
	srand(static_cast<unsigned int>(time(nullptr)));
	rand();

	// Test planner.
	testFrankaPanda();
	//testFrankaPanda2();

	return 0;
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
	Eigen::Vector3d tableOffsets(0.3, 0.65, 0.02);
	double legLength = tableOrigin(2) - tableOffsets(2);
	Eigen::Vector3d legOffsets(0.03, 0.03, legLength / 2);
	double legZVal = legLength / 2;
	MotionPlanner::ObjectType tableObjectType = MotionPlanner::ObjectType::Obstacle;
	Eigen::Vector3d zeroVec = Eigen::Vector3d::Zero();

	// Create table top and legs.
	Eigen::Vector3d legOrigin1 = Eigen::Vector3d(1.5 - 0.3 + legOffsets(0), 1 - 0.5 + legOffsets(1), legZVal);
	Eigen::Vector3d legOrigin2 = Eigen::Vector3d(1.5 - 0.3 + legOffsets(0), 1 + 0.5 - legOffsets(1), legZVal);
	Eigen::Vector3d legOrigin3 = Eigen::Vector3d(1.5 + 0.3 - legOffsets(0), 1 - 0.5 + legOffsets(1), legZVal);
	Eigen::Vector3d legOrigin4 = Eigen::Vector3d(1.5 + 0.3 - legOffsets(0), 1 + 0.5 - legOffsets(1), legZVal);
	MotionPlanner::Box tableTop(tableOrigin, zeroVec, tableOffsets, "Table Top", tableObjectType);
	MotionPlanner::Box tableLeg1(legOrigin1, zeroVec, legOffsets, "Table Leg 1", tableObjectType);
	MotionPlanner::Box tableLeg2(legOrigin2, zeroVec, legOffsets, "Table Leg 2", tableObjectType);
	MotionPlanner::Box tableLeg3(legOrigin3, zeroVec, legOffsets, "Table Leg 3", tableObjectType);
	MotionPlanner::Box tableLeg4(legOrigin4, zeroVec, legOffsets, "Table Leg 4", tableObjectType);

	// Add table to the scene.
	physicsScene->addObstacle(tableTop);
	physicsScene->addObstacle(tableLeg1);
	physicsScene->addObstacle(tableLeg2);
	physicsScene->addObstacle(tableLeg3);
	physicsScene->addObstacle(tableLeg4);

	// Obstacles on table.
	MotionPlanner::Sphere sphereObstacle(Eigen::Vector3d(1.5, 0.75, 0.85), Eigen::Vector3d::Zero(), 0.12, "Sphere Obstacle", MotionPlanner::ObjectType::Obstacle);
	MotionPlanner::Capsule capsuleObstacle(Eigen::Vector3d(1.3, 1.25, 0.55), Eigen::Vector3d(0, pi/2, 0), 0.2, 0.05, "Capsule Obstacle", MotionPlanner::ObjectType::Obstacle);
	MotionPlanner::Box boxObstacle(Eigen::Vector3d(1.8, 1, 0.45), Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0.05, 0.2, 0.2), "Box Obstacle", MotionPlanner::ObjectType::Obstacle);
	physicsScene->addObstacle(sphereObstacle);
	physicsScene->addObstacle(capsuleObstacle);
	physicsScene->addObstacle(boxObstacle);

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
	static_cast<void>(panda.setJointDisplacements(startAngles));
	Eigen::Matrix4d startTransform = panda.getEndFrameSpatialTransform();

	// Setup goal poses.
	Eigen::Matrix4d goalTransform1(startTransform);
	Eigen::Matrix4d goalTransform2(startTransform);
	goalTransform1.block(0, 3, 3, 1) = Eigen::Vector3d(0.6, -0.35, 0.45);
	goalTransform2.block(0, 3, 3, 1) = Eigen::Vector3d(0.6, 0.35, 0.45);

	// Generate motion plan.
	auto start = std::chrono::steady_clock::now();
	MotionPlanner::MotionPlanResults planResults1 = panda.motionPlan(goalTransform1);
	MotionPlanner::MotionPlanResults planResults2 = panda.motionPlan(goalTransform2);
	auto stop = std::chrono::steady_clock::now();

	// Display results.
	displayPlanResults(planResults1, 1);
	displayPlanResults(planResults2, 2);

	// Elapsed time.
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();
	std::cout << "Elapsed Time: " << elapsedTime << " ms\n";
}

void testFrankaPanda2()
{
	/*
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
	*/
}

void displayPlanResults(const MotionPlanner::MotionPlanResults& planResults, int n)
{
	std::cout << "----- Motion Plan " << n << " Results -----\n";
	std::cout << "Exit Code: " << planResults.exitCode << "\n";
	std::cout << "Plan Size: " << planResults.motionPlan.size() << "\n";
	std::cout << "Start Pose: \n" << planResults.startPose << "\n";
	std::cout << "Goal Pose: \n" << planResults.goalPose << "\n";
	std::cout << "Achieved Pose: \n" << planResults.achievedPose << "\n\n";
}

/*
To-Do:
  1. Create local planner exit info struct with exit code, iterations, plan vector, achieved pose, etc.
  2. Store jacobian rather than storing the end frame.
  3. Clean up main local planner loop.
*/
