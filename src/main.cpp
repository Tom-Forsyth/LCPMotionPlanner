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

void testFrankaPanda();
void displayPlanResults(const MotionPlanner::MotionPlanResults& planResults, int n);

int main()
{
	// Round outputs to console.
	std::cout.precision(4);

	// Test planner.
	testFrankaPanda();

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
	MotionPlanner::Sphere sphereObstacle2(Eigen::Vector3d(1.55, 1, 0.35), Eigen::Vector3d::Zero(), 0.075, "Sphere Obstacle", MotionPlanner::ObjectType::Obstacle);
	MotionPlanner::Capsule capsuleObstacle(Eigen::Vector3d(1.3, 1.25, 0.55), Eigen::Vector3d(0, pi/2, 0), 0.2, 0.05, "Capsule Obstacle", MotionPlanner::ObjectType::Obstacle);
	MotionPlanner::Box boxObstacle(Eigen::Vector3d(1.8, 1, 0.45), Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0.05, 0.2, 0.2), "Box Obstacle", MotionPlanner::ObjectType::Obstacle);
	//physicsScene->addObstacle(sphereObstacle);
	physicsScene->addObstacle(capsuleObstacle);
	//physicsScene->addObstacle(boxObstacle);
	physicsScene->addObstacle(sphereObstacle2);

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
	MotionPlanner::MotionPlanResults planResults1 = panda.motionPlan(goalTransform1);
	MotionPlanner::MotionPlanResults planResults2 = panda.motionPlan(goalTransform2);

	// Display results.
	displayPlanResults(planResults1, 1);
	displayPlanResults(planResults2, 2);
}

void displayPlanResults(const MotionPlanner::MotionPlanResults& planResults, int n)
{
	std::cout << "----- Motion Plan " << n << " Results -----\n";
	std::cout << "Exit Code: " << planResults.exitCode << "\n";
	std::cout << "Compute Time: " << planResults.computeTimeMilli << " ms\n";
	std::cout << "Planner Iterations: " << planResults.plannerIterations << "\n";
	std::cout << "Plan Size: " << planResults.motionPlan.size() << "\n";
	std::cout << "Start Pose: \n" << planResults.startPose << "\n";
	std::cout << "Goal Pose: \n" << planResults.goalPose << "\n";
	std::cout << "Achieved Pose: \n" << planResults.achievedPose << "\n\n";
}
