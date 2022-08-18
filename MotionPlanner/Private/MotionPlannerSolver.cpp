// Fill out your copyright notice in the Description page of Project Settings.


#include "MotionPlannerSolver.h"
#include <Eigen/Dense>
#include <vector>
#include "FrankaPanda.h"
#include "SphereObstacle.h"
#include "Containers/Array.h"
#include <chrono>

MotionPlannerSolver::MotionPlannerSolver()
{
	Eigen::Vector3d Origin {0, 0, 0};
	Panda = FrankaPanda(Origin);
	CurrentAngles.Init(0.0, 7);
	MotionPlan.reserve(10000);
}

MotionPlannerSolver::~MotionPlannerSolver()
{
}

void MotionPlannerSolver::ComputePlan()
{
	// Obstacles.
	//SphereObstacle obs1{ 0.0, -0.4, 0.8, 0.2 };
	SphereObstacle obs1{ 0.0, -0.4, 0.8, 0.2 };
	std::vector<SphereObstacle> obstacles{ obs1 };
	Panda.setObstacles(obstacles);

	// Start and goal angles.
	Eigen::Vector<double, 7> jointAngles1 = {0, 0, 0, 0, 0, 0, 0};
	//Eigen::Vector<double, 7> jointAngles1 = {0.0, -PI/4.0, 0.0, -3*PI/4.0, 0.0, PI/2.0, PI/4.0};
	Eigen::Vector<double, 7> jointAngles2 = {PI/2.0, PI/2.0, 0, 0, PI/2.0, 0, 0};

	// Start and goal configurations.
	Panda.setJointAngles(jointAngles2);
	Eigen::Matrix4d gGoal = Panda.gBaseFrames.at(7);
	Panda.setJointAngles(jointAngles1);
	Eigen::Matrix4d gStart = Panda.gBaseFrames.at(7);

	// Generate plan.
	auto startTime = std::chrono::system_clock::now();
	MotionPlan = Panda.motionPlan(gGoal);
	auto endTime = std::chrono::system_clock::now();
	PlanComputeTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
	PlanSize = MotionPlan.size();

	// Init.
	GetNextJointAngles();
	CurrentStep = 0;
}

void MotionPlannerSolver::GetNextJointAngles()
{
	if (CurrentStep < PlanSize - 1)
	{
		Eigen::VectorXd CurrentAnglesEigen = MotionPlan.at(CurrentStep);

		// Convert to TArray.
		for (int32 i = 0; i < CurrentAnglesEigen.size(); i++)
		{
			CurrentAngles[i] = CurrentAnglesEigen(i);
		}

		CurrentStep++;
	}
}
