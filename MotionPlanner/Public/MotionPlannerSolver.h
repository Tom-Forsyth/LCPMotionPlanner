// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include <vector>
#include <Eigen/Dense>
#include "FrankaPanda.h"
#include "Containers/Array.h"
#include <chrono>

/**
 * 
 */
class MOTIONPLANNER_API MotionPlannerSolver
{
public:
	MotionPlannerSolver();
	~MotionPlannerSolver();

	FrankaPanda Panda;
	std::vector<Eigen::VectorXd> MotionPlan;
	void ComputePlan();

	int32 CurrentStep;
	TArray<double> CurrentAngles;
	void GetNextJointAngles();

	int32 PlanSize;
	int32 PlanComputeTime;
};
