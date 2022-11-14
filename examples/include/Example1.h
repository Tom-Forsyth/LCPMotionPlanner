#pragma once

#include "PhysicsCore.h"
#include "PhysicsScene.h"
#include "FrankaPanda.h"
#include "MotionPlanResults.h"
#include <vector>

namespace MotionPlanner
{
	class PhysicsScene;
	class Shape;

	/// @brief Example simulation environment.
	class Example1
	{
		/// @brief Physics core.
		PhysicsCore m_physics;

		/// @brief Physics scene.
		PhysicsScene* m_physicsScene = nullptr;

		/// @brief Franka Panda manipulator.
		FrankaPanda m_panda;

		/// @brief Array of obstacles in scene.
		std::vector<Shape*> m_obstacles;

		/// @brief Goal transforms to use as inputs to the motion planner.
		std::vector<Eigen::Matrix4d> m_goalTransforms;

		/// @brief Results from the motion planner.
		std::vector<MotionPlanResults> m_planResults;

		/// @brief Initialize physics.
		void initPhysics();

		/// @brief Initialize robot.
		void initRobot();

		/// @brief Create obstacles.
		void createObstacles();

		/// @brief Run motion planner.
		void generatePlans();

		/// @brief Display plan results.
		void displayPlanResults();

	public:
		/// @breif Constructor.
		Example1();

		/// @brief Destructor.
		~Example1();

		/// @brief Run simulation.
		void runSimulation();

		/// @brief Play the final plan.
		void displayPlan();
	};
}
