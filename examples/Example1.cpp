#include "Example1.h"
#include "MotionPlanResults.h"
#include <Eigen/Dense>
#include <iostream>

namespace MotionPlanner
{
	Example1::Example1()
	{

	}

	Example1::~Example1()
	{
		for (int i = 0; i < m_obstacles.size(); i++)
		{
			delete m_obstacles[i];
		}
	}

	void Example1::runSimulation()
	{
		initPhysics();
		initRobot();
		createObstacles();
		generatePlans();
		displayPlanResults();
	}

	void Example1::displayPlan()
	{
		for (const MotionPlanner::MotionPlanResults& plan : m_planResults)
		{
			for (const Eigen::VectorXd& jointDisplacement : plan.motionPlan)
			{
				m_panda.setJointDisplacements(jointDisplacement);
			}
		}
	}

	void Example1::initPhysics()
	{
		m_physics.createPhysicsCore();
		m_physicsScene = m_physics.createPhysicsScene("MyPhysicsScene");
	}

	void Example1::initRobot()
	{
		// Initialize panda.
		Eigen::Matrix4d pandaBaseTransform{
			{1, 0, 0, 1},
			{0, 1, 0, 1},
			{0, 0, 1, 0},
			{0, 0, 0, 1}
		};
		m_panda.setBaseTransform(pandaBaseTransform);
		m_physicsScene->addSpatialManipulator(m_panda);

		// Set starting joint angles.
		Eigen::Vector<double, 7> startAngles(0, 0, 0, -EIGEN_PI / 2, 0, EIGEN_PI / 2, 0);
		static_cast<void>(m_panda.setJointDisplacements(startAngles));

		// Create target poses.
		Eigen::Matrix4d startTransform = m_panda.getEndFrameSpatialTransform();
		Eigen::Matrix4d goalTransform1(startTransform);
		Eigen::Matrix4d goalTransform2(startTransform);
		goalTransform1.block(0, 3, 3, 1) = Eigen::Vector3d(0.6, -0.35, 0.45);
		goalTransform2.block(0, 3, 3, 1) = Eigen::Vector3d(0.6, 0.35, 0.45);
		m_goalTransforms.push_back(goalTransform1);
		m_goalTransforms.push_back(goalTransform2);
	}

	void Example1::createObstacles()
	{
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
		m_obstacles.emplace_back(new MotionPlanner::Box(tableOrigin, zeroVec, tableOffsets, "Table Top", tableObjectType));
		m_obstacles.emplace_back(new MotionPlanner::Box(legOrigin1, zeroVec, legOffsets, "Table Leg 1", tableObjectType));
		m_obstacles.emplace_back(new MotionPlanner::Box(legOrigin2, zeroVec, legOffsets, "Table Leg 2", tableObjectType));
		m_obstacles.emplace_back(new MotionPlanner::Box(legOrigin3, zeroVec, legOffsets, "Table Leg 3", tableObjectType));
		m_obstacles.emplace_back(new MotionPlanner::Box(legOrigin4, zeroVec, legOffsets, "Table Leg 4", tableObjectType));

		// Obstacles on table.
		m_obstacles.emplace_back(new MotionPlanner::Sphere(Eigen::Vector3d(1.55, 1, 0.35), Eigen::Vector3d::Zero(), 0.075, "Sphere Obstacle", MotionPlanner::ObjectType::Obstacle));
		m_obstacles.emplace_back(new MotionPlanner::Capsule(Eigen::Vector3d(1.3, 1.25, 0.55), Eigen::Vector3d(0, EIGEN_PI / 2, 0), 0.2, 0.05, "Capsule Obstacle", MotionPlanner::ObjectType::Obstacle));

		// Add obstacles to scene.
		for (int i = 0; i < m_obstacles.size(); i++)
		{
			m_physicsScene->addObstacle(*m_obstacles[i]);
		}
	}

	void Example1::generatePlans()
	{
		m_planResults.reserve(m_goalTransforms.size());
		for (const Eigen::Matrix4d& goalPose : m_goalTransforms)
		{
			m_planResults.emplace_back(m_panda.motionPlan(goalPose));
		}
	}

	void Example1::displayPlanResults()
	{
		for (const MotionPlanResults& planResults : m_planResults)
		{
			static int planNum = 0;
			std::cout << "----- Motion Plan " << planNum << " Results -----\n";
			std::cout << "Exit Code: " << planResults.exitCode << "\n";
			std::cout << "Compute Time: " << planResults.computeTimeMilli << " ms\n";
			std::cout << "Planner Iterations: " << planResults.plannerIterations << "\n";
			std::cout << "Plan Size: " << planResults.motionPlan.size() << "\n";
			std::cout << "Linear Displacement of EE: " << planResults.linearDisplacement << " m\n";
			std::cout << "Start Pose: \n" << planResults.startPose << "\n";
			std::cout << "Goal Pose: \n" << planResults.goalPose << "\n";
			std::cout << "Achieved Pose: \n" << planResults.achievedPose << "\n\n";
			planNum++;
		}
	}
}
