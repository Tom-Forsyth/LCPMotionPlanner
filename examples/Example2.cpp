#include "Example2.h"
#include "MotionPlanResults.h"
#include <Eigen/Dense>
#include <iostream>

#include "DualQuaternion.h"

namespace MotionPlanner
{
	Example2::Example2()
	{

	}

	Example2::~Example2()
	{
		for (int i = 0; i < m_obstacles.size(); i++)
		{
			delete m_obstacles[i];
		}
	}

	void Example2::runSimulation()
	{
		initPhysics();
		initRobot();
		createObstacles();
		generatePlans();
		displayPlanResults();
	}

	void Example2::displayPlan()
	{
		for (const MotionPlanner::MotionPlanResults& plan : m_planResults)
		{
			for (const Eigen::VectorXd& jointDisplacement : plan.motionPlan)
			{
				m_panda.setJointDisplacements(jointDisplacement);
			}
		}
	}

	void Example2::initPhysics()
	{
		m_physics.createPhysicsCore();
		m_physicsScene = m_physics.createPhysicsScene("MyPhysicsScene");
	}

	void Example2::initRobot()
	{
		// Initialize panda.
		Eigen::Matrix4d pandaBaseTransform{
			{1, 0, 0, -2.5812},
			{0, 1, 0, 0},
			{0, 0, 1, 0.38367},
			{0, 0, 0, 1}
		};
		//m_panda.setBaseTransform(pandaBaseTransform);
		m_physicsScene->addSpatialManipulator(m_panda);

		// Set starting joint angles.
		Eigen::Vector<double, 7> startAngles(0, 0, 0, -EIGEN_PI / 2, 0, EIGEN_PI / 2, 0);
		static_cast<void>(m_panda.setJointDisplacements(startAngles));


		// Create target poses.
		Eigen::Matrix4d goalTransform1{
			{1, 0, 0, 0.6},
			{0, -1, 0, -0.35},
			{0, 0, -1, 0.45},
			{0, 0, 0, 1}
		};
		m_goalTransforms.push_back(goalTransform1);

		Eigen::Matrix4d goalTransform2{
			{1, 0, 0, 0.6},
			{0, -1, 0, 0.35},
			{0, 0, -1, 0.45},
			{0, 0, 0, 1}
		};
		m_goalTransforms.push_back(goalTransform2);

		Eigen::Matrix4d goalTransform3{
			{1, 0, 0, 0.6},
			{0, -1, 0, 0.35},
			{0, 0, -1, 0.35},
			{0, 0, 0, 1}
		};
		m_goalTransforms.push_back(goalTransform3);

		Eigen::Matrix4d goalTransform4{
			{1, 0, 0, 0.6},
			{0, -1, 0, 0.35},
			{0, 0, -1, 0.45},
			{0, 0, 0, 1}
		};
		m_goalTransforms.push_back(goalTransform4);

		Eigen::Matrix4d goalTransform5{
			{1, 0, 0, 0.4},
			{0, -1, 0, -0.35},
			{0, 0, -1, 0.45},
			{0, 0, 0, 1}
		};
		m_goalTransforms.push_back(goalTransform5);

		Eigen::Matrix4d goalTransform6{
			{0.707, -0.707, 0, 0.4},
			{-0.707, -0.707, 0, -0.35},
			{0, 0, -1, 0.45},
			{0, 0, 0, 1}
		};
		m_goalTransforms.push_back(goalTransform6);
	}

	void Example2::createObstacles()
	{
		// Object 1.
		Eigen::Vector3d position1(0.2992, 0.286482, 0.597666);
		Eigen::Vector3d orientation1(0, 0, 0);
		Eigen::Vector3d radii1(0.082018, 0.03590, 0.106719);
		m_obstacles.emplace_back(new MotionPlanner::Box(position1, orientation1, radii1, "Obstacle 1", MotionPlanner::ObjectType::Obstacle));

		// Add obstacles to scene.
		for (int i = 0; i < m_obstacles.size(); i++)
		{
			m_physicsScene->addObstacle(*m_obstacles[i]);
		}
	}

	void Example2::generatePlans()
	{
		m_planResults.reserve(m_goalTransforms.size());
		for (const Eigen::Matrix4d& goalPose : m_goalTransforms)
		{
			m_planResults.emplace_back(m_panda.motionPlan(goalPose));
		}
	}

	void Example2::displayPlanResults()
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
