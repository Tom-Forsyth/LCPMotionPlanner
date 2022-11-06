#include "GlobalPlanner.h"
#include "SpatialManipulator.h"
#include "RobotConfiguration.h"
#include "MotionPlanResults.h"
#include "PlannerExitCodes.h"
#include "LocalPlanner.h"
#include "TaskSpaceSampler.h"
#include "GlobalPlannerParams.h"
#include <Eigen/Dense>
#include <random>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/graph_traits.hpp>

namespace MotionPlanner
{
	GlobalPlanner::GlobalPlanner(SpatialManipulator* spatialManipulator, const Eigen::Matrix4d& goalPose)
		: m_spatialManipulator(spatialManipulator), m_goalPose(goalPose),
		m_sampler(spatialManipulator->getBaseTransform().block(0, 3, 3, 1), spatialManipulator->getMaxReach())
	{
		// Add initial configuration.
		addNode(m_spatialManipulator->getEndFrameSpatialTransform(), m_spatialManipulator->getJointDisplacements());

		// Reserve space for joint trajectory vector.
		size_t maxLocalPlanSize = 2000;
		size_t maxGlobalPlanSize = m_params.maxIterations;
		m_jointTrajectory.reserve(maxLocalPlanSize * maxGlobalPlanSize);
	}

	void GlobalPlanner::computePlan()
	{
		// Initialize RRT planner.
		int iter = 0;
		while (m_isRunning)
		{
			// Sample task space or joint space pose.
			Eigen::Matrix4d sampledPose;
			if (m_params.jointSpaceSampling)
			{
				// Draw joint space configuration and find the corresponding pose.
				const Eigen::VectorXd jointSpaceSample = m_sampler.drawJointSpaceSample(m_spatialManipulator);
				m_spatialManipulator->setJointDisplacements(jointSpaceSample);
				sampledPose = m_spatialManipulator->getEndFrameSpatialTransform();
			}
			else
			{
				// Draw SE3 configuration.
				sampledPose = drawPoseSample(iter);
			}

			// Find closest node.
			const VertexDescriptor closestNode = findClosestNode(sampledPose);

			// Generate an intermediate goal pose a fixed distance from the sampled node.
			const Eigen::Matrix4d intermediatePose = createIntermediatePose(sampledPose, closestNode);

			// Run the local planner.
			MotionPlanResults localPlan = generateLocalPlan(m_graph[closestNode], intermediatePose);

			// If the plan was successful, add to graph.
			if (static_cast<LocalPlannerExitCode>(localPlan.exitCode) == LocalPlannerExitCode::Success)
			{
				addNode(localPlan.achievedPose, localPlan.achievedJointDisplacements);
				addEdge(localPlan, closestNode, m_vertexDescriptors.back());
			}

			// If local plan to the goal is successful, problem is solved.
			if (m_attemptingGoal)
			{
				if (static_cast<LocalPlannerExitCode>(localPlan.exitCode) == LocalPlannerExitCode::Success)
				{
					m_isRunning = false;
					m_exitCode = GlobalPlannerExitCode::Success;
				}
			}

			// If we exceed the max iterations, terminate.
			if (iter == m_params.maxIterations - 1)
			{
				m_isRunning = false;
				m_exitCode = GlobalPlannerExitCode::MaxIterationsExceeded;
			}

			iter++;
		}
	}

	void GlobalPlanner::addNode(const Eigen::Matrix4d& pose, const Eigen::VectorXd& jointDisplacements)
	{
		m_vertexDescriptors.emplace_back(boost::add_vertex({ pose, jointDisplacements }, m_graph));
	}

	void GlobalPlanner::addEdge(const MotionPlanResults& planResults, const VertexDescriptor& startNode, const VertexDescriptor& endNode)
	{
		//m_edgeDescriptors.emplace_back(boost::add_edge(startNode, endNode, { planResults }, m_graph));
	}

	MotionPlanResults GlobalPlanner::getPlanResults() const
	{
		// Plan results and goal pose.
		MotionPlanResults globalPlanResults;
		globalPlanResults.exitCode = static_cast<int>(m_exitCode);
		globalPlanResults.motionPlan = m_jointTrajectory;
		globalPlanResults.goalPose = m_goalPose;

		// Starting parameters.
		const RobotConfiguration& initialConfig = m_graph[m_vertexDescriptors.front()];
		globalPlanResults.startPose = initialConfig.endEffectorPose;
		globalPlanResults.startJointDisplacements = initialConfig.jointDisplacements;

		// Achieved parameters.
		const RobotConfiguration& finalConfig = m_graph[m_vertexDescriptors.back()];
		globalPlanResults.achievedPose = finalConfig.endEffectorPose;
		globalPlanResults.achievedJointDisplacements = finalConfig.jointDisplacements;

		return globalPlanResults;
	}

	MotionPlanResults GlobalPlanner::generateLocalPlan(const RobotConfiguration& startConfig, const Eigen::Matrix4d& goalPose) const
	{
		// Set robot to start configuration.
		m_spatialManipulator->setJointDisplacements(startConfig.jointDisplacements);

		// Run local planner.
		LocalPlanner localPlanner(m_spatialManipulator, goalPose);
		localPlanner.computePlan();
		return localPlanner.getPlanResults();
	}

	VertexDescriptor GlobalPlanner::findClosestNode(const Eigen::Matrix4d& sampledPose) const
	{
		double minDistance = 1e10;
		VertexDescriptor closestNode = m_vertexDescriptors.front();
		for (const VertexDescriptor& vertexDesc: m_vertexDescriptors)
		{
			const RobotConfiguration& robotConfig = m_graph[vertexDesc];
			const double distance = (sampledPose - robotConfig.endEffectorPose).norm();
			if (distance < minDistance)
			{
				minDistance = distance;
				closestNode = vertexDesc;
			}
		}
		return closestNode;
	}

	Eigen::Matrix4d GlobalPlanner::createIntermediatePose(const Eigen::Matrix4d& sampledPose, const VertexDescriptor& closestNode)
	{
		// If attempting to go straight to the goal, do not create an intermediate pose.
		if (m_attemptingGoal)
		{
			return sampledPose;
		}

		// Determine if the sampled pose is too far from the closest node.
		const Eigen::Vector3d sampledPosition = sampledPose.block(0, 3, 3, 1);
		const Eigen::Matrix4d closestPose = m_graph[closestNode].endEffectorPose;
		const Eigen::Vector3d closestPosition = closestPose.block(0, 3, 3, 1);
		const Eigen::Vector3d translationVector = sampledPosition - closestPosition;
		const double distance = translationVector.norm();
		const bool isTooFar = (distance <= m_params.maxConnectionDistance) ? false : true;

		// Create intermediate pose along translation direction.
		Eigen::Matrix4d intermediatePose = Eigen::Matrix4d::Identity();
		if (isTooFar)
		{
			double scaleFactor = m_params.maxConnectionDistance / distance;
			Eigen::Vector3d intermediatePosition = closestPosition + scaleFactor * translationVector;
			intermediatePose.block(0, 3, 3, 1) = intermediatePosition;
		}

		// If we do not want to sample orientation, use the orientation of the starting node.
		if (!m_params.sampleOrientation)
		{
			intermediatePose.block(0, 0, 3, 3) = closestPose.block(0, 0, 3, 3);
		}
		else
		{
			intermediatePose.block(0, 0, 3, 3) = sampledPose.block(0, 0, 3, 3);
		}

		return intermediatePose;
	}

	Eigen::Matrix4d GlobalPlanner::drawPoseSample(int iteration)
	{
		const int nearestInt = round(1.0 / m_params.sampleGoalProbability);
		if (iteration % nearestInt)
		{
			m_attemptingGoal = false;
			return m_sampler.drawSE3Sample();
		}
		else
		{
			m_attemptingGoal = true;
			return m_goalPose;
		}
	}
}
