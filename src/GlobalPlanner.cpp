#include "GlobalPlanner.h"
#include "SpatialManipulator.h"
#include "RobotConfiguration.h"
#include "MotionPlanResults.h"
#include "PlannerExitCodes.h"
#include "LocalPlanner.h"
#include "TaskSpaceSampler.h"
#include "GlobalPlannerParams.h"
#include "Timer.h"
#include <Eigen/Dense>
#include <random>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/graph_traits.hpp>
#include <iostream>

namespace MotionPlanner
{
	GlobalPlanner::GlobalPlanner(SpatialManipulator* spatialManipulator, const Eigen::Matrix4d& goalPose)
		: m_spatialManipulator(spatialManipulator), m_goalPose(goalPose),
		m_sampler(spatialManipulator->getBaseTransform().block(0, 3, 3, 1), spatialManipulator->getMaxReach())
	{
		// Reserve space for vectors.
		m_vertexDescriptors.reserve(m_params.maxIterations);
		m_edgeDescriptors.reserve(m_params.maxIterations);

		// Add initial configuration.
		addNode(m_spatialManipulator->getEndFrameSpatialTransform(), m_spatialManipulator->getJointDisplacements());
	}

	void GlobalPlanner::computePlan()
	{
		// Initialize RRT planner.
		int iter = 0;
		m_timer.start();
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

			// Determine if the connection distance should be enforced.
			bool enforceConnectionDistance = true;
			const int nearestInt = round(1.0 / m_params.straightToGoalProbability);
			if (m_attemptingGoal && !(m_goalAttemptCount % nearestInt))
			{
				enforceConnectionDistance = false;
			}

			// Generate an intermediate goal pose between the start and goal in we enforce the connection distance.
			Eigen::Matrix4d intermediatePose;
			if (enforceConnectionDistance)
			{
				intermediatePose = createIntermediatePose(sampledPose, closestNode);
			}
			else
			{
				intermediatePose = sampledPose;
			}

			// Run the local planner.
			MotionPlanResults localPlan = generateLocalPlan(m_graph[closestNode], intermediatePose);

			// If the local plan was successful, add to graph.
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
		m_timer.stop();
		m_plannerIterations = iter;

		// Find the shortest path with Dijkstra's algorithm.
		findShortestPath();
	}

	void GlobalPlanner::addNode(const Eigen::Matrix4d& pose, const Eigen::VectorXd& jointDisplacements)
	{
		m_vertexDescriptors.emplace_back(boost::add_vertex({ pose, jointDisplacements }, m_graph));
	}

	void GlobalPlanner::addEdge(const MotionPlanResults& planResults, const VertexDescriptor& startNode, const VertexDescriptor& endNode)
	{
		m_edgeDescriptors.emplace_back(boost::add_edge(startNode, endNode, { planResults }, m_graph).first);
	}

	MotionPlanResults GlobalPlanner::getPlanResults() const
	{
		// Plan results and goal pose.
		MotionPlanResults globalPlanResults;
		globalPlanResults.exitCode = static_cast<int>(m_exitCode);
		globalPlanResults.motionPlan = m_plan;
		globalPlanResults.goalPose = m_goalPose;

		// Starting parameters.
		const RobotConfiguration& initialConfig = m_graph[m_vertexDescriptors.front()];
		globalPlanResults.startPose = initialConfig.endEffectorPose;
		globalPlanResults.startJointDisplacements = initialConfig.jointDisplacements;

		// Achieved parameters.
		const RobotConfiguration& finalConfig = m_graph[m_vertexDescriptors.back()];
		globalPlanResults.achievedPose = finalConfig.endEffectorPose;
		globalPlanResults.achievedJointDisplacements = finalConfig.jointDisplacements;

		// Time and iterations.
		globalPlanResults.computeTimeMilli = m_timer.getTimeMilli();
		globalPlanResults.plannerIterations = m_plannerIterations;

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
		/*
		*
		* 
		* FIX!!!
		* 
		* 
		* 
		*/
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
			m_goalAttemptCount++;
			return m_goalPose;
		}
	}

	void GlobalPlanner::findShortestPath()
	{
		Timer timer;
		timer.start();

		// Find shortest path with Dijkstra's algorithm.
		dijkstra_shortest_paths(m_graph, m_vertexDescriptors.front(),
			predecessor_map(boost::get(&RobotConfiguration::pathPredecessor, m_graph))
			.distance_map(boost::get(&RobotConfiguration::pathPredecessor, m_graph))
			.weight_map(boost::get(&MotionPlanResults::weight, m_graph)));

		// DEBUG
		std::vector<RobotConfiguration> testVec;
		for (const auto& elem : m_vertexDescriptors)
		{
			testVec.emplace_back(m_graph[elem]);
		}

		for (const auto& edgeDesc : boost::make_iterator_range(edges(m_graph)))
		{
			auto test = edgeDesc;
			int a = 1;
		}

		for (const auto& vertexDesc : boost::make_iterator_range(vertices(m_graph)))
		{
			auto test = vertexDesc;
		}

		// Display graph.
		std::cout << "\nGraph Display: \n";
		boost::print_graph(m_graph);
		std::cout << "\n";

		// Extract the indices of the shortest path from goal to start.
		std::vector<VertexDescriptor> shortestPath;
		VertexDescriptor currentVertex = m_vertexDescriptors.back();
		while (currentVertex != m_vertexDescriptors.front())
		{
			shortestPath.push_back(currentVertex);
			currentVertex = m_graph[currentVertex].pathPredecessor;
		}

		// Reverse path into the shortest path from start to goal.
		std::reverse(shortestPath.begin(), shortestPath.end());

		// Extract the joint trajectories of the local plans of the shortest path.
		size_t localPlanSize = 1000;
		m_plan.reserve(shortestPath.size() * localPlanSize);
		for (int nodeIndex : shortestPath)
		{
			// Extract the joint displacements in the local plan.
			const MotionPlanResults& localPlanResults = m_graph[m_edgeDescriptors[nodeIndex - 1]];
			for (const Eigen::VectorXd& displacements : localPlanResults.motionPlan)
			{
				m_plan.emplace_back(displacements);
			}
		}

		timer.stop();
		std::cout << "Dijkstra Time (us): " << timer.getTimeMicro() << "\n";
	}
}
