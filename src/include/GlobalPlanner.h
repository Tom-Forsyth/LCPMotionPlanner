#pragma once

#include "RobotConfiguration.h"
#include "MotionPlanResults.h"
#include "PlannerExitCodes.h"
#include "TaskSpaceSampler.h"
#include "GlobalPlannerParams.h"
#include <Eigen/Dense>
#include <vector>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/graph_traits.hpp>

namespace MotionPlanner
{
	class SpatialManipulator;

	/// @brief Graph with nodes as configuration and edges as local motion plans.
	typedef boost::adjacency_list<boost::vecS, boost::vecS,
								  boost::directedS,
								  RobotConfiguration,
								  MotionPlanResults
								  > Graph;

	/// @brief Vertex descriptor for custom graph type.
	typedef boost::graph_traits<Graph>::vertex_descriptor VertexDescriptor;

	/// @brief Edge descriptor for custom graph type.
	typedef boost::graph_traits<Graph>::edge_descriptor EdgeDescriptor;

	/// @brief Global RRT framework for local LCP/SclERP planner.
	class GlobalPlanner
	{
		/// @brief Pointer to the spatial manipulator to generate the plan with.
		SpatialManipulator* m_spatialManipulator;

		/// @brief Goal pose.
		Eigen::Matrix4d m_goalPose;

		/// @brief Graph with nodes as robot configuration and edges as local plans.
		Graph m_graph;

		/// @brief Vertex descriptors of the nodes of the graph.
		std::vector<VertexDescriptor> m_vertexDescriptors;

		/// @brief Edge descriptors for the graph.
		std::vector<EdgeDescriptor> m_edgeDescriptors;

		/// @brief Joint trajectory of the robot after planning.
		std::vector<Eigen::VectorXd> m_jointTrajectory;

		/// @brief Global planner exit code.
		GlobalPlannerExitCode m_exitCode = GlobalPlannerExitCode::Undefined;

		/// @brief True when the planner is running.
		bool m_isRunning = true;

		/// @brief True when the intermediate goal pose is the goal itself.
		bool m_attemptingGoal = false;

		/// @brief Draws samples from the task space.
		TaskSpaceSampler m_sampler;

		/// @brief Parameters for the global RRT planner.
		GlobalPlannerParams m_params;

		/// @brief Add robot configuration node to the graph.
		/// @param pose End-effector pose.
		/// @param jointDisplacements Robot joint displacements.
		void addNode(const Eigen::Matrix4d& pose, const Eigen::VectorXd& jointDisplacements);

		/// @brief Add a motion plan edge to the graph.
		/// @param planResults Motion planning results edge.
		void addEdge(const MotionPlanResults& planResults, const VertexDescriptor& startNode, const VertexDescriptor& endNode);

		/// @brief Attempt to generate local planner from the start configuration to the goal pose.
		/// @param startConfig Starting end-effector pose and joint configuration.
		/// @param goalPose Goal end-effector pose.
		/// @return Results of the local planner.
		MotionPlanResults generateLocalPlan(const RobotConfiguration& startConfig, const Eigen::Matrix4d& goalPose) const;

		/// @brief Find the closest node on the graph to the sampled pose.
		/// @param sampledPose Sampled pose.
		/// @return Closest node on the graph.
		VertexDescriptor findClosestNode(const Eigen::Matrix4d& sampledPose) const;

		/// @brief Create an intermediate goal pose a fixed distance from the closest node.
		/// @param sampledPose Sampled pose.
		/// @param closestNode Closest node on graph.
		/// @return Intermediate goal pose.
		Eigen::Matrix4d createIntermediatePose(const Eigen::Matrix4d& sampledPose, const VertexDescriptor& closestNode);

		/// @brief Draw a pose sample in SE3 with a certain probability for the sample to be the goal.
		/// @param iteration Iteration number used in sampling probability.
		/// @return Pose sample.
		Eigen::Matrix4d drawPoseSample(int iteration);

	public:
		/// @brief Constructor.
		GlobalPlanner(SpatialManipulator* spatialManipulator, const Eigen::Matrix4d& goalPose);

		/// @brief Run the global planner to generate a motion plan.
		void computePlan();

		/// @brief Get the joint trajectory of the robot after planning.
		MotionPlanResults getPlanResults() const;
	};
}
