#include "FrankaPanda.h"
#include "SpatialManipulator.h"
#include "RigidBodyChain.h"
#include "RigidBody.h"
#include "Joint.h"
#include "Sphere.h"
#include "Capsule.h"
#include "Box.h"
#include "ObjectType.h"
#include <Eigen/Dense>

namespace MotionPlanner
{
	FrankaPanda::FrankaPanda()
	{
		initRigidBodyChain();
	}

	FrankaPanda::FrankaPanda(const Eigen::Matrix4d& baseTransform)
		: SpatialManipulator(baseTransform)
	{
		initRigidBodyChain();
	}

	void FrankaPanda::initRigidBodyChain()
	{
		constexpr double pi = 3.14159265358979323846;

		// Link 0.
		Eigen::Matrix4d transform0 = Eigen::Matrix4d::Identity();
		Eigen::Vector3d localAxis0(0, 0, 0);
		Eigen::Vector3d spatialAxis0 = transform0.block(0, 0, 3, 3) * localAxis0;
		std::pair<double, double> jointLimits0 = {0, 0};
		Joint joint0(JointType::Fixed, spatialAxis0, transform0.block(0, 3, 3, 1), jointLimits0);
		RigidBody link0(joint0, transform0, "panda_link0");
		Sphere link0_sphere0(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), 0.15, "panda_link0_sphere0", ObjectType::RobotGeometry);
		link0.addCollider(link0_sphere0);

		// Link 1.
		Eigen::Matrix4d transform1 = Eigen::Matrix4d{
			{1, 0, 0, 0},
			{0, 1, 0, 0},
			{0, 0, 1, 0.333},
			{0, 0, 0, 1}
		};
		Eigen::Vector3d localAxis1(0, 0, 1);
		Eigen::Vector3d spatialAxis1 = transform1.block(0, 0, 3, 3) * localAxis1;
		std::pair<double, double> jointLimits1 = { -2.8973, 2.8973 };
		Joint joint1(JointType::Revolute, spatialAxis1, transform1.block(0, 3, 3, 1), jointLimits1);
		RigidBody link1(joint1, transform1, "panda_link1");
		Capsule link1_capsule0(Eigen::Vector3d(0, 0, -0.333 / 2), Eigen::Vector3d(0, pi / 2, 0), (0.333 - 0.06 * 2) / 2, 0.06, "panda_link1_capsule0", ObjectType::RobotGeometry);
		link1.addCollider(link1_capsule0);
		Capsule link1_capsule1(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, pi / 2), (0.283 - .06 * 2) / 2, 0.06, "panda_link1_capsule1", ObjectType::RobotGeometry);
		link1.addCollider(link1_capsule1);

		// Link 2.
		Eigen::Matrix4d transform2 = Eigen::Matrix4d{
			{1,  0, 0,     0},
			{0,  0, 1,     0},
			{0, -1, 0, 0.333},
			{0,  0, 0,     1}
		};
		Eigen::Vector3d localAxis2(0, 0, 1);
		Eigen::Vector3d spatialAxis2 = transform2.block(0, 0, 3, 3) * localAxis2;
		std::pair<double, double> jointLimits2 = { -1.7628, 1.7628 };
		Joint joint2(JointType::Revolute, spatialAxis2, transform2.block(0, 3, 3, 1), jointLimits2);
		RigidBody link2(joint2, transform2, "panda_link2");
		Capsule link2_capsule0(Eigen::Vector3d(0, -.316 / 2, 0), Eigen::Vector3d(0, 0, pi / 2), (0.316 - .06 * 2) / 2, 0.06, "panda_link2_capsule0", ObjectType::RobotGeometry);
		link2.addCollider(link2_capsule0);

		// Link 3.
		Eigen::Matrix4d transform3 = Eigen::Matrix4d{
			{1, 0, 0,     0},
			{0, 1, 0,     0},
			{0, 0, 1, 0.649},
			{0, 0, 0,     1}
		};
		Eigen::Vector3d localAxis3(0, 0, 1);
		Eigen::Vector3d spatialAxis3 = transform3.block(0, 0, 3, 3) * localAxis3;
		std::pair<double, double> jointLimits3 = { -2.8973, 2.8973 };
		Joint joint3(JointType::Revolute, spatialAxis3, transform3.block(0, 3, 3, 1), jointLimits3);
		RigidBody link3(joint3, transform3, "panda_link3");
		// 0.0825?
		Capsule link3_capsule0(Eigen::Vector3d(0.0825, 0, 0), Eigen::Vector3d(0, 0, pi / 2), (0.220 - .06 * 2) / 2, 0.06, "panda_link3_capsule0", ObjectType::RobotGeometry);
		link3.addCollider(link3_capsule0);

		// Link 4.
		Eigen::Matrix4d transform4 = Eigen::Matrix4d{
			{1, 0,  0, 0.0825},
			{0, 0, -1,      0},
			{0, 1,  0,  0.649},
			{0, 0, 0,       1}
		};
		Eigen::Vector3d localAxis4(0, 0, 1);
		Eigen::Vector3d spatialAxis4 = transform4.block(0, 0, 3, 3) * localAxis4;
		std::pair<double, double> jointLimits4 = { -3.0718, -0.0698 };
		Joint joint4(JointType::Revolute, spatialAxis4, transform4.block(0, 3, 3, 1), jointLimits4);
		RigidBody link4(joint4, transform4, "panda_link4");
		Capsule link4_capsule0(Eigen::Vector3d(-0.0825, 0.384 / 2, 0), Eigen::Vector3d(0, 0, pi / 2), (0.384 - .06 * 2) / 2, 0.06, "panda_link4_capsule0", ObjectType::RobotGeometry);
		link4.addCollider(link4_capsule0);

		// Link 5.
		Eigen::Matrix4d transform5 = Eigen::Matrix4d{
			{1, 0, 0,     0},
			{0, 1, 0,     0},
			{0, 0, 1, 1.033},
			{0, 0, 0,     1}
		};
		Eigen::Vector3d localAxis5(0, 0, 1);
		Eigen::Vector3d spatialAxis5 = transform5.block(0, 0, 3, 3) * localAxis5;
		std::pair<double, double> jointLimits5 = { -2.8973, 2.8973 };
		Joint joint5(JointType::Revolute, spatialAxis5, transform5.block(0, 3, 3, 1), jointLimits5);
		RigidBody link5(joint5, transform5, "panda_link5");
		Capsule link5_capsule0(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, pi / 2), (0.2 - .06 * 2) / 2, 0.06, "panda_link5_capsule0", ObjectType::RobotGeometry);
		link5.addCollider(link5_capsule0);

		// Link 6.
		Eigen::Matrix4d transform6 = Eigen::Matrix4d{
			{1, 0,  0,     0},
			{0, 0, -1,     0},
			{0, 1,  0, 1.033},
			{0, 0,  0,     1}
		};
		Eigen::Vector3d localAxis6(0, 0, 1);
		Eigen::Vector3d spatialAxis6 = transform6.block(0, 0, 3, 3) * localAxis6;
		std::pair<double, double> jointLimits6 = { -0.0175, 3.7525 };
		Joint joint6(JointType::Revolute, spatialAxis6, transform6.block(0, 3, 3, 1), jointLimits6);
		RigidBody link6(joint6, transform6, "panda_link6");
		Capsule link6_capsule0(Eigen::Vector3d(0.088, 0, 0), Eigen::Vector3d(0, 0, pi / 2), ((0.107 * 2) - .06 * 2) / 2, 0.06, "panda_link6_capsule0", ObjectType::RobotGeometry);
		link6.addCollider(link6_capsule0);
		Sphere link6_sphere0(Eigen::Vector3d(0.088 / 2, 0.06 / 2, 0), Eigen::Vector3d(0, 0, 0), 0.06, "panda_link6_sphere0", ObjectType::RobotGeometry);
		link6.addCollider(link6_sphere0);

		// Link 7.
		Eigen::Matrix4d transform7 = Eigen::Matrix4d{
			{1,  0,  0, 0.088},
			{0, -1,  0,     0},
			{0,  0, -1, 1.033},
			{0,  0,  0,     1}
		};
		Eigen::Vector3d localAxis7(0, 0, 1);
		Eigen::Vector3d spatialAxis7 = transform7.block(0, 0, 3, 3) * localAxis7;
		std::pair<double, double> jointLimits7 = { -2.8973, 2.8973 };
		Joint joint7(JointType::Revolute, spatialAxis7, transform7.block(0, 3, 3, 1), jointLimits7);
		RigidBody link7(joint7, transform7, "panda_link7");
		Box link7_box0(Eigen::Vector3d(0, 0, 0.1070 - (0.02)), Eigen::Vector3d(0, 0, -pi/4), Eigen::Vector3d(0.025, 0.075, 0.03), "panda_link7_box0", ObjectType::RobotGeometry);
		Box link7_box1(Eigen::Vector3d(0, 0, 0.125), Eigen::Vector3d(0, 0, -pi/4), Eigen::Vector3d(0.025, 0.1, 0.05), "panda_link7_box1", ObjectType::RobotGeometry);
		link7.addCollider(link7_box0);
		link7.addCollider(link7_box1);

		// Tip frame.
		Eigen::Matrix4d transformTip = Eigen::Matrix4d{
			{1,  0,  0, 0.0880},
			{0, -1,  0,      0},
			{0,  0, -1,  0.926},
			{0,  0,  0,      1}
		};
		Eigen::Vector3d localAxisTip(0, 0, 0);
		Eigen::Vector3d spatialAxisTip = transformTip.block(0, 0, 3, 3) * localAxisTip;
		std::pair<double, double> jointLimitsTip = { 0, 0 };
		Joint jointTip(JointType::Fixed, spatialAxisTip, transformTip.block(0, 3, 3, 1), jointLimitsTip);
		double fingerOffset = 0.015;
		RigidBody linkTip(jointTip, transformTip, "panda_linkTip");
		Box linkTip_box0(Eigen::Vector3d(fingerOffset, fingerOffset, 0.09), Eigen::Vector3d(0, 0, -pi/4), Eigen::Vector3d(0.01, 0.01, 0.03), "panda_linkTip_box0", ObjectType::RobotGeometry);
		Box linkTip_box1(Eigen::Vector3d(-fingerOffset, -fingerOffset, 0.09), Eigen::Vector3d(0, 0, -pi/4), Eigen::Vector3d(0.01, 0.01, 0.03), "panda_linkTip_box1", ObjectType::RobotGeometry);
		linkTip.addCollider(linkTip_box0);
		linkTip.addCollider(linkTip_box1);

		// End effector.
		Eigen::Matrix4d transformEE = Eigen::Matrix4d{
			{1,  0,  0, 0.0880},
			{0, -1,  0,      0},
			{0,  0, -1,  0.926},
			{0,  0,  0,      1}
		};
		Eigen::Vector3d localAxisEE(0, 0, 0);
		Eigen::Vector3d spatialAxisEE = transformEE.block(0, 0, 3, 3) * localAxisEE;
		std::pair<double, double> jointLimitsEE = { 0, 0 };
		Joint jointEE(JointType::Fixed, spatialAxisEE, transformEE.block(0, 3, 3, 1), jointLimitsEE);
		RigidBody linkEE(jointEE, transformEE, "panda_linkEE");

		// Create chain.
		m_rigidBodyChain.addBody(link0);
		m_rigidBodyChain.addBody(link1);
		m_rigidBodyChain.addBody(link2);
		m_rigidBodyChain.addBody(link3);
		m_rigidBodyChain.addBody(link4);
		m_rigidBodyChain.addBody(link5);
		m_rigidBodyChain.addBody(link6);
		m_rigidBodyChain.addBody(link7);
		m_rigidBodyChain.addBody(linkTip);
		m_rigidBodyChain.addBody(linkEE);

		// Finish initialization.
		setMaxReach(0.855);
		m_rigidBodyChain.postInit();
	}
}
