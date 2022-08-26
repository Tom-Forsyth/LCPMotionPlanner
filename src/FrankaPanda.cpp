#include "FrankaPanda.h"
#include "SpatialManipulator.h"
#include "RigidBodyChain.h"
#include "RigidBody.h"
#include "Joint.h"
#include "Sphere.h"
#include "Capsule.h"
#include "Box.h"
#include <Eigen/Dense>

// Default constructor.
FrankaPanda::FrankaPanda()
{
	initRigidBodyChain();
	setupSimulationEnvironment();
}

// Base transform constructor.
FrankaPanda::FrankaPanda(const Eigen::Matrix4d& baseTransform)
	: SpatialManipulator(baseTransform)
{
	initRigidBodyChain();
	setupSimulationEnvironment();
}


void FrankaPanda::initRigidBodyChain()
{
	// Setup.
	double pi = 3.1415;
	bool viewFrames = false;
	bool viewCollision = true;
	

	/* Create Kinematic Chain */
	// Link 0.
	Eigen::Matrix4d transform0 = Eigen::Matrix4d::Identity();
	Eigen::Vector3d localAxis0(0, 0, 0);
	Eigen::Vector3d spatialAxis0 = transform0.block(0, 0, 3, 3) * localAxis0;
	Joint joint0(Joint::FIXED, spatialAxis0, transform0.block(0, 3, 3, 1));
	RigidBody link0(joint0, transform0);
	

	// Link 1.
	Eigen::Matrix4d transform1 = Eigen::Matrix4d{
		{1, 0, 0, 0},
		{0, 1, 0, 0},
		{0, 0, 1, 0.333},
		{0, 0, 0, 1}
	};
	Eigen::Vector3d localAxis1(0, 0, 1);
	Eigen::Vector3d spatialAxis1 = transform1.block(0, 0, 3, 3) * localAxis1;
	Joint joint1(Joint::REVOLUTE, spatialAxis1, transform1.block(0, 3, 3, 1));
	RigidBody link1(joint1, transform1);

	// Link 2.
	Eigen::Matrix4d transform2 = Eigen::Matrix4d{
		{1,  0, 0,     0},
		{0,  0, 1,     0},
		{0, -1, 0, 0.333},
		{0,  0, 0,     1}
	};
	Eigen::Vector3d localAxis2(0, 0, 1);
	Eigen::Vector3d spatialAxis2 = transform2.block(0, 0, 3, 3) * localAxis2;
	Joint joint2(Joint::REVOLUTE, spatialAxis2, transform2.block(0, 3, 3, 1));
	RigidBody link2(joint2, transform2);

	// Link 3.
	Eigen::Matrix4d transform3 = Eigen::Matrix4d{
		{1, 0, 0,     0},
		{0, 1, 0,     0},
		{0, 0, 1, 0.649},
		{0, 0, 0,     1}
	};
	Eigen::Vector3d localAxis3(0, 0, 1);
	Eigen::Vector3d spatialAxis3 = transform3.block(0, 0, 3, 3) * localAxis3;
	Joint joint3(Joint::REVOLUTE, spatialAxis3, transform3.block(0, 3, 3, 1));
	RigidBody link3(joint3, transform3);

	// Link 4.
	Eigen::Matrix4d transform4 = Eigen::Matrix4d{
		{1, 0,  0, 0.0825},
		{0, 0, -1,      0},
		{0, 1,  0,  0.649},
		{0, 0, 0,       1}
	};
	Eigen::Vector3d localAxis4(0, 0, 1);
	Eigen::Vector3d spatialAxis4 = transform4.block(0, 0, 3, 3) * localAxis4;
	Joint joint4(Joint::REVOLUTE, spatialAxis4, transform4.block(0, 3, 3, 1));
	RigidBody link4(joint4, transform4);

	// Link 5.
	Eigen::Matrix4d transform5 = Eigen::Matrix4d{
		{1, 0, 0,     0},
		{0, 1, 0,     0},
		{0, 0, 1, 1.033},
		{0, 0, 0,     1}
	};
	Eigen::Vector3d localAxis5(0, 0, 1);
	Eigen::Vector3d spatialAxis5 = transform5.block(0, 0, 3, 3) * localAxis5;
	Joint joint5(Joint::REVOLUTE, spatialAxis5, transform5.block(0, 3, 3, 1));
	RigidBody link5(joint5, transform5);

	// Link 6.
	Eigen::Matrix4d transform6 = Eigen::Matrix4d{
		{1, 0,  0,     0},
		{0, 0, -1,     0},
		{0, 1,  0, 1.033},
		{0, 0,  0,     1}
	};
	Eigen::Vector3d localAxis6(0, 0, 1);
	Eigen::Vector3d spatialAxis6 = transform6.block(0, 0, 3, 3) * localAxis6;
	Joint joint6(Joint::REVOLUTE, spatialAxis6, transform6.block(0, 3, 3, 1));
	RigidBody link6(joint6, transform6);

	// Link 7.
	Eigen::Matrix4d transform7 = Eigen::Matrix4d{
		{1,  0,  0, 0.088},
		{0, -1,  0,     0},
		{0,  0, -1, 1.033},
		{0,  0,  0,     1}
	};
	Eigen::Vector3d localAxis7(0, 0, 1);
	Eigen::Vector3d spatialAxis7 = transform7.block(0, 0, 3, 3) * localAxis7;
	Joint joint7(Joint::REVOLUTE, spatialAxis7, transform7.block(0, 3, 3, 1));
	RigidBody link7(joint7, transform7);

	// Tip frame.
	Eigen::Matrix4d transformTip = transform7;
	Eigen::Vector3d localAxisTip(0, 0, 0);
	Eigen::Vector3d spatialAxisTip = transformTip.block(0, 0, 3, 3) * localAxisTip;
	Joint jointTip(Joint::FIXED, spatialAxisTip, transformTip.block(0, 3, 3, 1));
	RigidBody linkTip(jointTip, transformTip);


	/* Create Visual Model of Frames */
	if (viewFrames)
	{
		double visRad = 0.025;
		Sphere link0_origin(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), visRad, "link0_origin");
		link0.addCollider(link0_origin);

		Box link1_origin(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(visRad, visRad, visRad), "link1_origin");
		link1.addCollider(link0_origin);

		Box link2_origin(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(visRad, visRad, visRad), "link2_origin");
		link2.addCollider(link2_origin);

		Box link3_origin(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(visRad, visRad, visRad), "link3_origin");
		link3.addCollider(link3_origin);

		Box link4_origin(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(visRad, visRad, visRad), "link4_origin");
		link4.addCollider(link4_origin);

		Box link5_origin(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(visRad, visRad, visRad), "link5_origin");
		link5.addCollider(link5_origin);

		Box link6_origin(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(visRad, visRad, visRad), "link6_origin");
		link6.addCollider(link6_origin);

		Box link7_origin(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(visRad, visRad, visRad), "link7_origin");
		link7.addCollider(link7_origin);

		Sphere linkTip_origin(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), visRad, "linkTip_origin");
		linkTip.addCollider(linkTip_origin);
	}


	/* Create Colliders */
	if (viewCollision)
	{
		Sphere link0_sphere0(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), 0.15, "link0_sphere0");
		link0.addCollider(link0_sphere0);

		Capsule link1_capsule0(Eigen::Vector3d(0, 0, -0.333 / 2), Eigen::Vector3d(0, pi / 2, 0), (0.333 - 0.06 * 2) / 2, 0.06, "link1_capsule0");
		link1.addCollider(link1_capsule0);

		Capsule link1_capsule1(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, pi/2), (0.283 - .06 * 2) / 2, 0.06, "link1_capsule1");
		link1.addCollider(link1_capsule1);

		Capsule link2_capsule0(Eigen::Vector3d(0, -.316/2, 0), Eigen::Vector3d(0, 0, pi/2), (0.316 - .06 * 2) / 2, 0.06, "link2_capsule0");
		link2.addCollider(link2_capsule0);

		// 0.0825?
		Capsule link3_capsule0(Eigen::Vector3d(0.0825, 0, 0), Eigen::Vector3d(0, 0, pi/2), (0.220 - .06 * 2) / 2, 0.06, "link3_capsule0");
		link3.addCollider(link3_capsule0);

		Capsule link4_capsule0(Eigen::Vector3d(-0.0825, 0.384/2, 0), Eigen::Vector3d(0, 0, pi/2), (0.384 - .06 * 2) / 2, 0.06, "link4_capsule0");
		link4.addCollider(link4_capsule0);

		Capsule link5_capsule0(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, pi/2), (0.2 - .06 * 2) / 2, 0.06, "link5_capsule0");
		link5.addCollider(link5_capsule0);

		Capsule link6_capsule0(Eigen::Vector3d(0.088, 0, 0), Eigen::Vector3d(0, 0, pi/2), ((0.107*2) - .06 * 2) / 2, 0.06, "link6_capsule0");
		link6.addCollider(link6_capsule0);

		Sphere link6_sphere0(Eigen::Vector3d(0.088/2, 0.06/2, 0), Eigen::Vector3d(0, 0, 0), 0.06, "link6_sphere0");
		link6.addCollider(link6_sphere0);

		Box link7_box0(Eigen::Vector3d(0, 0, 0.1070 - (0.02/2)), Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0.05, 0.05, 0.02), "link7_box0");
		link7.addCollider(link7_box0);
	}


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

	// Finish initialization.
	m_rigidBodyChain.postInit();
}