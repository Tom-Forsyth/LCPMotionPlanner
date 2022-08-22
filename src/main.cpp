#include <iostream>
#include <Eigen/Dense>
#include "PxPhysicsAPI.h"
#include <chrono>

#include "Sphere.h"
#include "Capsule.h"
#include "Box.h"
#include "CollisionAggregate.h"

#include "Joint.h"
#include "RigidBody.h"
#include "RigidBodyChain.h"
#include "SpatialManipulator.h"
#include "FrankaPanda.h"
#include "PhysXEnv.h"
#include "Kinematics.h"
#include "MotionPlanner.h"

void testTransform();
void testTransInv();
void testLCPSolve();
void testEigenSpeed();
void testDualNumber();
void testDualQuaternion();
void testGeometry();
void testPandaSimulation();
void testMotionPlan();

int main()
{
	// Round outputs to console.
	std::cout.precision(4);

	// Reset seed.
	srand(time(NULL));

	//testTransform();
	//testTransInv();
	//testLCPSolve();
	//testEigenSpeed();
	//testDualNumber();
	//testDualQuaternion();
	//testGeometry();
	//testPandaSimulation();
	testMotionPlan();
	return 0;
}

void testTransform()
{
	// Create robot.
	Eigen::Matrix4d pandaBaseTransform{
		{1, 0, 0, 1},
		{0, 1, 0, 1},
		{0, 0, 1, 0},
		{0, 0, 0, 1}
	};

	// Create point in world frame.
	Eigen::Vector4d pointWorld{2, 2, 2, 1};

	// Transform point to spatial frame.
	Eigen::Vector4d pointSpatial = Kinematics::transformInverse(pandaBaseTransform) * pointWorld;
	std::cout << pointSpatial << std::endl;
}

void testTransInv()
{
	Eigen::Vector3d axis = {0.3152, 0.752, 0.126};
	double angle = 1.572;
	Eigen::Vector3d translation = {0.3352, 0.5472, 0.562};

	axis.normalize();
	Eigen::Matrix3d R = Kinematics::AxisAngletoRot(axis, angle);

	Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
	T.block(0, 0, 3, 3) = R;
	T.block(0, 3, 3, 1) = translation;

	std::cout << "Transform: \n" << T << std::endl;
	std::cout << "Matrix Inverse: \n" << T.inverse() << std::endl;
	std::cout << "Fast Inverse: \n" << Kinematics::transformInverse(T) << std::endl;
}

void testLCPSolve()
{
	// Init.
	size_t nContacts = 10;
	Eigen::MatrixXd M = Eigen::MatrixXd::Random(nContacts, nContacts);
	Eigen::VectorXd q = Eigen::VectorXd::Random(nContacts);

	// Code.
	auto start = std::chrono::steady_clock::now();
	for (size_t i = 0; i < 1000; i++)
	{
		//LCP sol = LCPSolve(M, q);
	}
	auto stop = std::chrono::steady_clock::now();

	// Elapsed time.
	auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count();
	std::cout << "Elapsed Time: " << elapsedTime << " us" << std::endl;
}

void testEigenSpeed()
{
	// Init.
	Eigen::Matrix4d mat1{
		{0.3425, 0.23561, 0.12461, 0.2152},
		{0.26536, 0.3462, -0.323461, -0.243612},
		{0.12365, -0.2357641, 0.001, 3.2351},
		{0, 0, 0, 1}
	};
	Eigen::Matrix4d mat2{
		{0.57425, 0.5561, 0.22461, 0.7552},
		{0.83536, -0.3462, 0.4623461, -0.8612},
		{0.78365, -0.97641, 0.174, 1.7351},
		{0, 0, 0, 1}
	};

	// Code to be tested.
	size_t numIter = 100;
	auto start = std::chrono::steady_clock::now();
	for (size_t i = 0; i < numIter; i++)
	{
		Eigen::Matrix4d matProd = mat1 * mat2;
	}
	auto stop = std::chrono::steady_clock::now();

	// Elapsed time.
	auto elapsedTime = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count();
	std::cout << "Elapsed Time: " << elapsedTime << " ns" << std::endl;
}

void testDualNumber()
{

}

void testDualQuaternion()
{

}

void testGeometry()
{
	// Sphere.
	Eigen::Vector3d sphereOrigin(0, 0, 0);
	Eigen::Vector3d sphereRPY(0, 0, 0);
	double sphereRadius = 0.2;
	Sphere sphere(sphereOrigin, sphereRPY, sphereRadius, "sphere");

	// Capsule.
	Eigen::Vector3d capsuleOrigin(5, 0, 0);
	Eigen::Vector3d capsuleRPY(0, 0, 0);
	double capsuleHalfHeight = 0.2;
	double capsuleRadius = 0.1;
	Capsule capsule(capsuleOrigin, capsuleRPY, capsuleHalfHeight, capsuleRadius, "capsule");

	// Box.
	Eigen::Vector3d boxOrigin(10, 0, 0);
	Eigen::Vector3d boxRPY(0, 0, 0);
	Eigen::Vector3d boxRadii(0.3, 0.3, 0.3);
	Box box(boxOrigin, boxRPY, boxRadii, "box");

	// Collision aggregate.
	CollisionAggregate collisionAggregate;
	collisionAggregate.addShape(sphere);
	collisionAggregate.addShape(capsule);
	collisionAggregate.addShape(box);

	// Get shapes in aggregate.
	std::vector<Sphere> spheres = collisionAggregate.getSpheres();
	std::vector<Capsule> capsules = collisionAggregate.getCapsules();
	std::vector<Box> boxes = collisionAggregate.getBoxes();

	// Get transforms of the shapes.
	Eigen::Matrix4d sphereTrans = spheres[0].getTransform();
	Eigen::Matrix4d capsuleTrans = capsules[0].getTransform();
	Eigen::Matrix4d boxTrans = boxes[0].getTransform();

	std::cout << sphereTrans << std::endl;
	std::cout << capsuleTrans << std::endl;
	std::cout << boxTrans << std::endl;
}

void testPandaSimulation()
{
	// Create robot.
	Eigen::Matrix4d pandaBaseTransform{
		{1, 0, 0, 1},
		{0, 1, 0, 1},
		{0, 0, 1, 0},
		{0, 0, 0, 1}
	};
	FrankaPanda panda(pandaBaseTransform);

	// Add obstacles.
	Sphere sphereObstacle(Eigen::Vector3d(1.0, 1.25, 0.65), Eigen::Vector3d(0, 0, 0), 0.1, "sphereObstacle");
	Sphere sphereObstacle2(Eigen::Vector3d(1.25, 1.0, 0.45), Eigen::Vector3d(0, 0, 0), 0.1, "sphereObstacle2");
	panda.addObstacle(sphereObstacle);
	panda.addObstacle(sphereObstacle2);

	// Update joint angles.
	double pi = 3.1415;
	double increment = 0.005 * pi/2;
	Eigen::VectorXd jointAngles = Eigen::Vector<double, 7>::Zero();
	panda.setJointDisplacements(jointAngles);

	std::vector<double> jointAnglesGoal(7, pi/2);
	jointAnglesGoal[3] = -pi / 2;
	auto start = std::chrono::steady_clock::now();
	for (size_t i = 0; i < 1000; i++)
	{
		double t = i / 1000.0;
		Eigen::VectorXd jointAnglesInterp = Eigen::Vector<double, 7>::Zero();
		for (size_t i = 0; i < jointAngles.size(); i++)
		{
			jointAnglesInterp[i] = (1 - t) * jointAngles[i] + t * jointAnglesGoal[i];
		}
		panda.setJointDisplacements(jointAnglesInterp);
	}
	auto stop = std::chrono::steady_clock::now();

	// Elapsed time.
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();
	std::cout << "Elapsed Time: " << elapsedTime << " ms" << std::endl;

	// Currently seeing ~13 ms / 1000 iterations with linear interpolation & 2 sphere obstacles.
	// ~14-15 ms with 2 box obstacles.
}

void testMotionPlan()
{
	constexpr double pi = 3.14159265358979323846;

	// Create robot.
	Eigen::Matrix4d pandaBaseTransform{
		{1, 0, 0, 1},
		{0, 1, 0, 1},
		{0, 0, 1, 0},
		{0, 0, 0, 1}
	};
	FrankaPanda panda(pandaBaseTransform);

	// Table obstacle parameters.
	Eigen::Vector3d tableOrigin(1.5, 1, 0.25);
	Eigen::Vector3d tableOffsets(0.3, 0.5, 0.02);
	double legLength = tableOrigin(2) - tableOffsets(2);
	Eigen::Vector3d legOffsets(0.03, 0.03, legLength/2);
	double legZVal = legLength / 2;

	// Table top.
	Box tableTop(tableOrigin, Eigen::Vector3d(0, 0, 0), tableOffsets, "Table Top");
	panda.addObstacle(tableTop);

	// Table legs.
	Box tableLeg1(Eigen::Vector3d(1.5 - 0.3 + legOffsets(0), 1 - 0.5 + legOffsets(1), legZVal), Eigen::Vector3d(0, 0, 0), legOffsets, "Table Leg 1");
	panda.addObstacle(tableLeg1);
	Box tableLeg2(Eigen::Vector3d(1.5 - 0.3 + legOffsets(0), 1 + 0.5 - legOffsets(1), legZVal), Eigen::Vector3d(0, 0, 0), legOffsets, "Table Leg 2");
	panda.addObstacle(tableLeg2);
	Box tableLeg3(Eigen::Vector3d(1.5 + 0.3 - legOffsets(0), 1 - 0.5 + legOffsets(1), legZVal), Eigen::Vector3d(0, 0, 0), legOffsets, "Table Leg 3");
	panda.addObstacle(tableLeg3);
	Box tableLeg4(Eigen::Vector3d(1.5 + 0.3 - legOffsets(0), 1 + 0.5 - legOffsets(1), legZVal), Eigen::Vector3d(0, 0, 0), legOffsets, "Table Leg 4");
	panda.addObstacle(tableLeg4);

	// Obstacle on table.
	Box boxObstacle(Eigen::Vector3d(1.5, 1, 0.35), Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0.1, 0.1, 0.1), "Box Obstacle");
	panda.addObstacle(boxObstacle);

	// Objects on table to pickup.
	Sphere object1(Eigen::Vector3d(1.65, 0.65, 0.32), Eigen::Vector3d(0, 0, 0), 0.05, "Object 1");
	panda.addObstacle(object1);
	Sphere object2(Eigen::Vector3d(1.65, 1.35, 0.32), Eigen::Vector3d(0, 0, 0), 0.05, "Object 2");
	panda.addObstacle(object2);

	// Setup start joint angles and transform.
	Eigen::Vector<double, 7> startAngles(0, 0, 0, -pi / 2, 0, pi / 2, 0);
	panda.setJointDisplacements(startAngles);
	Eigen::Matrix4d startTransform = panda.getEndFrameSpatialTransform();

	// Setup goal poses.
	Eigen::Matrix4d goalTransform1(startTransform);
	goalTransform1.block(0, 3, 3, 1) = Eigen::Vector3d(0.65, -0.35, 0.5);
	Eigen::Matrix4d goalTransform2(startTransform);
	Eigen::Matrix4d goalTransform3(startTransform);
	goalTransform3.block(0, 3, 3, 1) = Eigen::Vector3d(0.65, 0.35, 0.5);

	// Generate motion plan.
	auto start = std::chrono::steady_clock::now();
	panda.motionPlan(goalTransform1);
	//panda.motionPlan(goalTransform2);
	panda.motionPlan(goalTransform3);
	auto stop = std::chrono::steady_clock::now();

	std::cout << "Start Transform: \n" << startTransform << "\n\n";
	std::cout << "Goal Transform: \n" << goalTransform1 << "\n\n";
	std::cout << "Acheived Transform: \n" << panda.getEndFrameSpatialTransform() << "\n\n";

	// Elapsed time.
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();
	std::cout << "Elapsed Time: " << elapsedTime << " ms" << std::endl;

	// ~0.125 ms/iteration, or 8000 hz.
	
}

// ----------
// Next Steps:
//  + Transformation matrix inverse.
//  + Conversion jacobian.
//  + Contact jacobian.
//  + Motion Planner
// 
// Refactoring: Once planner is working as expected.
//	- B Matrix.
//  - Dynamic polymorphism for Shapes.
//  - Review structure and program flow.
//  - Optimize LCP solver.
//  - Link safety distance.
//  - Return EndFrame (and others) by reference, not copy.
//  - Sparse multiplication with zero-padded jacobians.
//  - Check if initial config is within range of goal to avoid divide by zero/nan issues.
//	- Eigen AxisAngle(Quaternion) wrong? Representation singularity.
//  - Pow of DualQuat needs fixing and optimization.
//  - Investigate why orientation is correct until EE is close to goal position, then goes wild.
//  - Review dual quaternion power and cleanup.
//  - Last box doesn't have collision?
//  - Contact points switch between links and obstacles... how to make consistant?
// 
//----------



//---------
// ScLERP Issue: Not interpolating correctly.
//	- Will either not generate configs between values, or will flip them.
// 
// Ideas:
//	- Storage direction of scalar -> vec or vec -> scalar in Eigen::Quaternion is causing issues with our kinematics.
//---------