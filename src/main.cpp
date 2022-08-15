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
//#include "LCPSolve.h"

void testLCPSolve();
void testEigenSpeed();
void testDualNumber();
void testDualQuaternion();
void testGeometry();
void testKinematics();
void testPandaSimulation();

int main()
{
	//testLCPSolve();
	//testEigenSpeed();
	//testDualNumber();
	//testDualQuaternion();
	//testGeometry();
	//testKinematics();
	testPandaSimulation();
	return 0;
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

void testKinematics()
{
	FrankaPanda panda;
	std::vector<double> jointAngles {0, 1, 1};
	panda.setJointDisplacements(jointAngles); 
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
	std::vector<double> jointAngles(7, 0);
	panda.setJointDisplacements(jointAngles);

	/*
	std::vector<double> jointAnglesGoal(7, pi/2);
	jointAnglesGoal[3] = -pi / 2;
	auto start = std::chrono::steady_clock::now();
	for (size_t i = 0; i < 1000; i++)
	{
		double t = i / 1000.0;
		std::vector<double> jointAnglesInterp(7, 0);
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

	*/
}

// ----------
// Next Steps:
//  - Transformation matrix inverse.
//  - Conversion jacobian.
//  - Contact jacobian.
//  - Motion Planner.
// 
// Refactoring: Once planner is working as expected.
//  - Dynamic polymorphism for Shapes.
//  - Review structure and program flow.
//  - Optimize LCP solver.
// 
//----------