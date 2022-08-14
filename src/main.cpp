#include <iostream>
#include <Eigen/Dense>
#include "PxPhysicsAPI.h"

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

void testDualNumber();
void testDualQuaternion();
void testGeometry();
void testKinematics();
void testPandaSimulation();

int main()
{
	//testDualNumber();
	//testDualQuaternion();
	//testGeometry();
	//testKinematics();
	testPandaSimulation();
	return 0;
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
	std::vector<double> jointAnglesGoal(7, pi/2);
	jointAnglesGoal[3] = -pi / 2;
	for (size_t i = 0; i < 100; i++)
	{
		double t = i / 100.0;
		std::vector<double> jointAnglesInterp(7, 0);
		for (size_t i = 0; i < jointAngles.size(); i++)
		{
			jointAnglesInterp[i] = (1 - t) * jointAngles[i] + t * jointAnglesGoal[i];
		}
		panda.setJointDisplacements(jointAnglesInterp);
	}
}
