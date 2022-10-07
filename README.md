# LCPMotionPlanner
## Summary
LCPMotionPlanner is a task space motion planner with obstacle avoidance for spatial manipulators. 

The motion planning is screw linear interpolation (ScLERP) based, and the obstacle avoidance is formulated as a linear complementarity problem (LCP). The benefit of using this approach is the natural expression of task space constraints, even with collision avoidance. Additionally, this approach is much more computationally efficient than random sampling.

<p align="center">
  <img src="docs/LCPMotionPlanner.png"
  width = auto
  height = auto />
</p>

## Supported Platforms
Currently, only Windows and Linux are supported. For Windows, builds have been tested on Windows 11 with MSVC 17. For Linux, builds have been tested on Ubuntu 20.04 with GCC 9.4.0.

## Dependencies
LCPMotionPlanner has two dependencies, Eigen and PhysX. They are included as submodules of this repository and handled by CMake, so no extra effort is required to get started.

If you are on Linux, you may need some extra packages for PhysX. Run the below command to ensure you do not get any issues related to missing header files.

```bash
sudo apt-get install build-essential libxxf86vm-dev libgl1-mesa-dev libglu1-mesa-dev gcc-multilib g++-multilib freeglut3-dev lib32z1
```

## Build Configurations
PhysX ships with four build configurations. Prefer to use the debug or release mode as they are most stable with LCPMotionPlanner.
"debug": Full debug build of PhysX + LCPMotionPlanner with checks and visualization.
"checked": Release build of PhysX with checks and visualization + release build of LCPMotionPlanner with debugging.
"profile": Release build of PhysX without chceks but still with visualization + release build of LCPMotionPlanner with debugging.
"release": Full release build of PhysX + LCPMotionPlanner with no visualization or debugging.

## Build Instructions
Clone the repository and setup submodules.
```bash
git clone https://github.com/Tom-Forsyth/LCPMotionPlanner.git
cd LCPMotionPlanner
git submodule init
git submodule update --recursive
```

Run CMake with the desired build configuration to generate a makefile (Linux) or Visual Studio solution (Windows).
```bash
cmake -B build -DCMAKE_BUILD_TYPE=release
```

If you are on Linux, build the project using the make file, and run the code.
```bash
cd build
make -j20
./LCPMotionPlanner
```

If you are on Windows, open "LCPMotionPlanner.sln" inside of the build directory. Press F7 to compile the project, and run the code.

## Debug Visualization
Currently, visualization is only supported through PhysX Visual Debugger, which is only available for Windows. PhysX provides primative a rendering application for scenes for Linux that will be supported shortly.

With PhysX Visual Debugger open, run the project in debug mode, and the primatives of the scene will appear in the PVD player.

## High Fidelity Visualization
Realistic visualation is available through UnrealSim, where we are using this module to generate collision free plans for the Franka Panda manipulator. Objects can be placed into the UnrealSim simulation scene, and the objects will then be loaded in as primatives into this library to generate and visualze a collision free plan. See more:

## Example Use
Below can find an example of how to create a manipulator, add obstacles, and generate a collision free motion plan. The public facing API of this project is provided through UnrealSim, so the internal API must be used here.

```cpp
#include "Sphere.h"
#include "Box.h"
#include "FrankaPanda.h"
#include "ObjectType.h"
#include "PhysicsCore.h"
#include "PhysicsScene.h"
#include <Eigen/Dense>
#include <iostream>

void generateMotionPlan()
{
	constexpr double pi = 3.14159265358979323846;

	// Create physics core and scene.
	MotionPlanner::PhysicsCore physics;
	physics.createPhysicsCore();
	MotionPlanner::PhysicsScene* physicsScene = physics.createPhysicsScene("MyTestScene");

	// Table obstacle parameters.
	Eigen::Vector3d tableOrigin(1.5, 1, 0.25);
	Eigen::Vector3d tableOffsets(0.3, 0.5, 0.02);
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
	MotionPlanner::Box tableTop(tableOrigin, zeroVec, tableOffsets, "Table Top", tableObjectType);
	MotionPlanner::Box tableLeg1(legOrigin1, zeroVec, legOffsets, "Table Leg 1", tableObjectType);
	MotionPlanner::Box tableLeg2(legOrigin2, zeroVec, legOffsets, "Table Leg 2", tableObjectType);
	MotionPlanner::Box tableLeg3(legOrigin3, zeroVec, legOffsets, "Table Leg 3", tableObjectType);
	MotionPlanner::Box tableLeg4(legOrigin4, zeroVec, legOffsets, "Table Leg 4", tableObjectType);

	// Add table to the scene.
	physicsScene->addObstacle(tableTop);
	physicsScene->addObstacle(tableLeg1);
	physicsScene->addObstacle(tableLeg2);
	physicsScene->addObstacle(tableLeg3);
	physicsScene->addObstacle(tableLeg4);

	// Obstacles on table.
	MotionPlanner::Box boxObstacle(Eigen::Vector3d(1.5, 1, 0.35), zeroVec, Eigen::Vector3d(0.1, 0.1, 0.1), "Box Obstacle", tableObjectType);
	MotionPlanner::Sphere object1(Eigen::Vector3d(1.65, 0.65, 0.32), zeroVec, 0.05, "Object 1", tableObjectType);
	MotionPlanner::Sphere object2(Eigen::Vector3d(1.65, 1.35, 0.32), zeroVec, 0.05, "Object 2", tableObjectType);
	physicsScene->addObstacle(boxObstacle);
	physicsScene->addObstacle(object1);
	physicsScene->addObstacle(object2);

	// Create robot.
	Eigen::Matrix4d pandaBaseTransform{
		{1, 0, 0, 1},
		{0, 1, 0, 1},
		{0, 0, 1, 0},
		{0, 0, 0, 1}
	};
	MotionPlanner::FrankaPanda panda(pandaBaseTransform);

	// Add robot to the scene.
	physicsScene->addSpatialManipulator(panda);

	// Setup start joint angles and transform.
	Eigen::Vector<double, 7> startAngles(0, 0, 0, -pi / 2, 0, pi / 2, 0);
	panda.setJointDisplacements(startAngles);
	Eigen::Matrix4d startTransform = panda.getEndFrameSpatialTransform();

	// Setup goal poses.
	Eigen::Matrix4d goalTransform1(startTransform);
	Eigen::Matrix4d goalTransform2(startTransform);
	const int plan = 0;

	// Pure translation.
	if (plan == 0)
	{
		goalTransform1.block(0, 3, 3, 1) = Eigen::Vector3d(0.65, -0.35, 0.4);
		goalTransform2.block(0, 3, 3, 1) = Eigen::Vector3d(0.65, 0.35, 0.4);
	}

	// Pure rotation.
	if (plan == 1)
	{
		goalTransform1.block(0, 0, 3, 3) = Eigen::Matrix3d{
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1}
		};
		goalTransform2.block(0, 0, 3, 3) = Eigen::Matrix3d{
			{1, 0,  0},
			{0, 0, -1},
			{0, 1,  0}
		};
	}

	// Generate motion plan.
	auto start = std::chrono::steady_clock::now();
	panda.motionPlan(goalTransform1);
	Eigen::Matrix4d achievedTransform1 = panda.getEndFrameSpatialTransform();
	Eigen::VectorXd currentJointAngles = panda.getJointDisplacements();
	panda.setJointDisplacements(currentJointAngles);
	panda.motionPlan(goalTransform2);
	Eigen::Matrix4d achievedTransform2 = panda.getEndFrameSpatialTransform();
	auto stop = std::chrono::steady_clock::now();

	std::cout << "Start Transform: \n" << startTransform << "\n\n";
	std::cout << "Goal Transform 1: \n" << goalTransform1 << "\n\n";
	std::cout << "Acheived Transform 1: \n" << achievedTransform1 << "\n\n";
	std::cout << "Goal Transform 2: \n" << goalTransform2 << "\n\n";
	std::cout << "Acheived Transform 2: \n" << achievedTransform2 << "\n\n";

	// Elapsed time.
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();
	std::cout << "Elapsed Time: " << elapsedTime << " ms\n";
}

int main()
{
    generateMotionPlan();
	return 0;
}
```

