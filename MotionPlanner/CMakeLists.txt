cmake_minimum_required(VERSION 3.16)

project(MotionPlanner
	VERSION 1.0
	DESCRIPTION "Spatial manipulator ScLERP motion planning with LCP based collision avoidance."
	LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE RELEASE)
set(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O1")

include_directories(
    include
)

add_executable(main
	src/main.cpp
	src/LCPSolve.cpp
	src/SphereObstacle.cpp
	src/SpatialManipulator.cpp
	src/kinematics.cpp
	src/DualQuaternion.cpp
	src/DualNumber.cpp
	include/LCPSolve.h
	include/SphereObstacle.h
	include/SpatialManipulator.h
	include/kinematics.h
	include/DualQuaternion.h
	include/DualNumber.h
)
