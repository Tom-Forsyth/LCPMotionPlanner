# Version & C++ standard.
cmake_minimum_required(VERSION 3.16.3)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Project setup.
project(LCPMotionPlanner
    VERSION 1.0
    DESCRIPTION "A task space motion planner for manipulators with obstacle avoidance."
    HOMEPAGE_URL "https://github.com/Tom-Forsyth/LCPMotionPlanner"
    LANGUAGES C CXX
)

# Setup Eigen3.
if (WIN32)
    set(EIGEN_PATH "external/eigen")
    list(APPEND CMAKE_MODULE_PATH ${EIGEN_PATH}/cmake)
    set(EIGEN3_INCLUDE_DIR ${EIGEN_PATH})
endif()
find_package(Eigen3 REQUIRED)

# Setup PhysX.
if (WIN32)

endif()

# Add subdirectories.
add_subdirectory(src)

# Create executable.
add_executable(LCPMotionPlannerTest src/main.cpp)
target_link_libraries(LCPMotionPlannerTest 
    PUBLIC LCPMotionPlannerLib
    PUBLIC Eigen3::Eigen
)