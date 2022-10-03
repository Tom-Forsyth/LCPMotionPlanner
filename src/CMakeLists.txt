# Create library.
add_library(LCPMotionPlannerLib STATIC
    Box.cpp
    Capsule.cpp
    CollisionAggregate.cpp
    ContactManager.cpp
    ContactPoint.cpp
    ContactReportCallback.cpp
    ContactReportFilterShader.cpp
    DualNumber.cpp
    DualQuaternion.cpp
    FrankaPanda.cpp
    Joint.cpp
    Kinematics.cpp
    LCPSolve.cpp
    ManipulatorMotionPlanner.cpp
    PhysicsCore.cpp
    PhysicsScene.cpp
    PhysXHelperFunctions.cpp
    RigidBody.cpp
    RigidBodyChain.cpp
    Shape.cpp
    SpatialManipulator.cpp
    Sphere.cpp
)

# Set include directories.
target_include_directories(LCPMotionPlannerLib 
    PUBLIC ${CMAKE_SOURCE_DIR}/include
)

# Link against Eigen and PhysX.
target_link_libraries(LCPMotionPlannerLib
    PUBLIC Eigen3::Eigen
    PUBLIC ${PHYSX_LIBRARIES}
)