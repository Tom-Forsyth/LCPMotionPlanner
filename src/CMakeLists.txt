# Source files.
set (SOURCES
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

# Header files.
set(INCLUDES
    include/Box.h
    include/Capsule.h
    include/CollisionAggregate.h
    include/ContactManager.h
    include/ContactPoint.h
    include/ContactReportCallback.h
    include/ContactReportFilterShader.h
    include/DualNumber.h
    include/DualQuaternion.h
    include/FrankaPanda.h
    include/Joint.h
    include/Kinematics.h
    include/LCPSolve.h
    include/ManipulatorMotionPlanner.h
    include/MotionPlanningParameters.h
    include/ObjectType.h
    include/PhysicsCore.h
    include/PhysicsScene.h
    include/PhysXHelperFunctions.h
    include/RigidBody.h
    include/RigidBodyChain.h
    include/Shape.h
    include/SpatialManipulator.h
    include/Sphere.h
)

# Create library.
add_library(LCPMotionPlannerLib STATIC
    ${SOURCES}
    ${INCLUDES}
)

# Set include directories.
target_include_directories(LCPMotionPlannerLib 
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link against Eigen and PhysX.
target_link_libraries(LCPMotionPlannerLib
    PUBLIC Eigen3::Eigen
    PUBLIC ${PHYSX_LIBRARIES}
)