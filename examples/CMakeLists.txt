# Source files.
set(SOURCES
	SimulationBase.cpp
)

# Header files.
set(HEADERS
	include/SimulationBase.h
)

# Create library.
add_library(LCPMotionPlannerExamples STATIC
    ${SOURCES}
    ${INCLUDES}
)

# Include directory.
target_include_directories(LCPMotionPlannerExamples
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link against Eigen and LCPMotionPlanner.
target_link_libraries(LCPMotionPlannerExamples
	PUBLIC Eigen3::Eigen
	PUBLIC LCPMotionPlannerLib
)
