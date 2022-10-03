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
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/external/eigen/cmake)
set(EIGEN3_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/external/eigen)
find_package(Eigen3 REQUIRED)

# Setup PhysX.
set(PHYSX_PATH ${CMAKE_SOURCE_DIR}/external/PhysX/physx)

#### Build PhysX library ####
# PHYSX_PATH - path to the `{cloned repository}/physx` repo directory git://github.com/NVIDIAGameWorks/PhysX.git
set( PHYSX_ROOT_DIR ${PHYSX_PATH} ) #This is needed for $ENV{PHYSX_PATH}/compiler/public/CMakeLists.txt
set( PHYSX_INCLUDE_DIRS ${PHYSX_PATH}/include/ ${PHYSX_PATH}/../pxshared/include/ )
set( PHYSX_LIBRARIES
	PhysXExtensions
	PhysX
	PhysXPvdSDK
	PhysXVehicle
	PhysXCharacterKinematic
	PhysXCooking
	PhysXCommon
	PhysXFoundation
	# SnippetUtils
)

set(TARGET_BUILD_PLATFORM "windows") # has to match the TARGET_BUILD_PLATFORM in $ENV{PHYSX_PATH}/physix/buildtools/cmake_generate_projects.py
set(PX_BUILDSNIPPETS OFF CACHE BOOL "Generate the snippets")
set(PX_BUILDPUBLICSAMPLES OFF CACHE BOOL "Generate the samples projects")
set(PX_GENERATE_STATIC_LIBRARIES ON CACHE BOOL "Generate static libraries")
set(PX_FLOAT_POINT_PRECISE_MATH OFF CACHE BOOL "Float point precise math")
set(NV_USE_STATIC_WINCRT ON CACHE BOOL "Use the statically linked windows CRT")
set(NV_USE_DEBUG_WINCRT ON CACHE BOOL "Use the debug version of the CRT")
set(PXSHARED_PATH ${PHYSX_PATH}/../pxshared)
set(PXSHARED_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
set(CMAKEMODULES_VERSION "1.27")
set(CMAKEMODULES_PATH ${PHYSX_PATH}/../externals/cmakemodules)
set(PX_OUTPUT_LIB_DIR ${CMAKE_CURRENT_BINARY_DIR}/externals/physx)
set(PX_OUTPUT_BIN_DIR ${CMAKE_CURRENT_BINARY_DIR}/externals/physx)
  
# Call into PhysX's CMake scripts
add_subdirectory(${PHYSX_PATH}/compiler/public external/physx)

# Add physx libraries to target
#target_link_libraries(target_name PUBLIC ${PHYSX_LIBRARIES})
#### Windows only: Copy the Physx dll files to the simulation executable####


#set(PhysX_DIR "C:/src/PhysX/build/sdk_source_bin/")
#find_package(PhysX REQUIRED)

# Add subdirectories.
add_subdirectory(src)

# Create executable.
add_executable(LCPMotionPlannerTest src/main.cpp)
target_link_libraries(LCPMotionPlannerTest 
    PUBLIC LCPMotionPlannerLib
    PUBLIC Eigen3::Eigen
)

target_link_libraries(LCPMotionPlannerTest PUBLIC ${PHYSX_LIBRARIES})
if (TARGET_BUILD_PLATFORM STREQUAL "windows")
	# References NvidiaBuildOptions.cmake to figure out if system is 32/64 bit
	IF(CMAKE_SIZEOF_VOID_P EQUAL 8)
		SET(LIBPATH_SUFFIX "64")
	ELSE()
		SET(LIBPATH_SUFFIX "32")
	ENDIF()
	GetPlatformBinName(PLATFORM_BIN_NAME ${LIBPATH_SUFFIX})
	set(PhysxOutputPath ${PX_OUTPUT_LIB_DIR}/bin/${PLATFORM_BIN_NAME}/)
	message("Physx Output Path: " ${PhysxOutputPath})

	# copy PhysX dll's to build dir. Happens on every build.
	add_custom_command(TARGET LCPMotionPlannerTest POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_directory "${PhysxOutputPath}" "build")
endif()