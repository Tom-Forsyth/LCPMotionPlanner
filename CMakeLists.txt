# CMake version.
cmake_minimum_required(VERSION 3.16.3)

# Project setup.
project(LCPMotionPlanner
    VERSION 1.0
    DESCRIPTION "A task space motion planner for manipulators with obstacle avoidance."
    HOMEPAGE_URL "https://github.com/Tom-Forsyth/LCPMotionPlanner"
    LANGUAGES C CXX
)

# C++ standard.
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Setup Eigen3.
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/external/eigen/cmake)
set(EIGEN3_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/external/eigen)
find_package(Eigen3 REQUIRED)

# Set PhysX paths and libraries.
set(PHYSX_PATH ${CMAKE_SOURCE_DIR}/external/PhysX/physx)
set(PHYSX_ROOT_DIR ${PHYSX_PATH}) #This is needed for $ENV{PHYSX_PATH}/compiler/public/CMakeLists.txt
set(PHYSX_INCLUDE_DIRS ${PHYSX_PATH}/include/ ${PHYSX_PATH}/../pxshared/include/)
set(PHYSX_LIBRARIES
	PhysXExtensions
	PhysX
	PhysXPvdSDK
	PhysXVehicle
	PhysXCharacterKinematic
	PhysXCooking
	PhysXCommon
	PhysXFoundation
)

# Setup PhysX build platform.
if(WIN32 AND CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
	set(TARGET_BUILD_PLATFORM "windows")

	# Windows Only - These were forgotten in PhysX's Windows CMakeLists.txt.
	SET(CMAKE_EXE_LINKER_FLAGS_PROFILE "/DEBUG")
	SET(CMAKE_EXE_LINKER_FLAGS_CHECKED "/DEBUG")	
elseif(UNIX AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
	set(TARGET_BUILD_PLATFORM "linux")
else()
	message(FATAL_ERROR "Only Windows (MSVC) and Linux (GNU) platforms are supported.")
endif()

# Set PhysX variables.
set(PX_BUILDSNIPPETS OFF CACHE BOOL "Generate the snippets")
set(PX_BUILDPUBLICSAMPLES OFF CACHE BOOL "Generate the samples projects")
set(PX_GENERATE_STATIC_LIBRARIES ON CACHE BOOL "Generate static libraries")
set(PX_FLOAT_POINT_PRECISE_MATH ON CACHE BOOL "Float point precise math")
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

# Linux/GCC specific flags to avoid PhysX build errors as of GCC 9.4.0.
if (TARGET_BUILD_PLATFORM STREQUAL "linux")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
	target_compile_options(PhysXCommon PRIVATE "-Wno-error=restrict")
	target_compile_options(PhysXExtensions PRIVATE "-Wno-error=class-memaccess")
endif()

# LCPSolve.
add_subdirectory(external/LCPSolve)

# Add source code subdirectory to create LCPMotionPlannerLib.
add_subdirectory(src)

# Create test executable.
add_executable(LCPMotionPlannerTest src/main.cpp)
target_link_libraries(LCPMotionPlannerTest 
    PUBLIC LCPMotionPlannerLib
)
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT LCPMotionPlannerTest)

# Ensure a valid build type is specified.
if (NOT DEFINED CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
	message(FATAL_ERROR "Please specify a build type with -DCMAKE_BUILD_TYPE=<build type>.")
endif()
if((NOT ${CMAKE_BUILD_TYPE} STREQUAL "debug") AND (NOT ${CMAKE_BUILD_TYPE} STREQUAL "checked") AND 
(NOT ${CMAKE_BUILD_TYPE} STREQUAL "profile") AND (NOT ${CMAKE_BUILD_TYPE} STREQUAL "release"))
	message(FATAL_ERROR "Invalid build type! Please use one of the following: 'debug', 'checked', 'profile', or 'release'.")
endif()

# Ensure that only the specified build type is available in IDEs.
set(CMAKE_CONFIGURATION_TYPES "${CMAKE_BUILD_TYPE}")

# LCPMotionPlanner compile options.
if (TARGET_BUILD_PLATFORM STREQUAL "linux")
	if (${CMAKE_BUILD_TYPE} STREQUAL "debug")
		set(LCPFlags "-D_DEBUG" "-g")
	elseif(${CMAKE_BUILD_TYPE} STREQUAL "checked")
		set(LCPFlags "-D_DEBUG" "-O2" "-fopenmp" "-g")
	elseif(${CMAKE_BUILD_TYPE} STREQUAL "profile")
		set(LCPFlags "-D_DEBUG" "-O2" "-fopenmp" "-g")
	elseif(${CMAKE_BUILD_TYPE} STREQUAL "release")
		set(LCPFlags "-fopenmp" "-O2")
	endif()
elseif (TARGET_BUILD_PLATFORM STREQUAL "windows")
	if (${CMAKE_BUILD_TYPE} STREQUAL "debug")
		set(LCPFlags "/MP" "/D_DEBUG")
	elseif(${CMAKE_BUILD_TYPE} STREQUAL "checked")
		set(LCPFlags "/MP" "/openmp" "/O2" "/D_DEBUG" "/Zo")
	elseif(${CMAKE_BUILD_TYPE} STREQUAL "profile")
		set(LCPFlags "/MP" "/openmp" "/O2" "/D_DEBUG" "/Zo")
	elseif(${CMAKE_BUILD_TYPE} STREQUAL "release")
		set(LCPFlags "/MP" "/openmp" "/O2")
	endif()
endif()

# Set flags.
target_compile_options(LCPMotionPlannerLib PRIVATE ${LCPFlags})
target_compile_options(LCPMotionPlannerTest PRIVATE ${LCPFlags})

# Windows only: Copy DLL files to directory of executable post build.
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