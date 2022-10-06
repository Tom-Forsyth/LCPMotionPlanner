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
if(WIN32)
	set(TARGET_BUILD_PLATFORM "windows")

	# Windows Only - These were forgotten in PhysX's Windows CMakeLists.txt.
	SET(CMAKE_EXE_LINKER_FLAGS_PROFILE "/DEBUG")
	SET(CMAKE_EXE_LINKER_FLAGS_CHECKED "/DEBUG")	
endif()
if(UNIX)
	set(TARGET_BUILD_PLATFORM "linux")
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

# Disable warning errors for PhysX bugs on linux.
if (UNIX)
	#set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -Wno-error=restrict)
	#set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -Wno-error=class-memaccess)
	#fast math, openmp, ndebug/debug

	#target_compile_options(PhysXVehicle PUBLIC -Wno-error
	#set(CMAKE_CXX_FLAGS_DEBUG ${CMAKE_CXX_FLAGS_DEBUG} -g)
	#set(CMAKE_CXX_FLAGS_RELEASE ${CMAKE_CXX_FLAGS_RELEASE} -DNDEBUG)
	#target_compile_definitions(LCPMotionPlannerLib PRIVATE _DEBUG)
	#target_compile_definitions(LCPMotionPlannerTest PRIVATE _DEBUG)
endif()

# Add source code subdirectory to create LCPMotionPlannerLib.
add_subdirectory(src)

# Create test executable.
add_executable(LCPMotionPlannerTest src/main.cpp)
target_link_libraries(LCPMotionPlannerTest 
    PUBLIC LCPMotionPlannerLib
)

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