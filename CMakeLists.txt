cmake_minimum_required(VERSION 3.25)
project(LoadPath LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# CMake seeds MSVC with /EHsc; strip it so the contract below is the only exception setting.
if(MSVC)
	string(REPLACE "/EHsc" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
endif()

option(LOADPATH_BUILD_TESTS "Build the doctest suite" ON)
option(LOADPATH_WARNINGS_AS_ERRORS "Treat warnings as errors" ON)
option(LOADPATH_PROFILE "Fill the phase timings on solver results; the normal build reads no clock" OFF)

# Compiler contract: host-neutral C++20, no exceptions, no RTTI, precise floating point.
# These are the constraints an Unreal module imposes, enforced here so the port is a copy.
add_library(loadpath_contract INTERFACE)
if(MSVC)
	target_compile_options(loadpath_contract INTERFACE
		/W4 /permissive- /utf-8 /Zc:__cplusplus /Zc:preprocessor
		/EHs-c- /GR- /fp:precise
		$<$<BOOL:${LOADPATH_WARNINGS_AS_ERRORS}>:/WX>)
	target_compile_definitions(loadpath_contract INTERFACE _HAS_EXCEPTIONS=0 NOMINMAX)
else()
	target_compile_options(loadpath_contract INTERFACE
		-Wall -Wextra -Wpedantic -Wconversion -Wshadow
		-fno-exceptions -fno-rtti -ffp-contract=off
		$<$<BOOL:${LOADPATH_WARNINGS_AS_ERRORS}>:-Werror>)
endif()
if(LOADPATH_PROFILE)
	target_compile_definitions(loadpath_contract INTERFACE LOADPATH_PROFILE=1)
endif()

# The engine.
add_library(loadpath STATIC
	src/CapSiteFactor.cpp
	src/Cascade.cpp
	src/Catalogue.cpp
	src/Cli.cpp
	src/CombinedAuthority.cpp
	src/Handles.cpp
	src/LimitAnalysisCertificate.cpp
	src/RbsmStandingAuthority.cpp
	src/StaticsCommon.cpp
	src/Structure.cpp
	src/StructureFile.cpp
	src/Units.cpp
	src/Vec3.cpp
	src/geometry/BoxContacts.cpp
	src/geometry/RunningBondWall.cpp
	src/geometry/Warehouse.cpp)
target_include_directories(loadpath PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# Eigen stays behind the solver interface: private, SYSTEM so /W4 /WX does not fire inside it, MPL2 only.
target_include_directories(loadpath SYSTEM PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen)
target_compile_definitions(loadpath PRIVATE EIGEN_MPL2_ONLY)
target_link_libraries(loadpath PUBLIC loadpath_contract)

add_executable(loadpath_cli cli/Main.cpp)
target_link_libraries(loadpath_cli PRIVATE loadpath)

if(LOADPATH_BUILD_TESTS)
	enable_testing()
	add_subdirectory(tests)
endif()
