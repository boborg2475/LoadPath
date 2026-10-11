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

# The engine. Header-only until the first slice adds a source file; then STATIC.
add_library(loadpath INTERFACE)
target_include_directories(loadpath INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(loadpath INTERFACE loadpath_contract)

if(LOADPATH_BUILD_TESTS)
	enable_testing()
	add_subdirectory(tests)
endif()
