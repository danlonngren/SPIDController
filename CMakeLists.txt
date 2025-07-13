cmake_minimum_required(VERSION 3.28.3)

# Enable use of VERSION in project()
if(POLICY CMP0048)
    cmake_policy(SET CMP0048 NEW)
endif()

project(sPIDControllerLib VERSION 0.1.1 DESCRIPTION "Simple PID Controller" LANGUAGES CXX) 

set(CMAKE_CXX_STANDARD 20)

# Include to enable testing and set BUILD_TESTING var
# TODO: Set this depending if release or debug build
include(CTest)

# Add all sub directories
add_subdirectory(external)
add_subdirectory(simplePIDController)