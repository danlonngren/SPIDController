cmake_minimum_required(VERSION 3.28.3)
project(sPIDControllerLib LANGUAGES CXX) 

option(BUILD_TESTS "Build sPIDController tests" OFF)

add_library(simplePIDController STATIC 
    simplePIDController.cpp
    simplePIDController.h
)

target_include_directories(simplePIDController
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/../include  # for IsPIDController.h
        ${CMAKE_CURRENT_SOURCE_DIR}             # for simplePIDController.h
)

target_compile_features(simplePIDController PUBLIC cxx_std_20)

if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()