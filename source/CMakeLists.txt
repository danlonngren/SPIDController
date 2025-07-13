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
