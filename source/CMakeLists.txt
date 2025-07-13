add_library(sPIDController STATIC 
    sPIDController.cpp
    sPIDController.h
)

target_include_directories(sPIDController
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/../include  # for IsPIDController.h
        ${CMAKE_CURRENT_SOURCE_DIR}             # for sPIDController.h
)

target_compile_features(sPIDController PUBLIC cxx_std_20)
