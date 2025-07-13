add_executable(simplePIDControllerTests 
    simplePIDControllerTests.cpp 
)

target_link_libraries(simplePIDControllerTests 
    PRIVATE 
        GTest::gtest_main
        sPIDController
)

target_include_directories(simplePIDControllerTests
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/src
)

include (GoogleTest)
gtest_discover_tests(simplePIDControllerTests)