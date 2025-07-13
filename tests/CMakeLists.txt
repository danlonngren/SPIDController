add_executable(simplePIDControllerTests 
    simplePIDControllerTests.cpp 
)

target_link_libraries(simplePIDControllerTests 
    PRIVATE 
        GTest::gtest_main
        simplePIDController
)

target_include_directories(simplePIDControllerTests
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/source
)

include (GoogleTest)
gtest_discover_tests(simplePIDControllerTests)