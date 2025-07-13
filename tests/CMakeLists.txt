add_executable(sPIDControllerTests 
    sPIDControllerTests.cpp 
)

target_link_libraries(sPIDControllerTests 
    PRIVATE 
        GTest::gtest_main
        sPIDController
)

target_include_directories(sPIDControllerTests
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/src
)

include (GoogleTest)
gtest_discover_tests(sPIDControllerTests)