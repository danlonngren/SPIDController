if (BUILD_TESTING)
    include_directories(${CMAKE_SOURCE_DIR}/simplePIDController)

    # Setup test executable
    add_executable(sPIDControllerTests 
        sPIDControllerTests.cpp
    )
    
    target_link_libraries(sPIDControllerTests PUBLIC 
        GTest::gtest_main
        simplePIDController
    )

    # # Include gtest and all tests in executable
    # # Include GoogleTest module to enable test discovery and integration
    # include (GoogleTest)
    # gtest_discover_tests(sPIDControllerTests)

    add_test(
        NAME sPIDControllerTests
        COMMAND sPIDControllerTests
    )
endif()