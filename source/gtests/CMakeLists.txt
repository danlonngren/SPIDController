if (BUILD_TESTING)
    # Include the circularBufferStatic library
    include_directories(${CMAKE_SOURCE_DIR}/circularBufferStatic)

    set(SOURCES
        sPIDControllerTests.cpp
    )

    # Setup test executable
    add_executable(sPIDControllerTests 
        ${SOURCES}
    )
    
    target_link_libraries(sPIDControllerTests PUBLIC 
        GTest::gtest_main
        circularBufferStatic
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