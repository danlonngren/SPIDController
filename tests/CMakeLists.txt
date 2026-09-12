find_package(GTest 1.14 CONFIG QUIET)

if(NOT GTest_FOUND)
    include(FetchContent)

    set(INSTALL_GTEST OFF CACHE BOOL "Do not install GoogleTest" FORCE)
    FetchContent_Declare(googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
        GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(googletest)
endif()

add_executable(SimplePIDControllerTests
    simplePIDControllerTests.cpp
)

target_link_libraries(SimplePIDControllerTests PRIVATE
    GTest::gtest_main
    SimplePIDController::SimplePIDController
)

include(GoogleTest)
gtest_discover_tests(SimplePIDControllerTests DISCOVERY_MODE PRE_TEST)
