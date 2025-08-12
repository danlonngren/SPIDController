include(FetchContent)

# Fetch googletests
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        v1.14.0  # Use any tag, commit hash, or branch
)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE) # For windows

FetchContent_MakeAvailable(googletest)

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