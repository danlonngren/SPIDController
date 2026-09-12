cmake_minimum_required(VERSION 3.28.3)

project(SimplePIDController
    VERSION 1.0.0
    DESCRIPTION "A lightweight C++ PID controller"
    LANGUAGES CXX
)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)


option(SPIDCONTROLLER_ENABLE_ANALYSIS "Enable the static-analysis target" OFF)
option(SPIDCONTROLLER_WARNINGS_AS_ERRORS "Treat project warnings as errors" OFF)
option(BUILD_TESTING "Build unit tests" OFF)

add_library(SimplePIDController STATIC
    src/simplePIDController.cpp
)
add_library(SimplePIDController::SimplePIDController ALIAS SimplePIDController)

target_compile_features(SimplePIDController PUBLIC cxx_std_20)
set_target_properties(SimplePIDController PROPERTIES CXX_EXTENSIONS OFF)

target_include_directories(SimplePIDController
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>"
        "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

if(MSVC)
    target_compile_options(SimplePIDController PRIVATE /W4 /permissive-)
else()
    target_compile_options(SimplePIDController PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -Wconversion
        -Wsign-conversion
        -Wshadow
        -Wnon-virtual-dtor
        -Wold-style-cast
        -Wcast-align
        -Wunused
        -Woverloaded-virtual
        -Wnull-dereference
        -Wdouble-promotion
        -Wformat=2
    )
endif()

if(SPIDCONTROLLER_WARNINGS_AS_ERRORS)
    set_property(TARGET SimplePIDController PROPERTY COMPILE_WARNING_AS_ERROR ON)
endif()

include(CTest)
if(BUILD_TESTING)
    add_subdirectory(tests)
endif()

if(SPIDCONTROLLER_ENABLE_ANALYSIS)
    find_program(RUN_CLANG_TIDY NAMES run-clang-tidy REQUIRED)
    find_program(CPPCHECK NAMES cppcheck REQUIRED)

    set(_clang_tidy_arguments -p=${CMAKE_BINARY_DIR})
    if(APPLE)
        find_program(XCRUN NAMES xcrun REQUIRED)
        execute_process(
            COMMAND "${XCRUN}" --show-sdk-path
            OUTPUT_VARIABLE APPLE_SDK_PATH
            OUTPUT_STRIP_TRAILING_WHITESPACE
            COMMAND_ERROR_IS_FATAL ANY
        )
        list(APPEND _clang_tidy_arguments
            -extra-arg-before=-isysroot
            -extra-arg-before=${APPLE_SDK_PATH}
        )
    endif()

    add_custom_target(static-analysis
        COMMAND "${RUN_CLANG_TIDY}" ${_clang_tidy_arguments}
        COMMAND "${CPPCHECK}"
            --project=${CMAKE_BINARY_DIR}/compile_commands.json
            --enable=warning,style,performance,portability
            --inline-suppr
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        COMMENT "Running clang-tidy and cppcheck"
        VERBATIM
    )
endif()

install(TARGETS SimplePIDController
    EXPORT SimplePIDControllerTargets
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
    INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)

install(DIRECTORY inc/ DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")

install(EXPORT SimplePIDControllerTargets
    FILE SimplePIDControllerTargets.cmake
    NAMESPACE SimplePIDController::
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/SimplePIDController"
)

configure_package_config_file(
    cmake/SimplePIDControllerConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/SimplePIDControllerConfig.cmake"
    INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/SimplePIDController"
)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/SimplePIDControllerConfigVersion.cmake"
    VERSION "${PROJECT_VERSION}"
    COMPATIBILITY SameMajorVersion
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/SimplePIDControllerConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/SimplePIDControllerConfigVersion.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/SimplePIDController"
)

set(CPACK_PACKAGE_NAME "${PROJECT_NAME}")
set(CPACK_PACKAGE_VENDOR "SimplePIDController")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_GENERATOR "TGZ;ZIP")
include(CPack)
