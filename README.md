# SimplePIDController

A lightweight C++20 PID controller suitable for bare-metal and embedded projects.

## Requirements

- CMake 3.28.3 or newer
- Ninja
- A C++20 compiler
- Internet access on the first test configuration so CMake can download GoogleTest

Static analysis additionally requires `clang-tidy` (or `run-clang-tidy`) and `cppcheck`.

## Build with presets

All generated files, installations, and packages are kept below `build/`.

```bash
# Development library build
cmake --preset debug
cmake --build --preset debug --parallel

# Debug unit-test build
cmake --preset debug-tests
cmake --build --preset debug-tests --parallel
ctest --preset debug-tests

# Optimised library build
cmake --preset release
cmake --build --preset release --parallel

# Optimised unit-test build, intended for release validation / CI
cmake --preset release-tests
cmake --build --preset release-tests --parallel
ctest --preset release-tests
```

Run `cmake --list-presets` to see available configure presets. Re-run the configure command after changing CMake files, presets, the compiler, or configuration options.

## Install and package

The release install prefix is configured as `build/release/install`; no system-wide installation is performed.

```bash
cmake --preset release
cmake --build --preset release --parallel
cmake --install build/release

# Creates .tar.gz and .zip archives in build/packages
cpack --preset release
```

## Use from another CMake project

After installation, point CMake at the staged package:

```cmake
find_package(SimplePIDController CONFIG REQUIRED)
target_link_libraries(my_target PRIVATE SimplePIDController::SimplePIDController)
```

Configure the consumer with `-DCMAKE_PREFIX_PATH=/path/to/SPIDController/build/release/install`.

## Static analysis

```bash
cmake --preset debug-analysis
cmake --build --preset debug-static-analysis --parallel
```

On macOS, the project obtains the active Apple SDK path automatically for Homebrew LLVM tooling.

## VS Code

Install the recommended C++ and CMake Tools extensions. Use **Terminal → Run Task** for Debug/Release builds, test runs, installing, packaging, or static analysis. Use the **Debug Unit Tests** launch configuration to build and debug the GoogleTest executable.
