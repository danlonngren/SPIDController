# Simple PID controller
A simple PID controller suitable for bare-metal projects

## Build and Run instuctions
```bash
# Update gtest
git submodule update --init --recursive

# Clean build
rm -rf build

# Generate build
cmake -S . -B build -DBUILD_TESTS=ON

# Build project
cmake --build build

# Run googletests
GTEST_COLOR=1 ctest --test-dir build --output-on-failure --j 12

# Single Command
cmake --build build && GTEST_COLOR=1 ctest --test-dir build --output-on-failure --j 12
