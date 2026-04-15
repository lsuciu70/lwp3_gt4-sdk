cmake_minimum_required(VERSION 3.22)
project(LWP3-GT4-SDK)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find SimpleBLE (Ensure libsimpleble-dev is installed on Ubuntu)
find_package(simpleble REQUIRED)

# Required for C++20 std::jthread and std::mutex on Linux
find_package(Threads REQUIRED)

include_directories(include)

# SDK Library
add_library(lwp3gt4 src/Lwp3Gt4.cpp)
target_link_libraries(lwp3gt4 simpleble::simpleble Threads::Threads)

# Example Application
add_executable(hello_gt4 examples/hello_gt4.cpp)
target_link_libraries(hello_gt4 lwp3gt4)

# Minimal BLE test harness (drop-in)
add_executable(ble_latency_test tests/ble_latency_test.cpp)
target_link_libraries(ble_latency_test lwp3gt4)

# Example requesting new sine-wave angles
add_executable(async_test examples/async_control_demo.cpp)
target_link_libraries(async_test lwp3gt4)
