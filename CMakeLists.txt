cmake_minimum_required(VERSION 3.22)
project(LWP3-GT4-SDK)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(simpleble REQUIRED)
find_package(Threads REQUIRED)

# SDK Library
add_library(lwp3gt4 src/Lwp3Gt4.cpp)

# This allows examples to use #include "Lwp3Gt4.hpp" without relative paths
target_include_directories(lwp3gt4 PUBLIC include)

target_link_libraries(lwp3gt4 simpleble::simpleble Threads::Threads)

# Examples & Tests
add_executable(hello_gt4 examples/hello_gt4.cpp)
target_link_libraries(hello_gt4 lwp3gt4)

add_executable(ble_latency_test tests/ble_latency_test.cpp)
target_link_libraries(ble_latency_test lwp3gt4)

add_executable(async_test examples/async_test.cpp)
target_link_libraries(async_test lwp3gt4)