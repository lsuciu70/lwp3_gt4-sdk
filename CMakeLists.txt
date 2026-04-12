cmake_minimum_required(VERSION 3.22)
project(LWP3-GT4-SDK)

set(CMAKE_CXX_STANDARD 20)

# Find SimpleBLE (Ensure libsimpleble-dev is installed on Ubuntu)
find_package(simpleble REQUIRED)

include_directories(include)

# SDK Library
add_library(lwp3gt4 src/Lwp3Gt4.cpp)
target_link_libraries(lwp3gt4 simpleble::simpleble)

# Example Application
add_executable(hello_gt4 examples/hello_gt4.cpp)
target_link_libraries(hello_gt4 lwp3gt4)