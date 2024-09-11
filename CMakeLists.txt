cmake_minimum_required(VERSION 3.0)
project(planning_pset1)

set(CMAKE_COLOR_DIAGNOSTICS ON)
set(CMAKE_CXX_STANDARD 14)

find_package(Boost REQUIRED)

include_directories(include)

# Add the main executable.
add_executable(run_test
        src/planner.cpp
        src/runtest.cpp)
# Have the executable depend on the header files.
target_include_directories(run_test PRIVATE ${CMAKE_SOURCE_DIR}/include ${Boost_INCLUDE_DIRS})

# Define the absolute path to relevant directories.
target_compile_definitions(run_test PRIVATE MAPS_DIR="${CMAKE_SOURCE_DIR}/maps")
target_compile_definitions(run_test PRIVATE OUTPUT_DIR="${CMAKE_SOURCE_DIR}/output")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
