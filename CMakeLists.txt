cmake_minimum_required(VERSION 3.0)
project(planning_pset1)

set(CMAKE_COLOR_DIAGNOSTICS ON)
set(CMAKE_CXX_STANDARD 14)

find_package(Boost REQUIRED)

include_directories(include)

# Add library of classes that I might potentially re-use
add_library(astar_lib
        src/GoalTrajectoryManager.cpp
        src/GraphManager.cpp
        src/LookupPriorityQueue.cpp
)

# Feed my library my headers
target_include_directories(astar_lib PRIVATE ${CMAKE_SOURCE_DIR}/include ${Boost_INCLUDE_DIRS})

# Add the main executable.
add_executable(run_test
        src/planner.cpp
        src/runtest.cpp
)
# Have the executable depend on the header files.
target_include_directories(run_test PRIVATE ${CMAKE_SOURCE_DIR}/include ${Boost_INCLUDE_DIRS})

# Link the library to the executable
target_link_libraries(run_test PRIVATE astar_lib)

# Define the absolute path to relevant directories.
target_compile_definitions(run_test PRIVATE MAPS_DIR="${CMAKE_SOURCE_DIR}/maps")
target_compile_definitions(run_test PRIVATE OUTPUT_DIR="${CMAKE_SOURCE_DIR}/output")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
