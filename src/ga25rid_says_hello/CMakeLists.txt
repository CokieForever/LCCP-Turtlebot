

# Declare the version of the CMake API for forward-compatibility
cmake_minimum_required(VERSION 2.8)

# Declare the name of the CMake Project
project(ga25rid_says_hello)

# Find Catkin
find_package(catkin REQUIRED)
# Declare this project as a catkin package
catkin_package()

# Find and get all the information about the roscpp package
find_package(roscpp REQUIRED)

# Add the headers from roscpp
include_directories(${roscpp_INCLUDE_DIRS})
# Define an execuable target called hello_world_node
add_executable(hello_world_node hello_world_node.cpp)
# Link the hello_world_node target against the libraries used by roscpp
target_link_libraries(hello_world_node ${roscpp_LIBRARIES})
