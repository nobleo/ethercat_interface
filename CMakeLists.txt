cmake_minimum_required(VERSION 2.8.3)
project(ethercat_interface)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  ethercat_soem)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS)

include_directories(include ${catkin_INCLUDE_DIRS})

# 'Original'
add_executable(ethercat_hardware src/ethercat_hardware.cpp src/el2008.cpp src/el2502.cpp src/el5002.cpp)
target_link_libraries(ethercat_hardware ${catkin_LIBRARIES})

# New and improved
add_library(ethercat_interface src/ethercat_interface.cpp)
target_link_libraries(ethercat_interface ${catkin_LIBRARIES})

add_executable(ethercat_interface_node src/ethercat_interface_node.cpp)
target_link_libraries(ethercat_interface_node ethercat_interface ${catkin_LIBRARIES})
