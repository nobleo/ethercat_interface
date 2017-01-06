cmake_minimum_required(VERSION 2.8.3)
project(ethercat_interface)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  ethercat_soem
  hardware_interface
  controller_manager)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ethercat_interface src/ethercat_interface.cpp src/el2008.cpp src/el4002.cpp src/el7332.cpp)
target_link_libraries(ethercat_interface ${catkin_LIBRARIES})
