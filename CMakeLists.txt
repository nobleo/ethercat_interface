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

add_executable(ethercat_interface src/ethercat_interface.cpp src/el2008.cpp src/el7332.cpp)
target_link_libraries(ethercat_interface ${catkin_LIBRARIES})

add_executable(ethercat_interface_analog src/ethercat_interface_analog.cpp src/el2008.cpp src/el4002.cpp)
target_link_libraries(ethercat_interface_analog ${catkin_LIBRARIES})

add_executable(ethercat_interface_fedra src/ethercat_interface_fedra.cpp src/el2502.cpp src/el2004.cpp src/el2008.cpp src/el5002.cpp)
target_link_libraries(ethercat_interface_fedra ${catkin_LIBRARIES})
