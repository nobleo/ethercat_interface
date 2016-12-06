cmake_minimum_required(VERSION 2.8.3)
project(ethercat_interface)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  ethercat_soem
  hardware_interface
  controller_manager)

## Declare ROS messages and services
add_message_files(FILES velocity_cmd.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(velocity_to_ethercat src/ethercat_interface.cpp src/el2008.cpp src/el7332.cpp)
target_link_libraries(velocity_to_ethercat ${catkin_LIBRARIES})

add_executable(geometry_twist_to_velocity src/geometry_twist_to_velocity.cpp)

add_dependencies(geometry_twist_to_velocity ethercat_interface_generate_messages_cpp)

target_link_libraries(geometry_twist_to_velocity ${catkin_LIBRARIES})


