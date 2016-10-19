cmake_minimum_required(VERSION 2.8.3)
project(ethercat_demo)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation ethercat_soem)

## Declare ROS messages and services
add_message_files(FILES velocity_cmd.msg distance.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(listener src/listener.cpp src/el2008.cpp src/el7332.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})

add_executable(nobleo_bot_V0 src/nobleo_bot_V0.cpp)
target_link_libraries(nobleo_bot_V0 ${catkin_LIBRARIES})
