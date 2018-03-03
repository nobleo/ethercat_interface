#include "ros/ros.h"
#include "ethercat_interface/ethercat_interface.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ethercat_interface_node");

  ros::NodeHandle nh("~");

  std::string ethercat_interface;
  nh.param<std::string>("ethercat_interface", ethercat_interface, "eth0");

  ROS_INFO("Constructing EtherCAT Interface");
  EthercatInterface interface;
  ROS_INFO("Constructed EtherCAT Interface");

  ROS_INFO("Shutting down");
  return 0;
}
