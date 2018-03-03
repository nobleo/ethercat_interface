#include "ros/ros.h"
#include "ethercat_interface/ethercat_interface.h"
#include "ethercat_interface/exceptions.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ethercat_interface_node");

    ros::NodeHandle nh("~");

    std::string ifname;
    nh.param<std::string>("ifname", ifname, "eth0");

    ROS_INFO("Constructing EtherCAT Interface");
    EthercatInterface interface;

    ROS_INFO("Initializing");
    try
    {
        if (!interface.initialize(ifname))
        {
            ROS_ERROR("Something went terribly wrong...");
            exit(1);
        }
    }
    catch (SocketError)
    {
        ROS_ERROR("No socket connection on %s. Try excecuting the following "
                  "command: sudo setcap cap_net_raw+ep $(readlink $(catkin_find "
                  "ethercat_interface ethercat_interface_node))\n",
                  ifname.c_str());
        exit(1);
    }

    ROS_INFO("Starting loop");
    ros::Rate rate(1000.0);
    while (ros::ok())
    {
        interface.receiveAll();
        interface.sendAll();
        rate.sleep();
    }

    ROS_INFO("Shutting down");
    return 0;
}
