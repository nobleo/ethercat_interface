#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <unistd.h>

#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include "ethercat_demo/velocity_cmd.h"

ros::Subscriber sub;
ros::Publisher pub;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  	ROS_INFO("I heard: [%f, %f]", msg->axes[1], msg->axes[4]);

    ethercat_demo::velocity_cmd velocity_msg;
	float vr = msg->axes[0];
	float vf = msg->axes[1];

	// two wheel, two joystick
	//velocity_msg.velocity_left = msg->axes[1];
	//velocity_msg.velocity_right = msg->axes[4];

	// two wheel, one joystick
	velocity_msg.velocity_left = vf-vr;
	velocity_msg.velocity_right = vf+vr;


    pub.publish(velocity_msg);
}



int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "joy_teleop");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  sub = n.subscribe("joy", 1000, joyCallback);
  pub = n.advertise<ethercat_demo::velocity_cmd>("velocity_in", 1000);
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
