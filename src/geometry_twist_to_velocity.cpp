#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <unistd.h>

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "ethercat_demo/velocity_cmd.h"

ros::Subscriber sub;
ros::Publisher pub;

#define PI 3.14159
#define VMAX_LIN 0.5 // m/s
#define VMAX_ROT (0.5*PI) // rad/s
#define WHEELBASE 0.215 //m
#define MOTORGAIN 26250 //m/s / DAC

static inline float clamp(float value, float min, float max){
	return (value>min)?((value<max)?value:max):min;
}

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void geometryTwistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    ethercat_demo::velocity_cmd velocity_msg;
	float v_right, v_left, v_linear, v_angular;

  	ROS_INFO("I heard: [%f, %f]", msg->linear.x, msg->angular.z);

	v_linear = clamp(msg->linear.x, -VMAX_LIN, VMAX_LIN);
	v_angular = clamp(msg->angular.z, -VMAX_ROT, VMAX_ROT);

	v_right = v_linear + WHEELBASE*v_angular;
	v_left  = v_linear - WHEELBASE*v_angular;

	// two wheel, two joystick
	//velocity_msg.velocity_left = msg->axes[1];
	//velocity_msg.velocity_right = msg->axes[4];

	// two wheel, one joystick
	velocity_msg.velocity_left = MOTORGAIN*v_left;
	velocity_msg.velocity_right = MOTORGAIN*v_right;


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
  ros::init(argc, argv, "nobleo_bot_V0");

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
  sub = n.subscribe("cmd_vel", 1000, geometryTwistCallback);
  pub = n.advertise<ethercat_demo::velocity_cmd>("velocity_in", 1000);
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
