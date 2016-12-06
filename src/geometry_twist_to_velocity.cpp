#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <unistd.h>

#include "ros/ros.h"
#include "ethercat_interface/velocity_cmd.h"

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

class MyRobot : public hardware_interface::RobotHW
{
public:
  MyRobot()
  {
    ros::NodeHandle n;

    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_a("A", &pos[0], &vel[0],
                                                        &eff[0]);
    jnt_state_interface.registerHandle(state_handle_a);

    hardware_interface::JointStateHandle state_handle_b("B", &pos[1], &vel[1],
                                                        &eff[1]);
    jnt_state_interface.registerHandle(state_handle_b);

    registerInterface(&jnt_state_interface);

    // connect and register the joint position interface
    hardware_interface::JointHandle pos_handle_a(
        jnt_state_interface.getHandle("A"), &cmd[0]);
    jnt_pos_interface.registerHandle(pos_handle_a);

    hardware_interface::JointHandle pos_handle_b(
        jnt_state_interface.getHandle("B"), &cmd[1]);
    jnt_pos_interface.registerHandle(pos_handle_b);

    registerInterface(&jnt_pos_interface);

    pub = n.advertise<ethercat_interface::velocity_cmd>("velocity_in", 1);
  }

  void write(void);

private:
  ros::Publisher pub;
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  double cmd[2];
  double pos[2];
  double vel[2];
  double eff[2];
  static const int MOTORGAIN = 26250; // m/s / DAC
};

void MyRobot::write(void)
{
  ethercat_interface::velocity_cmd velocity_msg;

  float v_right = cmd[0];
  float v_left = cmd[1];

  velocity_msg.velocity_left = MOTORGAIN * v_left;
  velocity_msg.velocity_right = MOTORGAIN * v_right;

  pub.publish(velocity_msg);
}

#define PI 3.14159
#define VMAX_LIN 0.5        // m/s
#define VMAX_ROT (0.5 * PI) // rad/s
#define WHEELBASE 0.215     // m

static inline float clamp(float value, float min, float max)
{
  return (value > min) ? ((value < max) ? value : max) : min;
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "hardware_interface");

  MyRobot robot;
  controller_manager::ControllerManager cm(&robot);

  ros::Rate r(10);

  while (ros::ok())
  {
    cm.update(ros::Time::now(), r.cycleTime());
    robot.write();
    r.sleep();
  }

  return 0;
}
