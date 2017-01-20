#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class EthercatHardware : public hardware_interface::RobotHW
{
public:
  EthercatHardware();

  void readJoints();

  void writeJoints();

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;

  double cmd[2];
  double pos[2];
  double vel[2];
  double eff[2];
};
