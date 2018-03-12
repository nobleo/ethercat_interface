#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class EthercatHardware : public hardware_interface::RobotHW
{
public:
  EthercatHardware();

  void readPivots();

  void readJoints();

  void writeJoints();

private:
};
