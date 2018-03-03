#ifndef ETHERCAT_HARDWARE_H
#define ETHERCAT_HARDWARE_H

class EthercatHardware
{
public:
  EthercatHardware();

  void readPivots();

  void readJoints();

  void writeJoints();

private:
};

#endif // ETHERCAT_HARDWARE_H
