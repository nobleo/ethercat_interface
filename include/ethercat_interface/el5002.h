#ifndef __EL5002_H
#define __EL5002_H

#include "ethercat_interface/ethercat_includes.h"

class EL5002{
  private:
  ec_slavet *ec_slave;
  int slave_number;
public:
  EL5002(ec_slavet *slave, int slave_num);
  uint32_t get_input(uint8_t channel);

  void write_config(uint8_t channel, uint8_t graycode, uint8_t multiturn, uint8_t framesize, uint8_t datalength);
  void read_config(uint8_t channel, uint8_t* graycode, uint8_t* multiturn, uint8_t* framesize, uint8_t* datalength);
};

#endif
