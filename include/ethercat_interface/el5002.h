#ifndef __EL5002_H
#define __EL5002_H

#include "ethercat_interface/ethercat_includes.h"

class EL5002{
  private:
  ec_slavet *ec_slave;
  int slave_number;
public:
  EL5002(ec_slavet *slave, int slave_num);
  void get_inputs(uint32_t* channel_1, uint32_t* channel_2);

  void write_config(uint8_t graycode, uint8_t multiturn);
  void read_config(uint8_t* graycode, uint8_t* multiturn);
};


#endif
