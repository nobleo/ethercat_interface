#ifndef __EL2008_H
#define __EL2008_H

#include "ethercat_interface/ethercat_includes.h"

class EL2008{
  private:
  ec_slavet *ec_slave;
  int slave_number;
public:
  EL2008(ec_slavet *slave, int slave_num);
  int check_slave();
  void set_output(uint8_t output_nr, boolean do_enable);
  void toggle_output(uint8_t output_nr);
  boolean get_output(uint8_t output_nr);
};


#endif
