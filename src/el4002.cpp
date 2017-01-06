#include "ethercat_interface/el4002.h"
#include <iostream>

EL4002::EL4002(ec_slavet *slave){
	ec_slave = slave;
}

int16_t EL4002::volt2dac(double value){
  //convert voltage setpoint to 16 bits dac value
  int16_t dac_setpoint = (int16_t) value*(1<<15)/10;
  return dac_setpoint;
}

void EL4002::set_output(uint8_t output_nr, double value){
  //2 bytes per output
  int16_t *setpoint = (int16_t *)&(ec_slave->outputs[2*output_nr]);
  *setpoint = (int16_t) volt2dac(value);
  std::cout << "volt2dac = " << volt2dac(value) << std::endl;

  std::cout << (int16_t) ec_slave->outputs[0] << " - "  << (int16_t) ec_slave->outputs[1] << " - "  << (int16_t) ec_slave->outputs[2] << " - "  << (int16_t) ec_slave->outputs[3] << std::endl;
}

double EL4002::get_output(uint8_t output_nr){
	return (ec_slave->outputs[0] & (1<<output_nr))?TRUE:FALSE;
}

