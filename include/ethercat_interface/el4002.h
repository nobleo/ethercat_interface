#ifndef __EL4002_H
#define __EL4002_H

#include "ethercat_interface/ethercat_includes.h"

class EL4002{
	private:
	ec_slavet *ec_slave;
public:
    EL4002(ec_slavet *slave);
    int16_t volt2dac(double value);
    double dac2volt(int16_t dac_value);
    void set_output(uint8_t output_nr, double value);
    double get_output(uint8_t output_nr);
};


#endif

