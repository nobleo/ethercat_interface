#ifndef __EL2004_H
#define __EL2004_H

#include "ethercat_interface/ethercat_includes.h"

class EL2004{
	private:
	ec_slavet *ec_slave;
public:
	EL2004(ec_slavet *slave);
	void set_output(uint8_t output_nr, boolean do_enable);
	void toggle_output(uint8_t output_nr);
	boolean get_output(uint8_t output_nr);
};


#endif
