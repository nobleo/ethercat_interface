#include "ethercat_interface/el2008.h"

EL2008::EL2008(ec_slavet *slave){
	ec_slave = slave;
}

void EL2008::set_output(uint8_t output_nr, boolean do_enable){
	if(do_enable){
		ec_slave->outputs[0] |= (1<<output_nr);
	}
	else{
		ec_slave->outputs[0] &= ~(1<<output_nr);
	}
}

void EL2008::toggle_output(uint8_t output_nr){
	ec_slave->outputs[0] ^= (1<<output_nr);
}

boolean EL2008::get_output(uint8_t output_nr){
	return (ec_slave->outputs[0] & (1<<output_nr))?TRUE:FALSE;
}

