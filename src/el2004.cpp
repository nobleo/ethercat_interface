#include "ethercat_interface/el2004.h"

EL2004::EL2004(ec_slavet *slave){
	ec_slave = slave;
}

void EL2004::set_output(uint8_t output_nr, boolean do_enable){
	if(do_enable){
		ec_slave->outputs[0] |= (1<<output_nr);
	}
	else{
		ec_slave->outputs[0] &= ~(1<<output_nr);
	}
}

void EL2004::toggle_output(uint8_t output_nr){
	ec_slave->outputs[0] ^= (1<<output_nr);
}

boolean EL2004::get_output(uint8_t output_nr){
	return (ec_slave->outputs[0] & (1<<output_nr))?TRUE:FALSE;
}
