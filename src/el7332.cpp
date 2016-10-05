#include "ethercat_demo/el7332.h"

EL7332::EL7332(ec_slavet *slave){
	ec_slave = slave;
}

void EL7332::enable(uint8_t axis, boolean do_enable){
	if(do_enable){
		ec_slave->outputs[0+axis*4] |= 0x01;
	}
	else{
		ec_slave->outputs[0+axis*4] &= ~(0x01);
	}
}

boolean EL7332::is_enabled(uint8_t axis){
	return (ec_slave->outputs[0+axis*4] & 0x01)?TRUE:FALSE;
}

void EL7332::set_velocity(uint8_t axis, int16_t velocity){
	int16_t *setpoint = (int16_t *)&(ec_slave->outputs[2+4*axis]);
	*setpoint = velocity;
}

int16_t EL7332::get_velocity(uint8_t axis){
	int16_t *setpoint = (int16_t *)&(ec_slave->outputs[2+4*axis]);
	return *setpoint;
}

