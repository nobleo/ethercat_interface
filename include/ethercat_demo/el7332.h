#ifndef __EL7332_H
#define __EL7332_H

#include "ethercat_soem/ethercat.h"

class EL7332{
	private:
	ec_slavet *ec_slave;
public:
	EL7332(ec_slavet *slave);
	void enable(uint8_t axis, boolean do_enable);
	boolean is_enabled(uint8_t axis);	
	void set_velocity(uint8_t, int16_t velocity);
	int16_t	get_velocity(uint8_t axis);

};


#endif

