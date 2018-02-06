#ifndef __EL2502_H
#define __EL2502_H

#include "ethercat_interface/ethercat_includes.h"

#define pwm_res 65536

class EL2502{
	private:
	ec_slavet *ec_slave;
	int slave_number;
	uint8_t pres_mode;
	uint16_t period_us;
public:
	EL2502(ec_slavet *slave, int slave_num);
	void set_output(uint8_t output_nr, double dutycycle);
	double get_output(uint8_t output_nr);

	void write_config(uint8_t pres_mode, uint16_t period_us);
	void read_config(uint8_t* pres_mode, uint16_t* period_us);
};


#endif
