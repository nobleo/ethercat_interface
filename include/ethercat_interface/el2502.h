#ifndef __EL2502_H
#define __EL2502_H

#include "ethercat_interface/ethercat_includes.h"

#define pwm_res 65536

#ifndef dabs
#define dabs(a)     ((a)>= 0 ? (a):-(a))
#endif
#ifndef dmin
#define dmin(a,b)   ((a)<=(b) ? (a):(b))
#endif
#ifndef dmax
#define dmax(a,b)   ((a)>=(b) ? (a):(b))
#endif

class EL2502{
	private:
	ec_slavet *ec_slave;
	int slave_number;
	uint8_t pres_mode;
	uint16_t period_us;
public:
	EL2502(ec_slavet *slave, int slave_num);
	int check_slave();
	void set_output(uint8_t channel, float32 dutycycle);
	float32 get_output(uint8_t channel);

	void write_config(uint8_t channel, uint8_t pres_mode, uint16_t period_us);
	void read_config(uint8_t channel, uint8_t* pres_mode, uint16_t* period_us);
};


#endif
