#include "ethercat_demo/el7332.h"
#include "ethercat_demo/ethercat_includes.h"

EL7332::EL7332(ec_slavet *slave, int slave_num){
	ec_slave = slave;
	slave_number = slave_num;

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

uint16_t EL7332::max_voltage()
{
	uint16_t voltage = 0;
        int size_of_max_voltage = sizeof(voltage);
	ec_SDOread(slave_number, 0x8010, 0x03, FALSE, &size_of_max_voltage, &voltage,EC_TIMEOUTRXM);
	return voltage;
}
uint16_t EL7332::max_voltage(uint16_t voltage)
{
	int size_of_max_voltage = sizeof(voltage);
	ec_SDOwrite(slave_number, 0x8010, 0x03, FALSE, size_of_max_voltage, &voltage,EC_TIMEOUTRXM);
	return max_voltage();
}
