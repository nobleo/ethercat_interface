#include "ethercat_interface/el2502.h"
#include "ethercat_interface/ethercat_includes.h"
#include <iostream>

EL2502::EL2502(ec_slavet *slave, int slave_num){
	ec_slave = slave;
	slave_number = slave_num;
	// Initialize settings in constructor with default Beckhoff values
	pres_mode = 0;
  period_us = 4000;
}

void EL2502::set_output(uint8_t output_nr, double dutycycle){
  uint16_t *dc_value = (uint16_t *)&(ec_slave->outputs[2*output_nr]);
  *dc_value =  (uint16_t) (dutycycle*pwm_res);
}

double EL2502::get_output(uint8_t output_nr){
  uint16_t *dc_value = (uint16_t *)&(ec_slave->outputs[2*output_nr]);
  return ((double)*dc_value)/((double)pwm_res);
}

void EL2502::write_config(uint8_t pres, uint16_t per)
{
  pres_mode = pres;
  period_us = per;
  /* Set presentation mode PWM channel 1 */
  uint8_t write_size = sizeof(pres_mode);
  ec_SDOwrite(slave_number,0x8000,0x02,FALSE,write_size,&pres_mode,EC_TIMEOUTRXM);

  /* Set PWM period channel 1 */
  write_size = sizeof(period_us);
  ec_SDOwrite(slave_number,0x8000,0x15,FALSE,write_size,&period_us,EC_TIMEOUTRXM);

  /* Set presentation mode PWM channel 2 */
  write_size = sizeof(pres_mode);
  ec_SDOwrite(slave_number,0x8010,0x02,FALSE,write_size,&pres_mode,EC_TIMEOUTRXM);

  /* Set PWM period channel 2 */
  write_size = sizeof(period_us);
  ec_SDOwrite(slave_number,0x8010,0x15,FALSE,write_size,&period_us,EC_TIMEOUTRXM);
}

void EL2502::read_config(uint8_t* pres, uint16_t* per)
{
  /* Get presentation mode PWM channel 1 */
  int read_size = sizeof(pres_mode);
  ec_SDOread(slave_number,0x8000,0x02,FALSE,&read_size,pres,EC_TIMEOUTRXM);

  /* Get PWM period channel 1 */
  read_size = sizeof(period_us);
  ec_SDOread(slave_number,0x8000,0x15,FALSE,&read_size,per,EC_TIMEOUTRXM);
//  std::cout << "pwm1 has presentation mode: " << pres_mode << " and period: " << period_us << " us\n"

  /* Get presentation mode PWM channel 2 */
  read_size = sizeof(pres_mode);
  ec_SDOread(slave_number,0x8010,0x02,FALSE,&read_size,pres,EC_TIMEOUTRXM);

  /* Get PWM period channel 2 */
  read_size = sizeof(period_us);
  ec_SDOread(slave_number,0x8010,0x15,FALSE,&read_size,per,EC_TIMEOUTRXM);
//  std::cout << "pwm2 has presentation mode: " << pres_mode << " and period: " << period_us << " us\n"
}
