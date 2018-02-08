#include "ethercat_interface/el5002.h"
#include "ethercat_interface/ethercat_includes.h"
#include <iostream>

EL5002::EL5002(ec_slavet *slave, int slave_num){
  ec_slave = slave;
  slave_number = slave_num;
}

void EL5002::get_inputs(uint32_t* channel_1, uint32_t* channel_2){
  *channel_1 = (uint32_t)(ec_slave->outputs[0]);
  *channel_2 = (uint32_t)(ec_slave->outputs[4]);
}

void EL5002::write_config(uint8_t graycode, uint8_t multiturn)
{
  /* Set presentation mode PWM channel 1 */
  uint8_t write_size = sizeof(graycode);
  ec_SDOwrite(slave_number,0x8000,0x06,FALSE,write_size,&graycode,EC_TIMEOUTRXM);

  /* Set PWM period channel 1 */
  write_size = sizeof(multiturn);
  ec_SDOwrite(slave_number,0x8000,0x0F,FALSE,write_size,&multiturn,EC_TIMEOUTRXM);
}

void EL5002::read_config(uint8_t* graycode, uint8_t* multiturn)
{
  /* Get presentation mode PWM channel 1 */
  int write_size = sizeof(graycode);
  ec_SDOread(slave_number,0x8000,0x06,FALSE,&write_size,graycode,EC_TIMEOUTRXM);

  /* Get PWM period channel 1 */
  write_size = sizeof(multiturn);
  ec_SDOread(slave_number,0x8000,0x0F,FALSE,&write_size,multiturn,EC_TIMEOUTRXM);
}
