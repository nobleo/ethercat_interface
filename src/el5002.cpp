#include "ethercat_interface/el5002.h"
#include "ethercat_interface/ethercat_includes.h"
#include <iostream>
#include <stdio.h>

EL5002::EL5002(ec_slavet *slave, int slave_num){
  ec_slave = slave;
  slave_number = slave_num;
}

uint32_t EL5002::get_input(uint8_t channel){
  /* Input-data is: Status_ch1 (2 bytes) Counter_ch1 (4 bytes) Status_ch2 (2 bytes) Counter_ch2 (4 bytes) */
  return *(uint32_t*)(&ec_slave->inputs[2+channel*6]);
}

void EL5002::write_config(uint8_t channel, uint8_t graycode, uint8_t multiturn, uint8_t framesize, uint8_t datalength)
{
  if (channel==0){
    /* Channel 1 */
    /* Set SSI coding, Graycode or Binary */
    uint8_t write_size = sizeof(graycode);
    ec_SDOwrite(slave_number,0x8000,0x06,FALSE,write_size,&graycode,EC_TIMEOUTRXM);

    /* Set SSI-frame type, Multiturn or Single turn */
    write_size = sizeof(multiturn);
    ec_SDOwrite(slave_number,0x8000,0x0F,FALSE,write_size,&multiturn,EC_TIMEOUTRXM);

    /* Set SSI-frame size in bits */
    write_size = sizeof(framesize);
    ec_SDOwrite(slave_number,0x8000,0x11,FALSE,write_size,&framesize,EC_TIMEOUTRXM);

    /* Set SSI-data length in bits */
    write_size = sizeof(datalength);
    ec_SDOwrite(slave_number,0x8000,0x12,FALSE,write_size,&datalength,EC_TIMEOUTRXM);
  }else{
    /* Channel 2 */
    /* Set SSI coding, Graycode or Binary */
    uint8_t write_size = sizeof(graycode);
    ec_SDOwrite(slave_number,0x8010,0x06,FALSE,write_size,&graycode,EC_TIMEOUTRXM);

    /* Set SSI-frame type, Multiturn or Single turn */
    write_size = sizeof(multiturn);
    ec_SDOwrite(slave_number,0x8010,0x0F,FALSE,write_size,&multiturn,EC_TIMEOUTRXM);

    /* Set SSI-frame size in bits */
    write_size = sizeof(framesize);
    ec_SDOwrite(slave_number,0x8010,0x11,FALSE,write_size,&framesize,EC_TIMEOUTRXM);

    /* Set SSI-data length in bits */
    write_size = sizeof(datalength);
    ec_SDOwrite(slave_number,0x8010,0x12,FALSE,write_size,&datalength,EC_TIMEOUTRXM);
  }
}

void EL5002::read_config(uint8_t channel, uint8_t* graycode, uint8_t* multiturn, uint8_t* framesize, uint8_t* datalength)
{
  if (channel==0){
    /* Channel 1 */
    /* Get SSI coding, Graycode or Binary */
    int read_size = sizeof(graycode);
    ec_SDOread(slave_number,0x8000,0x06,FALSE,&read_size,graycode,EC_TIMEOUTRXM);

    /* Get SSI-frame type, Multiturn or Single turn */
    read_size = sizeof(multiturn);
    ec_SDOread(slave_number,0x8000,0x0F,FALSE,&read_size,multiturn,EC_TIMEOUTRXM);

    /* Set SSI-frame size in bits */
    read_size = sizeof(framesize);
    ec_SDOread(slave_number,0x8000,0x11,FALSE,&read_size,framesize,EC_TIMEOUTRXM);

    /* Set SSI-data length in bits */
    read_size = sizeof(datalength);
    ec_SDOread(slave_number,0x8000,0x12,FALSE,&read_size,datalength,EC_TIMEOUTRXM);
  }else{
    /* Channel 1 */
    /* Get SSI coding, Graycode or Binary */
    int read_size = sizeof(graycode);
    ec_SDOread(slave_number,0x8010,0x06,FALSE,&read_size,graycode,EC_TIMEOUTRXM);

    /* Get SSI-frame type, Multiturn or Single turn */
    read_size = sizeof(multiturn);
    ec_SDOread(slave_number,0x8010,0x0F,FALSE,&read_size,multiturn,EC_TIMEOUTRXM);

    /* Set SSI-frame size in bits */
    read_size = sizeof(framesize);
    ec_SDOread(slave_number,0x8010,0x11,FALSE,&read_size,framesize,EC_TIMEOUTRXM);

    /* Set SSI-data length in bits */
    read_size = sizeof(datalength);
    ec_SDOread(slave_number,0x8010,0x12,FALSE,&read_size,datalength,EC_TIMEOUTRXM);
  }
}
