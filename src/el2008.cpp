#include "ethercat_interface/el2008.h"
#include "ethercat_interface/ethercat_includes.h"
#include "ros/ros.h"

EL2008::EL2008(ec_slavet *slave, int slave_num){
  ec_slave = (slave+slave_num);
  slave_number = slave_num;
}

int EL2008::check_slave(){
  if(strcmp(ec_slave->name,"EL2008")){
    ROS_ERROR("slave %d is configured to be a EL2008, but found a %s",slave_number,ec_slave->name);
    return 1;
  }else{
    return 0;
  }
}

void EL2008::set_output(uint8_t channel, boolean do_enable){
  if(channel<8){
    if(do_enable){
      ec_slave->outputs[0] |= (1<<channel);
    }
    else{
      ec_slave->outputs[0] &= ~(1<<channel);
    }
  }else{
    ROS_ERROR("slave %d (EL2008) Requested channel %d, which is not valid",slave_number,channel);
  }
}

void EL2008::toggle_output(uint8_t channel){
  if(channel<8){
    ec_slave->outputs[0] ^= (1<<channel);
  }else{
    ROS_ERROR("slave %d (EL2008) Requested channel %d, which is not valid",slave_number,channel);
  }
}

boolean EL2008::get_output(uint8_t channel){
  if(channel<8){
    return (ec_slave->outputs[0] & (1<<channel))?TRUE:FALSE;
  }else{
    ROS_ERROR("slave %d (EL2008) Requested channel %d, which is not valid",slave_number,channel);
    return FALSE;
  }
}
