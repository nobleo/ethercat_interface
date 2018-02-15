#include "ethercat_interface/el2004.h"
#include "ethercat_interface/ethercat_includes.h"
#include "ros/ros.h"

EL2004::EL2004(ec_slavet *slave, int slave_num){
  ec_slave = (slave+slave_num);
	slave_number = slave_num;
}

int EL2004::check_slave(){
  if(strcmp(ec_slave->name,"EL2004")){
    ROS_ERROR("slave %d is configured to be a EL2004, but found a %s",slave_number,ec_slave->name);
    return 1;
  }else{
    return 0;
  }
}

void EL2004::set_output(uint8_t channel, boolean do_enable){
  if(channel<4){
    if(do_enable){
      ec_slave->outputs[0] |= (1<<channel);
    }
    else{
      ec_slave->outputs[0] &= ~(1<<channel);
    }
  }else{
    ROS_ERROR("slave %d (EL2004) Requested channel %d, which is not valid",slave_number,channel);
  }
}

void EL2004::toggle_output(uint8_t channel){
  if(channel<4){
    ec_slave->outputs[0] ^= (1<<channel);
  }else{
    ROS_ERROR("slave %d (EL2004) Requested channel %d, which is not valid",slave_number,channel);
  }
}

boolean EL2004::get_output(uint8_t channel){
  if(channel<4){
    return (ec_slave->outputs[0] & (1<<channel))?TRUE:FALSE;
  }else{
    ROS_ERROR("slave %d (EL2004) Requested channel %d, which is not valid",slave_number,channel);
    return FALSE;
  }
}
