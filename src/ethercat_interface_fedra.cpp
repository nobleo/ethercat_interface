#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <unistd.h>

#include "ros/ros.h"

#include "ethercat_interface/ethercat_includes.h"
#include "ethercat_interface/ethercat_interface_fedra.h"

#include "ethercat_interface/el2004.h"
#include "ethercat_interface/el2008.h"
#include "ethercat_interface/el2502.h"
#include "ethercat_interface/el5002.h"

#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#define EC_TIMEOUTMON 500
#define PDO_PERIOD 5000
#define STATECHECK_PERIOD 100000

char IOmap[4096];
pthread_t thread_statecheck;
pthread_t thread_pdo;

volatile int expectedWKC;
volatile int wkc;

boolean pdo_transfer_active = FALSE;

/* Define EtherCAT Stack */
#define PWM_PRES_MODE 1
#define PWM_PERIOD_US 10000 // 10.000 us a.k.a. 100Hz
EL2502 pwmdriver_pivot1(ec_slave,2);
EL2004 digitalOut_pivot1(ec_slave,3);
#define GRAYCODE 1
#define MULTITURN 2 // 0: Multiturn (25 bits), 1: Single-turn (13 bits), 2: Variable (1-32 bits, configured via FRAMESIZE/DATALENGTH)
#define FRAMESIZE 25
#define DATALENGTH 25
#define PIVOT_ENC_RES 8192
EL5002 encoder_pivot1(ec_slave,4);
EL2502 pwmdriver_pivot2(ec_slave,6);
EL2004 digitalOut_pivot2(ec_slave,7);
//EL5002 encoder_pivot2(ec_slave,8);
EL2008 digitalOut_laminator(ec_slave,8);

/* Define ROS-topic publishers */
ros::Publisher encoder_pivot1_pub;
ros::Publisher encoder_pivot2_pub;

boolean check_ethercat_slaves(){
  int slaves_wrong = 0;
  slaves_wrong = pwmdriver_pivot1.check_slave() +
      digitalOut_pivot1.check_slave() +
      encoder_pivot1.check_slave() +
      pwmdriver_pivot2.check_slave() +
      digitalOut_pivot2.check_slave() +
      digitalOut_laminator.check_slave();
  if(slaves_wrong>0){
    ROS_ERROR("%d slaves are NOT connected in the expected order",slaves_wrong);
    return FALSE;
  }else{
    return TRUE;
  }
}

boolean setup_ethercat(char* ifname)
{
  int i, j, chk;

  /* initialise SOEM, bind socket to ifname */
  if (ec_init(ifname))
  {
    ROS_INFO("ec_init on %s succeeded.", ifname);
    /* find and auto-config slaves */

    if (ec_config_init(FALSE) > 0)
    {
      ROS_INFO("%d slaves found and configured.", ec_slavecount);

      ec_config_map(&IOmap);

      ec_configdc();

      pwmdriver_pivot1.write_config(0,PWM_PRES_MODE, PWM_PERIOD_US);
      pwmdriver_pivot1.write_config(1,PWM_PRES_MODE, PWM_PERIOD_US);
      uint8_t pres; uint16_t per;
      pwmdriver_pivot1.read_config(0, &pres, &per);
      ROS_INFO("PWM Channel 0, Written values %d, %d, Received values %d, %d", PWM_PRES_MODE, PWM_PERIOD_US, pres, per);
      pwmdriver_pivot1.read_config(1, &pres, &per);
      ROS_INFO("PWM Channel 1, Written values %d, %d, Received values %d, %d", PWM_PRES_MODE, PWM_PERIOD_US, pres, per);

      pwmdriver_pivot2.write_config(0,PWM_PRES_MODE, PWM_PERIOD_US);
      pwmdriver_pivot2.write_config(1,PWM_PRES_MODE, PWM_PERIOD_US);
      pwmdriver_pivot2.read_config(0, &pres, &per);
      ROS_INFO("PWM Channel 0, Written values %d, %d, Received values %d, %d", PWM_PRES_MODE, PWM_PERIOD_US, pres, per);
      pwmdriver_pivot2.read_config(1, &pres, &per);
      ROS_INFO("PWM Channel 1, Written values %d, %d, Received values %d, %d", PWM_PRES_MODE, PWM_PERIOD_US, pres, per);

      encoder_pivot1.write_config(0,GRAYCODE,MULTITURN,FRAMESIZE,DATALENGTH);
      encoder_pivot1.write_config(1,GRAYCODE,MULTITURN,FRAMESIZE,DATALENGTH);
      uint8_t graycode; uint8_t multiturn; uint8_t framesize; uint8_t datalength;
      encoder_pivot1.read_config(0, &graycode, &multiturn, &framesize, &datalength);
      ROS_INFO("SSI Channel 0, Written values %d, %d, %d, %d, Received values %d, %d, %d, %d", GRAYCODE, MULTITURN, FRAMESIZE, DATALENGTH,graycode, multiturn, framesize, datalength);
      encoder_pivot1.read_config(1, &graycode, &multiturn, &framesize, &datalength);
      ROS_INFO("SSI Channel 1, Written values %d, %d, %d, %d, Received values %d, %d, %d, %d", GRAYCODE, MULTITURN, FRAMESIZE, DATALENGTH,graycode, multiturn, framesize, datalength);

//      encoder_pivot2.write_config(0,GRAYCODE,MULTITURN,FRAMESIZE,DATALENGTH);
//      encoder_pivot2.write_config(1,GRAYCODE,MULTITURN,FRAMESIZE,DATALENGTH);
//      uint8_t graycode; uint8_t multiturn; uint8_t framesize; uint8_t datalength;
//      encoder_pivot2.read_config(0, &graycode, &multiturn, &framesize, &datalength);
//      ROS_INFO("SSI Channel 0, Written values %d, %d, %d, %d, Received values %d, %d, %d, %d", GRAYCODE, MULTITURN, FRAMESIZE, DATALENGTH,graycode, multiturn, framesize, datalength);
//      encoder_pivot2.read_config(1, &graycode, &multiturn, &framesize, &datalength);
//      ROS_INFO("SSI Channel 1, Written values %d, %d, %d, %d, Received values %d, %d, %d, %d", GRAYCODE, MULTITURN, FRAMESIZE, DATALENGTH,graycode, multiturn, framesize, datalength);

      ROS_INFO("Slaves mapped, state to SAFE_OP.");
      /* wait for all slaves to reach SAFE_OP state */
      ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

      ROS_INFO("segments : %d : %d %d %d %d", ec_group[0].nsegments,
               ec_group[0].IOsegment[0], ec_group[0].IOsegment[1],
               ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

      ROS_INFO("Request operational state for all slaves");
      expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
      ROS_INFO("Calculated workcounter %d", expectedWKC);
      ec_slave[0].state = EC_STATE_OPERATIONAL;
      /* send one valid process data to make outputs in slaves happy*/
      ec_send_processdata();
      ec_receive_processdata(EC_TIMEOUTRET);
      /* request OP state for all slaves */
      ec_writestate(0);
      chk = 40;
      /* wait for all slaves to reach OP state */

      do
      {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
      } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
      if (ec_slave[0].state == EC_STATE_OPERATIONAL)
      {
        ROS_INFO("Operational state reached for all slaves.");
        pdo_transfer_active = TRUE;
        /* Check if slaves are found in the expected order */
        return check_ethercat_slaves();
      }
      else
      {
        ROS_WARN("Not all slaves reached operational state.");
        ec_readstate();
        for (i = 1; i <= ec_slavecount; i++)
        {
          if (ec_slave[i].state != EC_STATE_OPERATIONAL)
          {
            ROS_WARN("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s", i,
                     ec_slave[i].state, ec_slave[i].ALstatuscode,
                     ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
          }
        }
      }
    }
    else
    {
      ROS_ERROR("No slaves found!");
    }
  }
  else
  {
    ROS_ERROR("No socket connection on %s. Try excecuting the following "
              "command: sudo setcap 'cap_net_raw=ep cap_sys_nice=eip' $(readlink $(catkin_find "
              "ethercat_interface ethercat_interface_fedra))\n",
              ifname);
  }
  return FALSE;
}

void stop_ethercat()
{
  /* stop PDO transfer in Thread */
  pdo_transfer_active = FALSE;

  /* request INIT state for all slaves */
  ROS_INFO("Request init state for all slaves");
  ec_slave[0].state = EC_STATE_INIT;
  ec_writestate(0);

  /* stop SOEM, close socket */
  ec_close();
}

bool start_soem(char* ifname)
{
  /* initialise SOEM and bring to operational state*/
  if (!setup_ethercat(ifname)){
    ROS_ERROR("Initialization failed");
    return FALSE;
  }else{
    return TRUE;
  }
}

void* ecat_pdotransfer(void* ptr)
{
  while (ros::ok())
  {
    if (pdo_transfer_active)
    {
      ec_send_processdata();
      wkc = ec_receive_processdata(EC_TIMEOUTRET);
    }
    osal_usleep(PDO_PERIOD);
  }
}

void* ecat_statecheck(void* ptr)
{
  int slave;
  uint8 currentgroup = 0;

  while (ros::ok())
  {
    if (pdo_transfer_active &&
        ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
    {
      /* one ore more slaves are not responding */
      ec_group[currentgroup].docheckstate = FALSE;
      ec_readstate();
      for (slave = 1; slave <= ec_slavecount; slave++)
      {
        if ((ec_slave[slave].group == currentgroup) &&
            (ec_slave[slave].state != EC_STATE_OPERATIONAL))
        {
          ec_group[currentgroup].docheckstate = TRUE;
          if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
          {
            ROS_ERROR("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.",
                      slave);
            ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
            ec_writestate(slave);
          }
          else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
          {
            ROS_WARN("slave %d is in SAFE_OP, change to OPERATIONAL.", slave);
            ec_slave[slave].state = EC_STATE_OPERATIONAL;
            ec_writestate(slave);
          }
          else if (ec_slave[slave].state > 0)
          {
            if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
            {
              ec_slave[slave].islost = FALSE;
              ROS_INFO("MESSAGE : slave %d reconfigured", slave);
            }
          }
          else if (!ec_slave[slave].islost)
          {
            /* re-check state */
            ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
            if (!ec_slave[slave].state)
            {
              ec_slave[slave].islost = TRUE;
              ROS_ERROR("slave %d lost", slave);
            }
          }
        }
        if (ec_slave[slave].islost)
        {
          if (!ec_slave[slave].state)
          {
            if (ec_recover_slave(slave, EC_TIMEOUTMON))
            {
              ec_slave[slave].islost = FALSE;
              ROS_INFO("MESSAGE : slave %d recovered", slave);
            }
          }
          else
          {
            ec_slave[slave].islost = FALSE;
            ROS_INFO("MESSAGE : slave %d found", slave);
          }
        }
      }

      if (!ec_group[currentgroup].docheckstate)
      {
        ROS_INFO("OK : all slaves resumed OPERATIONAL.");
      }
    }
    osal_usleep(STATECHECK_PERIOD);
  }
}


void readPivots()
{
  uint32_t chan1,chan2;
  double q1,q2;

  chan1 = encoder_pivot1.get_input(0);
  chan2 = encoder_pivot1.get_input(1);

  q1 = ((double)chan1)/PIVOT_ENC_RES*2*M_PI;
  q2 = ((double)chan2)/PIVOT_ENC_RES*2*M_PI;
  ROS_DEBUG("encoder_pivot1 Encs: %u %u  Rot: %g %g",chan1,chan2,q1,q2);

  std_msgs::Float64 msg;
  msg.data = q1;
  encoder_pivot1_pub.publish(msg);
  msg.data = q2;
  encoder_pivot2_pub.publish(msg);
}

void pwm1Callback(const std_msgs::Float32::ConstPtr& msg)
{
  ROS_DEBUG("I heard: [%f]", msg->data);
  if(pdo_transfer_active){
    pwmdriver_pivot1.set_output(0,msg->data);
  }
}

void pwm2Callback(const std_msgs::Float32::ConstPtr& msg)
{
  ROS_DEBUG("I heard: [%f]", msg->data);
  if(pdo_transfer_active){
    pwmdriver_pivot1.set_output(1,msg->data);
  }
}

void pwm3Callback(const std_msgs::Float32::ConstPtr& msg)
{
  ROS_DEBUG("I heard: [%f]", msg->data);
  if(pdo_transfer_active){
    pwmdriver_pivot2.set_output(0,msg->data);
  }
}

void pwm4Callback(const std_msgs::Float32::ConstPtr& msg)
{
  ROS_DEBUG("I heard: [%f]", msg->data);
  if(pdo_transfer_active){
    pwmdriver_pivot2.set_output(1,msg->data);
  }
}

void bool1Callback(const std_msgs::Bool::ConstPtr& msg)
{
  ROS_DEBUG("I heard: [%d]", msg->data);
  if(pdo_transfer_active){
    digitalOut_pivot1.set_output(0,msg->data);
  }
}

void bool2Callback(const std_msgs::Bool::ConstPtr& msg)
{
  ROS_DEBUG("I heard: [%d]", msg->data);
  if(pdo_transfer_active){
    digitalOut_pivot1.set_output(1,msg->data);
  }
}

void bool3Callback(const std_msgs::Bool::ConstPtr& msg)
{
  ROS_DEBUG("I heard: [%d]", msg->data);
  if(pdo_transfer_active){
    digitalOut_pivot2.set_output(0,msg->data);
  }
}

void bool4Callback(const std_msgs::Bool::ConstPtr& msg)
{
  ROS_DEBUG("I heard: [%d]", msg->data);
  if(pdo_transfer_active){
    digitalOut_pivot2.set_output(1,msg->data);
  }
}

// Based on: http://www.yonch.com/tech/82-linux-thread-priority
void set_realtime_priority(pthread_t* thread) {
     int ret;
     // struct sched_param is used to store the scheduling priority
     struct sched_param params;

     // We'll set the priority to the maximum.
     params.sched_priority = sched_get_priority_max(SCHED_FIFO);
     ROS_INFO("Trying to set thread realtime prio = %d",params.sched_priority);

     // Attempt to set thread real-time priority to the SCHED_FIFO policy
     ret = pthread_setschedparam(*thread, SCHED_FIFO, &params);
     if (ret != 0) {
         ROS_ERROR("Unsuccessful in setting thread realtime prio, got error: %d. Possible errors: ESRCH(%d), EINVAL(%d), EPERM(%d)",ret,ESRCH,EINVAL,EPERM);
         if(ret==EPERM){
           ROS_ERROR("No appropriate permissions. Try excecuting the following "
                     "command: sudo setcap 'cap_net_raw=ep cap_sys_nice=eip' $(readlink $(catkin_find "
                     "ethercat_interface ethercat_interface_fedra))\n");
         }
         return;
     }
     // Now verify the change in thread priority
     int policy = 0;
     ret = pthread_getschedparam(*thread, &policy, &params);
     if (ret != 0) {
         ROS_ERROR("Couldn't retrieve real-time scheduling parameters, got error: %d. Possible errors: ESRCH(%d), EINVAL(%d), EPERM(%d)",ret,ESRCH,EINVAL,EPERM);
         return;
     }

     // Check the correct policy was applied
     if(policy != SCHED_FIFO) {
         ROS_ERROR("Scheduling is NOT SCHED_FIFO! Got: %d",policy);
     } else {
         ROS_INFO("SCHED_FIFO OK, Thread priority is %d",params.sched_priority);
     }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ethercat_interface_fedra");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  int freq = 10; // in Hz

  ros::NodeHandle nh_priv("~");
  nh_priv.param<int>("freq", freq, freq);
  ros::Rate r(freq);

  encoder_pivot1_pub = nh_priv.advertise<std_msgs::Float64>("encoder1", 1);
  encoder_pivot2_pub = nh_priv.advertise<std_msgs::Float64>("encoder2", 1);
  ros::Subscriber pwm1_sub = nh_priv.subscribe<std_msgs::Float32>("pwm1", 1, pwm1Callback);
  ros::Subscriber pwm2_sub = nh_priv.subscribe<std_msgs::Float32>("pwm2", 1, pwm2Callback);
  ros::Subscriber pwm3_sub = nh_priv.subscribe<std_msgs::Float32>("pwm3", 1, pwm3Callback);
  ros::Subscriber pwm4_sub = nh_priv.subscribe<std_msgs::Float32>("pwm4", 1, pwm4Callback);
  ros::Subscriber bool1_sub = nh_priv.subscribe<std_msgs::Bool>("bool1", 1, bool1Callback);
  ros::Subscriber bool2_sub = nh_priv.subscribe<std_msgs::Bool>("bool2", 1, bool2Callback);
  ros::Subscriber bool3_sub = nh_priv.subscribe<std_msgs::Bool>("bool3", 1, bool3Callback);
  ros::Subscriber bool4_sub = nh_priv.subscribe<std_msgs::Bool>("bool4", 1, bool4Callback);

  std::string ethercat_interface;
  if (nh_priv.getParam("ethercat_interface", ethercat_interface))
  {
    ROS_INFO("configured interface = %s", ethercat_interface.c_str());

    pthread_create(&thread_statecheck, NULL, ecat_statecheck, (void*)&ctime);
    pthread_create(&thread_pdo, NULL, ecat_pdotransfer, (void*)&ctime);

    // Try to set realtime prio on PDO-thread
    set_realtime_priority(&thread_pdo);

    /* start cyclic part */
    char* interface = new char[ethercat_interface.size() + 1];
    std::copy(ethercat_interface.begin(), ethercat_interface.end(), interface);
    interface[ethercat_interface.size()] = '\0';

    if(start_soem(interface)){
      while (ros::ok())
      {
        readPivots();

        r.sleep();
        ros::spinOnce();
      }
    }

    ROS_INFO("stop transferring messages");
    pdo_transfer_active = FALSE;

    ROS_INFO("stop ethercat");
    stop_ethercat();
  }
  else
  {
    ROS_ERROR("no ethercat interface defined, EXIT");
  }

  spinner.stop();

  ROS_INFO("Shutdown completed");

  return 0;
}
