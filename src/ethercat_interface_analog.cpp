#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <unistd.h>

#include "ros/ros.h"

#include "ethercat_interface/ethercat_includes.h"
#include "ethercat_interface/ethercat_interface_analog.h"

#include "ethercat_interface/el2008.h"
#include "ethercat_interface/el4002.h"

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

#define EC_TIMEOUTMON 500
#define PDO_PERIOD 5000
#define MOTORGAIN 0.334 // 1/(200/7/60*2*pi)    [V/rad/s]

char IOmap[4096];
pthread_t thread_statecheck;
pthread_t thread_pdo;

volatile int expectedWKC;
volatile int wkc;

boolean pdo_transfer_active = FALSE;

EL2008 digitalOut(&ec_slave[3]);
EL4002 analogOut(&ec_slave[2]);

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
        return TRUE;
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
              "command: sudo setcap cap_net_raw+ep $(readlink $(catkin_find "
              "ethercat_interface ethercat_interface_analog))\n",
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

void start_nobleobot(char* ifname)
{
  /* initialise SOEM and bring to operational state*/
  if (setup_ethercat(ifname))
  {
    ROS_INFO("Initialization succeeded");
  }
  else
  {
    ROS_ERROR("Initialization failed");
  }
}

void* ecat_pdotransfer(void* ptr)
{
  while (1)
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

  while (1)
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
    osal_usleep(10000);
  }
}

EthercatHardware::EthercatHardware()
{
  std::vector<std::string> joint_names;
  joint_names.push_back("left_wheel");
  joint_names.push_back("right_wheel");

  for (size_t i = 0; i < joint_names.size(); ++i)
  {
    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle(joint_names[i], &pos[i], &vel[i], &eff[i]);
    jnt_state_interface.registerHandle(state_handle);

    registerInterface(&jnt_state_interface);

    // connect and register the joint velocity interface
    hardware_interface::JointHandle vel_handle(jnt_state_interface.getHandle(joint_names[i]), &cmd[i]);
    jnt_vel_interface.registerHandle(vel_handle);

    registerInterface(&jnt_vel_interface);
  }

}

void EthercatHardware::writeJoints()
{
  double analog_values[2] = { cmd[0] * MOTORGAIN, cmd[1] * MOTORGAIN };
  
  // Sanitizing input
  if ( std::abs(analog_values[0]) > 10)
  {
    ROS_WARN("Analog value [0] has a value of %f, truncating to 10",analog_values[0]);
    analog_values[0] = std::min(10.0,std::max(-10.0,analog_values[0]));
  }
  if ( std::abs(analog_values[1]) > 10)
  {
    ROS_WARN("Analog value [1] has a value of %f, truncating to 10",analog_values[1]);
    analog_values[1] = std::min(10.0,std::max(-10.0,analog_values[1]));
  }

  // Enabling motor and writing analog values and direction boolean
  digitalOut.set_output(3,std::abs(analog_values[0]) < 1e-5);
  digitalOut.set_output(5,std::abs(analog_values[1]) < 1e-5);
  digitalOut.set_output(2,analog_values[0] > 0.0); // move in the right direction
  digitalOut.set_output(4,analog_values[1] <= 0.0);
  analogOut.set_output(0, std::abs(analog_values[0])); // With this value
  analogOut.set_output(1, std::abs(analog_values[1]));
  ROS_DEBUG( "Digital = [%d, %d, %d], Analog = [%f, %f]", digitalOut.get_output(0), digitalOut.get_output(2), digitalOut.get_output(4), std::abs(analog_values[0]), std::abs(analog_values[1]) );
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ethercat_interface");

  EthercatHardware robot;
  controller_manager::ControllerManager cm(&robot);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  int freq = 50; // in Hz

  ros::NodeHandle nh;
  nh.param<int>("freq", freq, freq);
  ros::Rate r(freq);

  std::string ethercat_interface;
  if (ros::param::get("ethercat_interface", ethercat_interface))
  {
    ROS_INFO("configured interface = %s", ethercat_interface.c_str());

    pthread_create(&thread_statecheck, NULL, ecat_statecheck, (void*)&ctime);
    pthread_create(&thread_pdo, NULL, ecat_pdotransfer, (void*)&ctime);

    /* start cyclic part */

    char* interface = new char[ethercat_interface.size() + 1];
    std::copy(ethercat_interface.begin(), ethercat_interface.end(), interface);
    interface[ethercat_interface.size()] = '\0';

    start_nobleobot(interface);

    digitalOut.set_output(0,true); // Enable motors

    while (ros::ok())
    {
      ros::Time t = ros::Time::now();

      // robot.readJoints();

      cm.update(t, ros::Duration(1.0f / freq));

      robot.writeJoints();

      r.sleep();
    }

    digitalOut.set_output(0,false); // Disable motors

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

  return 0;
}
