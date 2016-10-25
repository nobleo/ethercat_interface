#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <unistd.h>

#include "ros/ros.h"

#include "ethercat_demo/ethercat_includes.h"

#include "ethercat_demo/velocity_cmd.h"
#include "ethercat_demo/el7332.h"
#include "ethercat_demo/el2008.h"

#include "ultrasonic_sensor_interface/distance.h"

#define EC_TIMEOUTMON 500
#define PDO_PERIOD 5000

char IOmap[4096];
pthread_t thread_statecheck;
pthread_t thread_pdo;

volatile int expectedWKC;
volatile int wkc;

boolean pdo_transfer_active = FALSE;

EL7332 motordriver(&ec_slave[3],3);
EL2008 digitalOut(&ec_slave[2]);

boolean setup_ethercat(char *ifname)
{
    int i, j, chk;
   
    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname))
    {   
        printf("ec_init on %s succeeded.\n",ifname);
        /* find and auto-config slaves */

        if ( ec_config_init(FALSE) > 0 )
        {
            printf("%d slaves found and configured.\n",ec_slavecount);

            ec_config_map(&IOmap);

            ec_configdc();

	        /* write configuration parameters for motor driver */
	        uint16_t max_voltage = 12000;
	        uint16_t max_voltage_received;
            max_voltage_received = motordriver.max_voltage(max_voltage);
	        printf("Written value %d, Received value %d\n", max_voltage, max_voltage_received);


            printf("Slaves mapped, state to SAFE_OP.\n");
            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

            printf("segments : %d : %d %d %d %d\n",ec_group[0].nsegments ,ec_group[0].IOsegment[0],ec_group[0].IOsegment[1],ec_group[0].IOsegment[2],ec_group[0].IOsegment[3]);

            printf("Request operational state for all slaves\n");
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);
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
            }
            while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
            if (ec_slave[0].state == EC_STATE_OPERATIONAL )
            {
                printf("Operational state reached for all slaves.\n");
			    pdo_transfer_active = TRUE;
			    return TRUE;
            }
            else
            {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for(i = 1; i<=ec_slavecount ; i++)
                {
                    if(ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                            i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }           
        }
        else
        {
            printf("No slaves found!\n");
        }
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n",ifname);
    } 
    return FALSE;
}

void stop_ethercat()
{
		/* stop PDO transfer in Thread */
		pdo_transfer_active = FALSE;

		/* request INIT state for all slaves */
		printf("\nRequest init state for all slaves\n");
		ec_slave[0].state = EC_STATE_INIT;
		ec_writestate(0);

		/* stop SOEM, close socket */
		ec_close();
}


void start_nobleo_bot(char *ifname)
{
	/* initialise SOEM and bring to operational state*/
	if(setup_ethercat(ifname))
	{
		motordriver.enable(0,TRUE);
		motordriver.enable(1,TRUE);
	}
	else
	{
		printf("Initialization failed\n");
	}
}

void *ecat_pdotransfer(void *ptr){
	while(1)
	{
		if(pdo_transfer_active)
		{
			ec_send_processdata();
			wkc = ec_receive_processdata(EC_TIMEOUTRET);
		}
		osal_usleep(PDO_PERIOD);
	}
}

void *ecat_statecheck( void *ptr )
{
	int slave;
	uint8 currentgroup = 0;

	while(1)
	{
		if( pdo_transfer_active && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
		{
			/* one ore more slaves are not responding */
			ec_group[currentgroup].docheckstate = FALSE;
			ec_readstate();
			for (slave = 1; slave <= ec_slavecount; slave++)
			{
				if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
				{
					ec_group[currentgroup].docheckstate = TRUE;
					if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
					{
						printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
						ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
						ec_writestate(slave);
					}
					else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
					{
						printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
						ec_slave[slave].state = EC_STATE_OPERATIONAL;
						ec_writestate(slave);
					}
					else if(ec_slave[slave].state > 0)
					{
						if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
						{
							ec_slave[slave].islost = FALSE;
							printf("MESSAGE : slave %d reconfigured\n",slave);
						}
					}
					else if(!ec_slave[slave].islost)
					{
						/* re-check state */
						ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
						if (!ec_slave[slave].state)
						{
							ec_slave[slave].islost = TRUE;
							printf("ERROR : slave %d lost\n",slave);
						}
					}
				}
				if (ec_slave[slave].islost)
				{
					if(!ec_slave[slave].state)
					{
						if(ec_recover_slave(slave, EC_TIMEOUTMON))
						{
							ec_slave[slave].islost = FALSE;
							printf("MESSAGE : slave %d recovered\n",slave);
						}
					}
					else
					{
						ec_slave[slave].islost = FALSE;
						printf("MESSAGE : slave %d found\n",slave);
					}
				}
			}

			if(!ec_group[currentgroup].docheckstate){
				printf("OK : all slaves resumed OPERATIONAL.\n");
			}
		}
		osal_usleep(10000);
	}
}

void velocityCallback(const ethercat_demo::velocity_cmd::ConstPtr& msg)
{
  	//ROS_INFO("I heard: [%f, %f]", msg->velocity_left, msg->velocity_right);
	float vl = msg->velocity_left;
	float vr = msg->velocity_right;
	float vmax = 12000;	

	vl = (vl<vmax)?((vl>-vmax)?vl:-vmax):vmax;	
	vr = (vr<vmax)?((vr>-vmax)?vr:-vmax):vmax;

	motordriver.set_velocity(0,-vr);
	motordriver.set_velocity(1, vl);
	digitalOut.toggle_output(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velocity_to_ethercat");

    ros::NodeHandle n;

    ros::Subscriber velocity_sub = n.subscribe("velocity_in", 1000, velocityCallback);

    std::string ethercat_interface;
    if (ros::param::get("/ethercat_interface", ethercat_interface))
    {
        printf("configured interface = %s\n",ethercat_interface.c_str());

        pthread_create(&thread_statecheck, NULL, ecat_statecheck, (void*) &ctime);
        pthread_create(&thread_pdo, NULL, ecat_pdotransfer, (void*) &ctime);

        /* start cyclic part */

        char * interface = new char[ethercat_interface.size() + 1];
        std::copy(ethercat_interface.begin(), ethercat_interface.end(), interface);
        interface[ethercat_interface.size()] = '\0';

        start_nobleo_bot(interface);

        ros::spin();
        
        printf("stop transferring messages\n");
        pdo_transfer_active = FALSE;

        printf("stop ethercat\n");
        stop_ethercat();
    }
    else
    {
        printf("no ethercat interface defined, EXIT");
    }
    return 0;
}