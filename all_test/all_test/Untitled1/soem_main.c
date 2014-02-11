/** \file
 * \brief Main program for the University of Auckland delta robot in Industrial Informatics
 *
 * This program is used to connect to and control a delta robot 
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sched.h>
#include <pthread.h>
#include <sys/time.h>
#include <signal.h>

#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatconfig.h"
#include "ethercatconfig.h"
#include "ethercatcoe.h"
#include "ethercatdc.h"
#include "ethercatprint.h"
#include "ethercatsoe.h"

#include "wago_steppers.h"
#include "state_machine.h"
#include "error.h"

#define NSEC_PER_SEC 1000000000
#define TICK_RATE 100000
#define EtherCAT_TIMEOUT EC_TIMEOUTRET

struct input_msg_t {
	int quit;
};

/**
 * Input handling function, run by input handling thread
 *
 * This function handles commandline input. It could be used at a later time to 
 * facilitate control of the EtherCAT devices using the keyboard. Right now it is only
 * used to signal to the main program that we would like to quit. 
 * \warning don't modify this function so that it updates the iomap. If you want to do 
 * keyboard control, the far better way to implement it is to send events, glibc may 
 * be helpful for this.
 * @param[in] ptr A pointer to the input_msg_t used to signal exit to the other threads
 */
void check_input(void *ptr)
{
	struct input_msg_t *input_msg = (struct input_msg_t *) ptr;
	while (getc(stdin) != 'q');
	
	printf("quitting\n");
	input_msg->quit = 1;
}

/* memory for holding slave data */
char IOmap[4096];

/* commandline args */
char *eth_dev = "eth0";
uint32 cycle_time = 1000;
uint32 move_coord = 0;
 
/* wago device configuration */
const int WAGO_DEVICE_OFFSETS_MOSI[] = {0x0005, 0x0011, 0x001d};
const int WAGO_DEVICE_OFFSETS_MISO[] = {0x0030, 0x003c, 0x0048};

uint32_t cycle_count = 0;

/**
 * Function for testing writing of an SoE parameter.
 *
 * This function tests writing an SoE parameter. It does not work correctly. Need to  
 * examine wireshark logs of TwinCAT3 software and this to check whats different. Only 
 * need to test with TwinCATs inbuilt motor testing rather than actual code.
 * @param[in]	slave The slave number to write to. (currently not used)
 * @param[in]	drive_no Which drive on the slave we're writing to. (currently not used)
 */
void read_soe_info(int slave, int drive_no)
{
	int o_size;
	int i_size;
	
/*	int ret = ec_readIDNmap(1, &o_size, &i_size);
	if (ret > 0)
		printf("Osize: %d, Isize: %d\n", o_size, i_size);
	else
		printf("soe info failed\n");*/

	unsigned char param_buffer[2] = {0x00,0x07};
	int param_buffer_size = sizeof(param_buffer);
	int wkc = ec_SoEwrite(1, 0, EC_SOE_VALUE_B, 15, param_buffer_size, param_buffer, EC_TIMEOUTRXM);
	printf("WKC is: %d\n", wkc);
}

/**
 * Function for testing reading of an SoE parameter.
 *
 * This function tests reading an SoE parameter. It does not work correctly. Need to  
 * examine wireshark logs of TwinCAT3 software and this to check whats different. Only 
 * need to test with TwinCATs inbuilt motor testing rather than actual code.
 * @param[in]	slave The slave number to write to. (currently not used)
 * @param[in]	drive_no Which drive on the slave we're writing to. (currently not used)
 */
void read_soe_info_simp(int slave, int drive_no)
{
	unsigned char param_buffer[20] = {0xFF};
	int param_buffer_size = sizeof(param_buffer);
	int wkc = ec_SoEread(1, 0, EC_SOE_DATASTATE_B, 1, &param_buffer_size, param_buffer, EC_TIMEOUTRXM);
	printf("WKC is: %d\n", wkc);
}

/**
 * Function for testing reading of an SoE parameter.
 *
 * This function tests reading an SoE parameter. It does not work correctly. Need to  
 * examine wireshark logs of TwinCAT3 software and this to check whats different. Only 
 * need to test with TwinCATs inbuilt motor testing rather than actual code. 
 * This is an idea for how to deal with setting up and verifying startup parameters.
 * (use two arrays, one to hold parameter number, one to hold data, then can just loop 
 * through), an array of struct is probably better, struct holding both param and 
 * value.
 * @param[in]	slave The slave number to write to. (currently not used)
 * @param[in]	drive_no Which drive on the slave we're writing to. (currently not used)
 */
void read_soe_info2(int slave, int drive_no) 
{
	unsigned char param_buffer[500] = {0};
	int param_buffer_size = sizeof(param_buffer);

	int element_flags = EC_SOE_DATASTATE_B | EC_SOE_NAME_B | EC_SOE_ATTRIBUTE_B | EC_SOE_UNIT_B | EC_SOE_MIN_B | EC_SOE_MAX_B | EC_SOE_VALUE_B;

	unsigned char read_list[] = {1, 2, 15, 16, 24, 32, 91, 100, 101, 106, 107, 109, 111, 113, 136, 137, 201, 204};

	for (unsigned char i=0; i<sizeof(read_list); i++) {
		printf("Retrieving %d", read_list[i]);
		int wkc = ec_SoEread(1, 1, element_flags, read_list[i], &param_buffer_size, param_buffer, EC_TIMEOUTRXM);
		printf("\tWKC: %d\t", wkc);

		for (int j=0; (j<param_buffer_size) && j<sizeof(param_buffer); j++) {
			printf("byte%d: %u\t", j, param_buffer[j]);
		}
		printf("\n");
	}
}

/**
 * adds an amount of nanoseconds to a timespec
 *
 * Adds addtime (units nanoseconds) to timespec
 * @param[in,out]	ts timespec to add nanoseconds to
 * @param[in]		addtime number of nanoseconds to add
 */
void add_timespec(struct timespec *ts, int64 addtime)
{
	ts->tv_sec += addtime / NSEC_PER_SEC;
	ts->tv_nsec += addtime % NSEC_PER_SEC;
	if ( ts->tv_sec > NSEC_PER_SEC ) {
		ts->tv_sec += (ts->tv_nsec / NSEC_PER_SEC);
		ts->tv_nsec = ts->tv_nsec % NSEC_PER_SEC;
	}
}

#define TIMESPEC_LESS -1
#define TIMESPEC_GREATER 1
#define TIMESPEC_EQUAL 0
/**
 * Compares timespecs for greater than, less than and equality
 *
 * This function takes two timespecs and compares returning greather than, less than, or equality
 * @param[in]	ts1 First timespec
 * @param[in]	ts2 Second timespec	
 * @return Returns TIMESPEC_LESS when ts1 is less than ts2
 * TIMESPEC_GREATER when ts1 is greater than ts2
 * TIMESPEC_EQUAL when ts1 is equal to ts2
 */
int compare_timespec(struct timespec ts1, struct timespec ts2)
{
	if (ts1.tv_sec < ts2.tv_sec)
		return TIMESPEC_LESS;
	if (ts1.tv_sec > ts2.tv_sec)
		return TIMESPEC_GREATER;
	if (ts1.tv_nsec < ts2.tv_nsec)
		return TIMESPEC_LESS;
	if (ts1.tv_nsec > ts2.tv_nsec)
		return TIMESPEC_GREATER;
	return TIMESPEC_EQUAL;
}

/**
 * Subtracts two timespec
 *
 * Subtract ts2 from ts1
 * @param[in]	ts1 First timespec
 * @param[in]	ts2 Second timespec	
 * @param[out]	result difference in time as a timespec
 * @return Returns 0
 */
int subtract_timespec(struct timespec ts1, struct timespec ts2, struct timespec *result)
{
	if ( (ts1.tv_sec < ts2.tv_sec) || ( (ts1.tv_sec == ts2.tv_sec) && (ts1.tv_nsec < ts2.tv_nsec) ) ) {
		return -1;
	}

	result->tv_sec = ts1.tv_sec - ts2.tv_sec;

	if (ts1.tv_nsec < ts2.tv_nsec) {
		ts1.tv_nsec += 1000000000L;
		(result->tv_sec)--;
	}
	result->tv_nsec = ts1.tv_nsec - ts2.tv_nsec;

	return 0;
}

/**
 * PI calculation to get linux time synced to Distributed Clock time
 *
 * This was taken from the SOEM ebox example. I'm not entirely sure what it does.
 * @param[in]	reftime some sort of reference time?
 * @param[in]	cycletime the period of a cycle, probably in nanoseconds
 * @param[out]	offsettime maybe an amount of time to add or remove from next cycle?
 */
void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime)
{
	static int64 integral = 0;

	/* set linux sync point 50us later than DC sync, just as example */
	int64 delta = (reftime - 50000) % cycletime;
	if (delta> (cycletime/2)) delta -= cycletime;
	if (delta>0) integral++;
	if (delta<0) integral--;
	*offsettime = -(delta/100) - (integral/20);
}

/**
 * Initialises the Ethernet connectioon to use for EtherCAT communications
 *
 * @param[in]	ifname The interface name to use, ie. eth0
 * @return ERR_SUCCESS on success, ERR_ETH_DEV_FAIL on failure.
 */
int ethercat_init_device(char *ifname) {
	/* open the ethernet device */
	if (ec_init(ifname) < 0) {
		printf("EtherCAT: Could not initialise device %s\n", ifname);
		return ERR_ETH_DEV_FAIL;
	}
	return ERR_SUCCESS;
}

/**
 * Bring slaves from init to pre-op
 *
 * Configure EtherCAT modules and brings them to pre-op.
 * @return Returns ERR_SUCCESS on success, ERR_CONFIG_FAIL if mailboxes could not be configured, ERR_FAILED_PRE_OP if all slaves could not be brought to pre-op state
 */
int ethercat_init_to_pre_op()
{
	/* configure mailboxes, this requests pre-op */
	int wkc = ec_config_init(FALSE);
	/* FIXME: this should check for the correct number of slaves */
	printf("EtherCAT: Found %d slaves\n", wkc);
	if (wkc <= 0) {
		printf("EtherCAT: Failed to configure mailboxes. Got WKC: %d\n", wkc);
		ec_close();
		return ERR_CONFIG_FAIL;
	}

	ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
	/* FIXME: some of the slaves do not reach pre_op here, but have no problems goign to safe-op later? should find out why so the following can still be checked */
	/*if (ec_slave[0].state != EC_STATE_PRE_OP) {
		printf("EtherCAT: Not all devices reached pre op!\n");
		for (int i=0; i< 4; i++) {
			printf("state%d: %d\n", i, ec_slave[i].state);
		}
		ec_close();
		return ERR_FAILED_PRE_OP;
	}*/
	return ERR_SUCCESS;
}

/**
 * Bring slaves into safe-op
 *
 * Give SOEM access to the I/O map and bring devices into safe-op
 * @return ERR_SUCCESS on success, ERR_EC_NO_SLAVES on failure to map slaves into io map (this error name should probably change...), ERR_FAILED_SAFE_OP if all slaves could not be brought into safe-op state.
 */
int ethercat_pre_op_to_safe_op()
{

	/* find slaves and automatically configure */
	if (ec_config_map(&IOmap) <= 0 ) {
		printf("EtherCAT: Failed to map IOmap\n");
		ec_close();
		return ERR_EC_NO_SLAVES;
	}

	while(EcatError) printf("%s", ec_elist2string());

	printf("%d slaves found and configured\n", ec_slavecount);

	int expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
	printf("Calculated workcounter %d\n", expectedWKC);

	/* wait for all slaves to reach SAFE_OP state */
	ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 3);
	if (ec_slave[0].state != EC_STATE_SAFE_OP) {
		printf("EtherCAT: Not all slaves reached safe operational state\n");
		ec_readstate();
		for (int i=1; i<=ec_slavecount; i++)
		{
			if (ec_slave[i].state != EC_STATE_SAFE_OP)
			{
				printf("EtherCAT: Slave %d State=%2x StatusCode=%4x : %s\n", i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
			}
		}
		return ERR_FAILED_SAFE_OP;
	}

	return ERR_SUCCESS;
}

/**
 * Bring slaves back to pre-op
 *
 * @return Returns ERR_SUCCESS this needs to be updated to check it worked and return an error if it didnt.
 */
int ethercat_safe_op_to_pre_op() {
	ec_slave[0].state = EC_STATE_PRE_OP;
	ec_writestate(0);
	ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);

	return ERR_SUCCESS;
}

/**
 * Bring slaves into operational state
 *
 * @return Returns ERR_SUCCESS on success, ERR_FAILED_OP on failure to bring all slaves into operational state
 */
int ethercat_safe_op_to_op()
{
	ec_slave[0].state = EC_STATE_OPERATIONAL;		
	
	ec_send_processdata();
	int wkc = ec_receive_processdata(EtherCAT_TIMEOUT);
//	printf("got wkc of: %d\n", wkc);

	ec_writestate(0);
//	printf("lowest state: %d\n", ec_readstate());
	ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);

	if (ec_readstate() != EC_STATE_OPERATIONAL) {
		return ERR_FAILED_OP;
	}

	return ERR_SUCCESS;
}

/**
 * Brings slave back to safe-op
 *
 * @return ERR_SUCCESS. This function needs to be updated to detect problems
 */
int ethercat_op_to_safe_op(){

	struct timespec next_run;
	clock_gettime(CLOCK_REALTIME, &next_run);
	struct timespec current_time;
	struct timespec result;

	ec_slave[0].state = EC_STATE_SAFE_OP;
	ec_writestate(0);
	
	/* still need to maintain synchronization until we get back to safe op */		
	/* FIXME: this should probably still be maintained from the ethercat thread instead of doing this */
	while (ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE)!=EC_STATE_SAFE_OP) {
		/* check if timer has not elapsed */
		clock_gettime(CLOCK_REALTIME, &current_time);
  
		if ( compare_timespec(current_time, next_run) == -1 ){
			continue;
		}

		/* timer has elapsed update expiration time */
		memcpy(&next_run, &current_time, sizeof(struct timespec));
		add_timespec(&next_run, TICK_RATE); /* add ns to current time */

		/* timer is sorted, lets go!!! */
		ec_send_processdata();
		ec_receive_processdata(EtherCAT_TIMEOUT);
	}

	return ERR_SUCCESS;
}

/**
 * Ethercat update thread
 *
 * Handles the Initialisation and (should handle) maintainance of EtherCAT network.
 * Updates the I/O map at specific timing intervals. This needs to happen at specific 
 * intervals or the slaves will enter error states.
 * @param[in]	ptr a reference to the structure which keeps track of whether the program should exit (type is input_msg_t).
 */
void ethercat_thread(void * ptr)
{
	printf("Starting input_test\n");

	struct input_msg_t *input_msg = (struct input_msg_t *) ptr;

	if (ethercat_init_device(eth_dev) < 0) return;
	printf("EtherCAT: Initialised device: %s\n", eth_dev);

	if (ethercat_init_to_pre_op() < 0) return;
	printf("EtherCAT: Slaves are in pre-op.\n");

	if (ethercat_pre_op_to_safe_op() < 0) return;
	printf("EtherCAT: Slaves are in safe-op\n");

	/* FIXME: use distributed clocks */
	/* I've read distributed clocks are required to use the AX5000 */
//	ec_configdc();

	if (ethercat_safe_op_to_op() < 0) return;
	printf("EtherCAT: Slaves are in op\n");
//	ec_readstate();
//	read_soe_info(1, 0);

	struct timespec next_run;
	clock_gettime(CLOCK_REALTIME, &next_run);
	struct timespec current_time;
	struct timespec result;

	/* FIXME: this should be updated to be detected automatically, theres no reason other than laziness to do it this way */
	for (int i=0; i<3; i++)
	{
		wago_steppers[i][WAGO_OUTPUT_SPACE] = (struct wago_stepper_t *) &IOmap[WAGO_DEVICE_OFFSETS_MOSI[i]];
		wago_steppers[i][WAGO_INPUT_SPACE] = (struct wago_stepper_t *) &IOmap[WAGO_DEVICE_OFFSETS_MISO[i]];
	}

	/* FIXME: this shouldn't be needed as structure is zeroed in main, remove it and check it still works */
	input_msg->quit = 0;

	while (input_msg->quit == 0) {
		/* FIXME: this timing should probably be changed to use pthread_cond_timedwait like the SOEM ebox example does. pthread_cond_timedwait has the advantage of built in timing error detection */
		/* check if timer has not elapsed */
		clock_gettime(CLOCK_REALTIME, &current_time);
		/* FIXME I think print statements significantly slow this down? should test at some point */	   
		if ( compare_timespec(current_time, next_run) == -1 ){
			/* FIXME: should there be a usleep here to avoid starvation of other thread? */
			continue;
		}

		/* XXX: the following prints out whether timing has been met. The printf() call will very likely cause delay. I would avoid using this, the output probably isn't even reliable */
#ifdef SHOW_TPS
		int valid = subtract_timespec(current_time, next_run, &result);
		printf("valid? %d. difference is %d sec and %ld nsec\n", valid, result.tv_sec, result.tv_nsec);
#endif /* SHOW_TPS */

		/* timer has elapsed update expiration time */
		memcpy(&next_run, &current_time, sizeof(struct timespec));
		add_timespec(&next_run, TICK_RATE); /* add ns to current time */

		/* timer is sorted, lets go!!! */
		// the following should not be needed as it is now done in the ethercat thread 
		pthread_mutex_lock(&io_mutex);
		ec_send_processdata();
		ec_receive_processdata(EtherCAT_TIMEOUT);
		pthread_mutex_unlock(&io_mutex);

		/* FIXME: this is dangerous for time constraints but otherwise can't get other thread to get access :S This is possibly a good candidate for pthread_yield(), I don't know whether pthread_yield has less overhead than usleep though */
		usleep(1);
	}

	/* bring the device back to pre-op so it is safe to disconnect */
//	ethercat_op_to_safe_op();
//	ethercat_safe_op_to_pre_op();

	ec_close();	
}

/**
 * Displays help message on commandline
 */
void help(void)
{
	printf("Welcome to the SOEM based Delta robot controller\n");
	printf("-c = cycle time (us), int\n");
	printf("-d = device, string\n");
	printf("-m = coordinate to move all wago stepper motors to\n");
}

/**
 * Processes commandline arguments
 *
 * @param[in]	argc commandline argument count
 * @param[in]	argv commandline arguments
 */
void process_cmd_opts(int argc, char *argv[])
{	
	int c;
	while ( (c=getopt(argc, argv, "c:d:m:")) != -1) {
		switch (c) {
		case 'c':
			cycle_time = atoi(optarg);
			printf("setting cycle time to %d\n", cycle_time);
			break;
		case 'd':
			eth_dev = optarg;
			printf("setting ethernet device to %s\n", eth_dev);
			break;
		case 'm':
			move_coord = atoi(optarg);
			printf("Moving wago steppers to: %d\n", move_coord);
			break;
		case '?':
			help();
			break;
		default:
			abort();
		}
	}
}

/**
 * main function
 *
 * Sets up threads
 * cyclically calls state machine to update state and I/O's
 * @param[in]	argc commandline argument count
 * @param[in]	argv commandline arguments	
 * @return Returns 0
 */
int main(int argc, char *argv[])
{
	/* FIXME: should probably do some error checking in this function */
	int iret1;
	int ctime;
	struct sched_param schedp;
	struct sched_param param;
	int policy = SCHED_OTHER;
	pthread_t ethercat_thread_handle;		
	pthread_t input_handle;

	printf("SOEM (Simple Open EtherCAT Master)\nInput Test\n");

	process_cmd_opts(argc, argv);

	/* increase thread priority and set to fifo mode */
	memset(&schedp, 0, sizeof(schedp));
	/* do not set priority above 49, otherwise sockets are starved */
	schedp.sched_priority = 30;
	sched_setscheduler(0, SCHED_FIFO, &schedp);

	/* attach input 'hook' */	
	struct input_msg_t input_msg;
	memset(&input_msg, 0, sizeof(input_msg));
	pthread_create( &input_handle , NULL, (void *) &check_input, (void *) &input_msg);
	
	/* create RealTime thread */
	iret1 = pthread_create( &ethercat_thread_handle, NULL, (void *) &ethercat_thread, (void *) &input_msg);

	/* set thread priority */
	memset(&param, 0, sizeof(param));
	param.sched_priority = 40;
	iret1 = pthread_setschedparam(ethercat_thread_handle, policy, &param);

	/* give time for ethercat thread to setup ethercat devices. FIXME: should consider getting ethercat thread to signal back when its ready instead. */
	sleep(2);

	/* start a cyclic routine (update state machine) */
	while (input_msg.quit == 0) {
		/* we're ready to run! */
		usleep(500000);
		if (state_machine() == ERR_STATE_MACHINE_STOPPED) break;
	}

	/* FIXME maybe join threads? */

	schedp.sched_priority = 0;
	sched_setscheduler(0, SCHED_OTHER, &schedp);

	printf("End Program\n");

	return 0;
}				
