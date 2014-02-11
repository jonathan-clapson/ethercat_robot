/** \file
 * \brief Main program for the University of Auckland delta robot in Industrial Informatics
 *
 * This program is used to connect to a delta robot 
 *
 * Authors:
 * Jonathan Clapson (NOV 2013-FEB 2014)
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

void read_soe_info_simp(int slave, int drive_no)
{
	unsigned char param_buffer[20] = {0xFF};
	int param_buffer_size = sizeof(param_buffer);
	int wkc = ec_SoEread(1, 0, EC_SOE_DATASTATE_B, 1, &param_buffer_size, param_buffer, EC_TIMEOUTRXM);
	printf("WKC is: %d\n", wkc);
}

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

void init_servo(int slave, int drive_no) 
{

//	int wkc = ec_SoEwrite(slave, drive_no, 
}

void add_timespec(struct timespec *ts, int64 addtime)
{
	ts->tv_sec += addtime / NSEC_PER_SEC;
	ts->tv_nsec += addtime % NSEC_PER_SEC;
	if ( ts->tv_sec > NSEC_PER_SEC ) {
		ts->tv_sec += (ts->tv_nsec / NSEC_PER_SEC);
		ts->tv_nsec = ts->tv_nsec % NSEC_PER_SEC;
	}
}

int compare_timespec(struct timespec ts1, struct timespec ts2)
{
	if (ts1.tv_sec < ts2.tv_sec)
		return -1;
	if (ts1.tv_sec > ts2.tv_sec)
		return 1;
	if (ts1.tv_nsec < ts2.tv_nsec)
		return -1;
	if (ts1.tv_nsec > ts2.tv_nsec)
		return 1;
	return 0;
}

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

/* PI calculation to get linux time synced to Distributed Clock time */
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

int ethercat_init_device(char *ifname) {
	/* open the ethernet device */
	if (ec_init(ifname) < 0) {
		printf("EtherCAT: Could not initialise device %s\n", ifname);
		return ERR_ETH_DEV_FAIL;
	}
	return ERR_SUCCESS;
}

/* Bring slaves from init to pre-op */
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

/* Bring slaves into safe-op */
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

/* Bring slaves back to pre_op */
int ethercat_safe_op_to_pre_op() {
	ec_slave[0].state = EC_STATE_PRE_OP;
	ec_writestate(0);
	ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);

	return ERR_SUCCESS;
}

/* Bring slaves into op */
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

/* Bring slaves back to safe op */
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

void ethercat_thread(void * ptr)//(struct input_msg_t *input_msg)
{
	printf("Starting input_test\n");

	struct input_msg_t *input_msg = (struct input_msg_t *) ptr;

	if (ethercat_init_device(eth_dev) < 0) return;
	printf("EtherCAT: Initialised device: %s\n", eth_dev);

	if (ethercat_init_to_pre_op() < 0) return;
	printf("EtherCAT: Slaves are in pre-op.\n");

	if (ethercat_pre_op_to_safe_op() < 0) return;
	printf("EtherCAT: Slaves are in safe-op\n");

	/* use distributed clocks */
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
		/* check if timer has not elapsed */
		clock_gettime(CLOCK_REALTIME, &current_time);
		/* FIXME I think print statements significantly slow this down? should test at some point */	   
		if ( compare_timespec(current_time, next_run) == -1 ){
			/* FIXME should there be a usleep here to avoid starvation of other thread? */
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

/* this function is the slightly modified original code from the ebox example. I think the pthread_cond_timedwait is a better method of syncing than I'm currently using, should switch to this when possible */
void ethercat_thread_orig( void *ptr )
{
	struct timespec ts;
	struct timeval tp;
	int rc;
	int ht;
	int pcounter = 0;
	int64 cycletime = *(int*) ptr * 1000; /* cycletime in ns */

	pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
	pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

	rc = pthread_mutex_lock(&mutex);
	rc = gettimeofday(&tp, NULL);

	/* Convert from timeval to timespec */
	ts.tv_sec = tp.tv_sec;
	ht = (tp.tv_usec/1000) + 1; /* round to nearest ms */
	ts.tv_nsec = ht * 1000000;
	static int64 toff = 0;

	printf("RealTime EtherCAT Thread started.\n");
	
	while (1) {
		/* calculate next cycle start */
		add_timespec(&ts, cycletime + toff);

		/* wait until cycle start time */
		rc = pthread_cond_timedwait(&cond, &mutex, &ts);

		rc = gettimeofday(&tp, NULL);

		ec_send_processdata();
		ec_receive_processdata(EtherCAT_TIMEOUT); //1 second timeout, this should probably be decreased?

		//not needed? could be used to detect errors?
		cycle_count++;
		
		/* calculate toff to get linux time and ?DC? synced */
		ec_sync(ec_DCtime, cycletime, &toff);	
	}
}

void help(void)
{
	printf("This help is not particularly useful!\n");
	printf("-c = cycle time (us), int\n");
	printf("-d = device, string\n");
}

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

int main(int argc, char *argv[])
{
	/* should probably do some error checking in this function */
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

	/* start a cyclic routine */
	//input_test(&input_msg);

	int counter = 0;
	sleep(2);


	/* Control loop */
	while (input_msg.quit == 0) {
		/* we're ready to run! */
		usleep(500000);
		if (state_machine() == ERR_STATE_MACHINE_STOPPED) break;
	}

	/* FIXME maybe join thread? */

	schedp.sched_priority = 0;
	sched_setscheduler(0, SCHED_OTHER, &schedp);

	printf("End Program\n");

	return 0;
}				
