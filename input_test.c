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

#define NSEC_PER_SEC 1000000000
#define TICK_RATE 100000


/* Error Codes */
#define ERR_SUCCESS 0
#define ERR_ETH_DEV_FAIL -1
#define ERR_EC_NO_SLAVES -2
#define ERR_FAILED_SAFE_OP -3
#define ERR_FAILED_OP -4
#define ERR_STATE_MACHINE_STOPPED -5

pthread_mutex_t io_mutex = PTHREAD_MUTEX_INITIALIZER;

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

char IOmap[4096];

/* args */
char *eth_dev = "eth0";
uint32 cycle_time = 1000;
uint32 move_coord = 0;

/* wago device configuration */
const int WAGO_DEVICE_OFFSETS_MOSI[] = {0x0005, 0x0011, 0x001d};
const int WAGO_DEVICE_OFFSETS_MISO[] = {0x0030, 0x003c, 0x0048};

/* this struct is configured so that the second address is 0 for master out, 1 for master in */
struct wago_stepper_t *wago_steppers[3][2];
/* io structures */
struct PACKED wago_stepper_t {
	union {
		uint8 value;
		struct {
			uint8 reserved : 5;
			uint8 mbx_mode : 1;
			uint8 error : 1; /* this is read only */
			uint8 reserved2 : 1;
		} bit;
	} stat_cont0;

	uint8 reserved;
	
	union {
		struct {
			uint8 velocity_lbyte;
			uint8 velocity_hbyte;
			uint8 acceleration_lbyte;
			uint8 acceleration_hbyte;
			uint8 position_lbyte;
			uint8 position_mbyte;
			uint8 position_hbyte;
		} positioning;
		struct {
			uint8 opcode;
			uint8 control;
			uint8 mail[4];
			uint8 reserved;
		} mailbox;
	} message;
	
	
	uint8 stat_cont3;
	union {
		uint8 value;
		struct {
			uint8 on_target : 1;
			uint8 busy : 1;
			uint8 standstill : 1;
			uint8 on_speed : 1;
			uint8 direction : 1;
			uint8 reference_ok : 1;
			uint8 precalc_ack : 1;
			uint8 error : 1;
		} status_bits;

		struct {
			uint8 to_be_defined : 8;
		} control_bits;
	} stat_cont2;
	union {
		uint8 value;
		struct {
			uint8 enable : 1;
			uint8 stop2_n : 1;
			uint8 start : 1;
			uint8 m_positioning : 1;
			uint8 m_program : 1;
			uint8 m_reference : 1;
			uint8 m_jog : 1;
			uint8 m_drive_by_mbx : 1;
		} bit;
	} stat_cont1;	
};

typedef struct PACKED
{
	uint8 bit_input;
} in_EK1002t;
typedef struct PACKED
{
	uint8 bit_output;
} out_EK2008t;

typedef struct PACKED
{
	uint8 counter;
	uint16 stream[100];
} in_EK1002_streamt;

#define MAXSTREAM 200000
#define EtherCAT_TIMEOUT EC_TIMEOUTRET

struct sched_param schedp;
int32 cycle_count = 0;
in_EK1002_streamt *in_EK1002;

int streampos; //used uninitialised?!
int16 stream1[MAXSTREAM];
int16 stream2[MAXSTREAM];

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

int state_machine()
{
#ifdef STATE_MACHINE_MAILBOX
	enum states {
		enable_mailbox = 0,
		enable_mailbox_drive,
		init2,
		xxx
	};
	static states current_state = enable_mailbox;
	switch (current_state) {
	case init:
		for (int i=0; i<3; i++) {
			/* control 0 setup */
		`	wago_steppers[i][0]->stat_cont0.bit.mbx_mode = 1;
		}
		current_state = enable_mailbox_drive;
		break;

	case enable_mailbox_drive:
		/* control 1 setup */
		wago_steppers[i][0]->stat_cont1.bit.enable = 1;
		/* set stop2_n, this must be set to be able to set an operating mode */
		wago_steppers[i][0]->stat_cont1.bit.stop2_n = 1;
		wago_steppers[i][0]->stat_cont1.bit.m_drive_by_mbx = 1;
			
		wago_steppers[i][0]->message.mailbox.opcode = 0x40;
		wago_steppers[i][0]->message.mailbox.control = (toggle<<7);
		wago_steppers[i][0]->message.mailbox.mail[0] = 0x02;
		wago_steppers[i][0]->message.mailbox.mail[1] = 42;
		wago_steppers[i][0]->message.mailbox.mail[2] = 42;
		wago_steppers[i][0]->message.mailbox.mail[3] = 42;

		toggle = !toggle;
	
		/* confirm mailbox mode is set */
		for (int i=0; i<3; i++) {
			printf("dev %d mailbox mode is: %d\n", i, wago_steppers[i][1]->stat_cont0.bit.mbx_mode);
		}
		break;
	}
#endif /* STATE_MACHINE_MAILBOX */
#ifdef STATE_MACHINE_WAGO_PROCESS_IMAGE 
	enum states {
		set_terminate_operating_mode = 0,
		confirm_terminate_operating_mode,
		set_setup_mode,
		confirm_setup_mode,
		set_positioning_mode,
		confirm_positioning_mode,
		set_position,
		check_position,
		stop
	};
	
	uint16 max_accel = 5000; 	
	static enum states current_state = set_terminate_operating_mode;
	uint16 max_vel = 5000;
	int confirmed;

	switch (current_state) {
	case set_terminate_operating_mode:
		printf("terminating existing\n");
		
		//printf("term: attempting to lock\n");
		pthread_mutex_lock(&io_mutex);
		//printf("term: locked\n");
		for (int i=0; i<3; i++){
			wago_steppers[i][0]->stat_cont1.bit.enable = 0;
			wago_steppers[i][0]->stat_cont1.bit.stop2_n = 0;
			wago_steppers[i][0]->stat_cont1.bit.start = 0;
		}
		pthread_mutex_unlock(&io_mutex);
		//printf("term: freed\n");

		current_state = confirm_terminate_operating_mode;
		break;
	case confirm_terminate_operating_mode:
		printf("Terminate Operating Mode\n");
		confirmed = 1;
		pthread_mutex_lock(&io_mutex);
		for (int i=0; i<3; i++) {
			if (wago_steppers[i][1]->stat_cont1.bit.enable != 0) {
				printf("%d: Enable not yet reset!\n", i);
				confirmed = 0;
			}
			if (wago_steppers[i][1]->stat_cont1.bit.stop2_n != 0) {
				printf("%d: Stop not yet reset!\n", i);
				confirmed = 0;
			}
			if (wago_steppers[i][1]->stat_cont1.bit.start != 0) {
				printf("%d: start not yet reset!\n", i);
				confirmed = 0;
			}
	
		}
		pthread_mutex_unlock(&io_mutex);
		if (confirmed) 
			current_state=set_setup_mode;
		break;
	case set_setup_mode:
		pthread_mutex_lock(&io_mutex);
		for (int i=0; i<3; i++) {
			wago_steppers[i][0]->stat_cont1.bit.enable = 1;
			wago_steppers[i][0]->stat_cont1.bit.stop2_n = 1;
			wago_steppers[i][0]->stat_cont1.bit.start = 0;
		}
		pthread_mutex_unlock(&io_mutex);
		current_state = confirm_setup_mode;
		break;
	case confirm_setup_mode:
		printf("Setup Mode\n");
		confirmed = 1;
		pthread_mutex_lock(&io_mutex);
		for (int i=0; i<3; i++) {
			if (wago_steppers[i][1]->stat_cont1.bit.enable != 1) {
				printf("%d: Trying to set enable to: %d Enable is: %d!\n", i, wago_steppers[i][0]->stat_cont1.bit.enable, wago_steppers[i][1]->stat_cont1.bit.enable);
				confirmed = 0;
			}
			if (wago_steppers[i][1]->stat_cont1.bit.stop2_n != 1) {
				printf("%d: Trying to set stop to: %d Stop is: %d!\n", i, wago_steppers[i][0]->stat_cont1.bit.stop2_n, wago_steppers[i][1]->stat_cont1.bit.stop2_n);
				confirmed = 0;
			}
			if (wago_steppers[i][1]->stat_cont1.bit.start != 0) {
				printf("%d: Trying to set start to: %d Start is: %d!\n", i, wago_steppers[i][0]->stat_cont1.bit.start, wago_steppers[i][1]->stat_cont1.bit.start);
				confirmed = 0;
			}	
		}
		pthread_mutex_unlock(&io_mutex);

		if (confirmed)
			current_state=set_positioning_mode;
		break;
	case set_positioning_mode:
		pthread_mutex_lock(&io_mutex);
		for (int i=0; i<3; i++) {
			wago_steppers[i][0]->stat_cont1.bit.m_positioning = 1;
		}
		pthread_mutex_unlock(&io_mutex);
		current_state = confirm_positioning_mode;
		break;
	case confirm_positioning_mode:
		pthread_mutex_lock(&io_mutex);
		printf("positioning mode active? s1:%s s2:%s s3:%s\n",
			(wago_steppers[0][1]->stat_cont1.bit.m_positioning)?"y":"n",
			(wago_steppers[1][1]->stat_cont1.bit.m_positioning)?"y":"n",	
			(wago_steppers[2][1]->stat_cont1.bit.m_positioning)?"y":"n"
		);
		int all_ready = 1;
		for (int i=0; i<3; i++) {
			if (!wago_steppers[i][1]->stat_cont1.bit.m_positioning){
				all_ready = 0;
			}
		}
		pthread_mutex_unlock(&io_mutex);
		if (all_ready)	
			current_state = set_position;
		break;
	case set_position:
		pthread_mutex_lock(&io_mutex);
		for (int i=0; i<3; i++) {
			printf("m_positioning? %d\n", wago_steppers[i][1]->stat_cont1.bit.m_positioning);
			
			wago_steppers[i][0]->message.positioning.velocity_lbyte = (uint8) ((max_vel>>0)&0xFF);
			wago_steppers[i][0]->message.positioning.velocity_hbyte = (uint8) ((max_vel>>8)&0xFF);

			wago_steppers[i][0]->message.positioning.acceleration_lbyte = (uint8) ((max_accel>>0)&0xFF);
			wago_steppers[i][0]->message.positioning.acceleration_hbyte = (uint8) ((max_accel>>8)&0xFF);
			
			wago_steppers[i][0]->message.positioning.position_lbyte = (uint8) ((move_coord>>0)&0xFF);
			wago_steppers[i][0]->message.positioning.position_mbyte = (uint8) ((move_coord>>8)&0xFF);
			wago_steppers[i][0]->message.positioning.position_hbyte = (uint8) ((move_coord>>16)&0xFF);

			printf("err? %d\n", wago_steppers[i][1]->stat_cont0.bit.error);

			wago_steppers[i][0]->stat_cont1.bit.start = 1;
		}
		pthread_mutex_unlock(&io_mutex);
		current_state = check_position;
		break;
	case check_position:
		pthread_mutex_lock(&io_mutex);
		if (wago_steppers[0][1]->stat_cont2.status_bits.on_target){
			printf("Reached destination!\n");
			current_state = stop;
		}
		pthread_mutex_unlock(&io_mutex);
		break;

	case stop:
		pthread_mutex_lock(&io_mutex);
		printf("positioning mode: %d\n", wago_steppers[0][1]->stat_cont1.bit.m_positioning);
		pthread_mutex_unlock(&io_mutex);
		return ERR_STATE_MACHINE_STOPPED;
		break;
	default:
		printf("ERROR: should not be in this state\n");
		break;

	}
	return 0;
#endif /* STATE_MACHINE_WAGO_PROCESS_IMAGE */
}

/* Bring slaves from init to pre-op */
int ethercat_init_to_pre_op(char *ifname)
{
	/* initialise SOEM, bind socket to ifname */
	return (ec_init(ifname)>0) ? ERR_SUCCESS : ERR_ETH_DEV_FAIL;
}

/* Bring slaves into safe-op */
int ethercat_pre_op_to_safe_op()
{
	/* find slaves and automatically configure */
	if (ec_config(FALSE, &IOmap) <= 0 ) {
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
		printf("Not all slaves reached safe operational state\n");
		ec_readstate();
		for (int i=1; i<=ec_slavecount; i++)
		{
			if (ec_slave[i].state != EC_STATE_SAFE_OP)
			{
				printf("Slave %d State=%2x StatusCode=%4x : %s\n", i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
			}
		}
		return ERR_FAILED_SAFE_OP;
	}

}

/* Bring slaves back to pre_op */
int ethercat_safe_op_to_pre_op() {
	ec_slave[0].state = EC_STATE_PRE_OP;
	ec_writestate(0);
	ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
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
	/*for (int i=1; i<=ec_slavecount; i++) {
		uint8 state = ec_statecheck(i, 8, EC_TIMEOUTSTATE);
		printf("Slave %d state: %u\n", i,state);
	}*/

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
	while (ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE)!=EC_STATE_SAFE_OP) {
		/* check if timer has not elapsed */
		clock_gettime(CLOCK_REALTIME, &current_time);
		/* FIXME I think print statements significantly slow this down? should test at some point */	   
		if ( compare_timespec(current_time, next_run) == -1 ){
			continue;
		}

		/* timer has elapsed update expiration time */
		memcpy(&next_run, &current_time, sizeof(struct timespec));
		add_timespec(&next_run, TICK_RATE); /* add ns to current time */

		/* timer is sorted, lets go!!! */
		// the following should not be needed as it is now done in the ethercat thread 
		ec_send_processdata();
		ec_receive_processdata(EtherCAT_TIMEOUT);
	}

}

void ethercat_thread(void * ptr)//(struct input_msg_t *input_msg)
{
	printf("Starting input_test\n");

	struct input_msg_t *input_msg = (struct input_msg_t *) ptr;

	if (ethercat_init_to_pre_op(eth_dev) < 0) {
		printf("EtherCAT: Can't open device %s\n", eth_dev);
		return;
	}
	printf("EtherCAT: Initialised on %s.\n", eth_dev);


	if (ethercat_pre_op_to_safe_op() < 0) {
		printf("No slaves found\n");
		return;
	}
	printf("EtherCAT: Slaves are in safe-op\n");
	/* use distributed clocks */
	ec_configdc();

	if (ethercat_safe_op_to_op() < 0) {
		printf("EtherCAT: Failed to bring slaves into operational state\n");
		return;
	}
	printf("EtherCAT: Slaves are in op\n");
//	ec_readstate();

	struct timespec next_run;
	clock_gettime(CLOCK_REALTIME, &next_run);
	struct timespec current_time;
	struct timespec result;

	for (int i=0; i<3; i++)
	{
		wago_steppers[i][0] = (struct wago_stepper_t *) &IOmap[WAGO_DEVICE_OFFSETS_MOSI[i]];
		wago_steppers[i][1] = (struct wago_stepper_t *) &IOmap[WAGO_DEVICE_OFFSETS_MISO[i]];
	}
	/*wago_steppers = { 
		{ (struct wago_stepper_t *) &IOmap[WAGO_DEVICE_OFFSETS_MOSI[0]], (struct wago_stepper_t *) &IOmap[WAGO_DEVICE_OFFSETS_MISO[0]] },
	   	{ (struct wago_stepper_t *) &IOmap[WAGO_DEVICE_OFFSETS_MOSI[1]], (struct wago_stepper_t *) &IOmap[WAGO_DEVICE_OFFSETS_MISO[1]] },
		{ (struct wago_stepper_t *) &IOmap[WAGO_DEVICE_OFFSETS_MOSI[2]], (struct wago_stepper_t *) &IOmap[WAGO_DEVICE_OFFSETS_MISO[2]] }
	};*/

	while (input_msg->quit == 0) {
		/* check if timer has not elapsed */
		clock_gettime(CLOCK_REALTIME, &current_time);
		/* FIXME I think print statements significantly slow this down? should test at some point */	   
		if ( compare_timespec(current_time, next_run) == -1 ){
			/* FIXME should there be a usleep here to avoid starvation of other thread? */
			continue;
		}
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

		/* FIXME this is dangerous for time constraints but otherwise can't get other thread to get access :S */
		usleep(1);
	}

	/* bring the device back to pre-op so it is safe to disconnect */
	ethercat_op_to_safe_op();
	ethercat_safe_op_to_pre_op();

	ec_close();	
}

void ethercat_thread7( void *ptr )
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
	
/*	if (ethercat_init_to_pre_op(eth_dev) < 0) {
		printf("EtherCAT: Can't open device %s\n", eth_dev);
		return;
	}
	printf("EtherCAT: Initialised on %s.\n", eth_dev);*/

	while (1) {
		/* calculate next cycle start */
		add_timespec(&ts, cycletime + toff);

		/* wait until cycle start time */
		rc = pthread_cond_timedwait(&cond, &mutex, &ts);

		rc = gettimeofday(&tp, NULL);

//		ec_send_processdata();
//		ec_receive_processdata(EtherCAT_TIMEOUT); //1 second timeout, this should probably be decreased?

		//not needed?
		cycle_count++;

		/* weird error detection block */
	/*	if ((in_EK1002->counter != pcounter) && (streampos < (MAXSTREAM-1))) {*/
			/* 
			 *  check if there are timing problems in master
			 * if so, overwrite stream data so it shows up clearly in plots? 
			 */
/*			if (in_EK1002->counter > (pcounter+1)) {
				printf("timing problem?!\n");
				for (int i=0; i<50; i++) {
					stream1[streampos] = 20000;
					stream2[streampos++] = -20000;
				}
			} else {
				for (int i=0; i<50; i++) {
					stream1[streampos] = in_EK1002->stream[(i * 2)];
					stream2[streampos] = in_EK1002->stream[(i * 2) + 1];
				}
			}
			pcounter = in_EK1002->counter;
		}*/
		
		/*calculate toff to get linux time and ?DC? synced */
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
	struct sched_param param;
	int policy = SCHED_OTHER;
	pthread_t ethercat_thread_handle;		
	pthread_t input_handle;

	printf("SOEM (Simple Open EtherCAT Master)\nInput Test\n");

	process_cmd_opts(argc, argv);

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

	struct timespec next_run;
	clock_gettime(CLOCK_REALTIME, &next_run);
	struct timespec current_time;
	struct timespec result;

	/* FIXME: clean up triple timer thing, its just stupid */
	while (input_msg.quit == 0) {
		/* check if timer has not elapsed */
/*		clock_gettime(CLOCK_REALTIME, &current_time);
		if ( compare_timespec(current_time, next_run) == -1 ){
			usleep(10000);
			continue;
		}*/
#ifdef SHOW_TPS
		int valid = subtract_timespec(current_time, next_run, &result);
		printf("valid? %d. difference is %d sec and %ld nsec\n", valid, result.tv_sec, result.tv_nsec);
#endif /* SHOW_TPS */

		/* timer has elapsed update expiration time */
	/*	memcpy(&next_run, &current_time, sizeof(struct timespec));
		add_timespec(&next_run, TICK_RATE);*/ /* add ns to current time */
		
		/* we're ready to run! */
//		static int toggle = 0;
//		if (counter > 1000) {
		usleep(500000);
			if (state_machine() == ERR_STATE_MACHINE_STOPPED) break;
//		}
	}

	/* FIXME maybe join thread? */

	schedp.sched_priority = 0;
	sched_setscheduler(0, SCHED_OTHER, &schedp);

	printf("End Program\n");

	return 0;
}				
