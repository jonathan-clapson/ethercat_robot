#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sched.h>
#include <pthread.h>
#include <sys/time.h>

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

char IOmap[4096];

/* args */
char *eth_dev = "eth0";
uint32 cycle_time = 1000;

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
pthread_t ethercat_thread_handle;		
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
uint32 cycle_count = 0;
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

void state_machine()
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
#ifdef STATE_MACHINE_PROCESS_IMAGE 
	enum states {
		terminate_operating_mode = 0,
		select_mode,
		set_positioning_mode,
		goto_stepping,
		check_stepping,
		stop
	};
	
	static int position = 1000000;
	uint16 max_accel = 5000; 	
	static enum states current_state = terminate_operating_mode;
	uint16 max_vel = 5000;
	switch (current_state) {
	case terminate_operating_mode:
		printf("terminating existing\n");
		for (int i=0; i<3; i++){
			wago_steppers[i][0]->stat_cont1.bit.enable = 0;
			wago_steppers[i][0]->stat_cont1.bit.stop2_n = 0;
			wago_steppers[i][0]->stat_cont1.bit.start = 0;
		}
		current_state = select_mode;
		break;
	case select_mode:
		printf("last: en %d, stop %d, start %d\n", wago_steppers[0][1]->stat_cont1.bit.enable, wago_steppers[0][1]->stat_cont1.bit.stop2_n, wago_steppers[0][1]->stat_cont1.bit.start);
		for (int i=0; i<3; i++) {
			wago_steppers[i][0]->stat_cont1.bit.enable = 1;
			wago_steppers[i][0]->stat_cont1.bit.stop2_n = 1;
			wago_steppers[i][0]->stat_cont1.bit.start = 0;
		}
		current_state = set_positioning_mode;
		break;
	case set_positioning_mode:
		printf("last: en %d, stop %d, start %d\n", wago_steppers[0][1]->stat_cont1.bit.enable, wago_steppers[0][1]->stat_cont1.bit.stop2_n, wago_steppers[0][1]->stat_cont1.bit.start);
		for (int i=0; i<3; i++) {
			wago_steppers[i][0]->stat_cont1.bit.m_positioning = 1;
		}
		current_state = goto_stepping;
		break;
	case goto_stepping:
		printf("m_positioning? %d\n", wago_steppers[0][1]->stat_cont1.bit.m_positioning);
		for (int i=0; i<3; i++) {
			wago_steppers[i][0]->message.positioning.velocity_lbyte = (uint8) ((max_vel>>0)&0xFF);
			wago_steppers[i][0]->message.positioning.velocity_hbyte = (uint8) ((max_vel>>8)&0xFF);

			wago_steppers[i][0]->message.positioning.acceleration_lbyte = (uint8) ((max_accel>>0)&0xFF);
			wago_steppers[i][0]->message.positioning.acceleration_hbyte = (uint8) ((max_accel>>8)&0xFF);
			
			wago_steppers[i][0]->message.positioning.position_lbyte = (uint8) ((position>>0)&0xFF);
			wago_steppers[i][0]->message.positioning.position_mbyte = (uint8) ((position>>8)&0xFF);
			wago_steppers[i][0]->message.positioning.position_hbyte = (uint8) ((position>>16)&0xFF);

			printf("err? %d\n", wago_steppers[i][1]->stat_cont0.bit.error);

			wago_steppers[i][0]->stat_cont1.bit.start = 1;
		}
		break;
	case check_stepping:

		if (wago_steppers[0][1]->stat_cont2.status_bits.on_target)
			printf("Reached destination!\n");

		current_state = check_stepping;
		break;

	case stop:
		printf("positioning mode: %d\n", wago_steppers[0][1]->stat_cont1.bit.m_positioning);
		break;
	default:
		printf("ERROR: should not be in this state\n");
		break;

	}
#endif /* STATE_MACHINE_PROCESS_IMAGE */
}

void input_test(char *ifname)
{
	printf("Starting input_test\n");

	/* initialise SOEM, bind socket to ifname */
	if (!ec_init(eth_dev))
	{
		printf("Can't open device %s\n", eth_dev);
		return;
	}
	
	printf("EtherCAT initialised on %s\n", eth_dev);
	
	/* find slaves and automatically configure */
	if (ec_config(FALSE, &IOmap) <= 0 ) {
		printf("No slaves found\n");
		ec_close();
		return;
	}

	ec_configdc();

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
	}

//	ec_readstate();
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
	printf("lowest state: %d\n", ec_readstate());

#ifdef COUNTER
	int counter = 0;
#endif
#ifdef MAGNET_COUNTER
	int magnet_direction = 0;
#endif

#ifdef MBX_TEST
	ec_mbxbuft rx;
	ec_mbxbuft tx;
#endif

#define TICK_RATE 100000


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


	while (1) {
		/* check if timer has not elapsed */
		clock_gettime(CLOCK_REALTIME, &current_time);
		/* FIXME I think print statements significantly slow this down? should test at some point */	   
		if ( compare_timespec(current_time, next_run) == -1 ){
			continue;
		}
		//printf("here2\n");
#ifdef SHOW_TPS
		int valid = subtract_timespec(current_time, next_run, &result);
		printf("valid? %d. difference is %d sec and %ld nsec\n", valid, result.tv_sec, result.tv_nsec);
#endif /* SHOW_TPS */

		/* timer has elapsed update expiration time */
		memcpy(&next_run, &current_time, sizeof(struct timespec));
		add_timespec(&next_run, TICK_RATE); /* add ns to current time */

		/* timer is sorted, lets go!!! */
		ec_send_processdata();
		ec_receive_processdata(EtherCAT_TIMEOUT);

#if defined(STATE_MACHINE_PROCESS_IMAGE) || defined(STATE_MACHINE_MAILBOX)
	static int toggle = 0;
	if (counter > 10000) {
		state_machine();
	}
#endif /* WAGO_DRIVE_MOTOR */


#ifdef WAGO_TEST_SLAVE	
		if (counter > 10000) {	
			printf("IOmap 0x0005: %u\n", IOmap[0x0005]);
			printf("IOmap 0x0030: %u\n", IOmap[0x0030]);
			printf("IOmap 0x003c: %u\n", IOmap[0x003c]);
			printf("IOmap 0x0048: %u\n", IOmap[0x0048]);
			printf("IOmap 0x0000: %u\n", IOmap[0x0000]);			
			printf("IOmap 0x0029: %u\n", IOmap[0x0029]);
			
			IOmap[WAGOBASE1+CONT0_OFFSET] = CONTSTAT0_BIT_MBX_EN;
			IOmap[WAGOBASE2+CONT0_OFFSET] = CONTSTAT0_BIT_MBX_EN;
			IOmap[WAGOBASE3+CONT0_OFFSET] = CONTSTAT0_BIT_MBX_EN;
		}
#endif /* WAGO_TEST_SLAVE */

/*		if (counter > 10000) {
			printf("IOmap 0x0???: %u\n", IOmap[0x00??]);

			IOmap[
		}*/
		/*struct stepper_24v_t stepper_state;
		memset(&stepper_state, 0, sizeof(struct stepper_24v_t));
		stepper_state.stat_cont0.bit.mbx_mode = 0;

		int data_size = sizeof(uint8);
		printf("data is: %u, data size is %u\n", stepper_state.stat_cont0, data_size);
		//ec_TxPDO(2, 0x0005, &data_size, &(stepper_state.stat_cont0), EC_TIMEOUTTXM);

		printf("wrote %u\n", data_size);

		uint8 ret;
		ec_RxPDO(2,  0x6000, sizeof(uint8), &ret);
		printf("ret: %u\n", ret);
		fflush(stdout);*/
		//ec_mbxsend(4, 

#ifdef MBX_TEST
		//mailbox commands, unused
//		ec_clearmbx(rx);
//		ec_clearmbx(tx);

//		ec_mboxsend(4, &tx, ECTIMEOUTTXM);
//		ec_mboxreceive(4, &rx, ECTIMEOUTRXM);

		struct stepper_24v_t stepper_state;

		memset(&stepper_state, 0, sizeof(struct stepper_24v_t));

		stepper_state.stat_cont0.bit.mbx_mode = 1;

		ec_TxPDO(4,
#endif

/*
		//this code was to use SDO objects to read from inputs, inputs aren't connected to anything though? xP
		int psize = 2;
		char data[3];
		printf("here1\n");
		fflush(stdout);
		//ec_SDOread(4, 0x7040, 0x01, FALSE, &psize, (void *)&data, EC_TIMEOUTRXM);
		ec_SDOread(4,0x6000, 0x01, FALSE, &psize, (void *)&data, EC_TIMEOUTRXM);
		printf("here2\n");
		fflush(stdout);
		printf("size of data returned is: %d\n", psize);
		printf("result is: %u", (uint8)data);*/

/*		for (int i=0; i<=4; i++) {
			printf("name: %s\n", ec_slave[i].name);
			uint8 *inval = ec_slave[i].inputs;
			uint8 *outval = ec_slave[i].outputs;		
			//printf("Val: %u\n",	*val);
			printf("inpointer: %p\n", inval);
			printf("outpointer: %p\n", outval);

		}*/
		/*for (int i=0; i<10; i++) {
			printf("iomap%d: %d\n", i, IOmap[i]);
			printf("ptr: %p\n", &(IOmap[i]));
		}*/
#ifdef DUMP_IOMAP_24VSTEPPER
		uint8 *ptr = ec_slave[4].inputs;
		for (int i=0; i<42; i++) {
			printf("data%d: %u\t", i, ptr[i]);
		}
		printf("\n");
#endif /* DUMP_IOMAP_24VSTEPPER */
#ifdef MAGNET_COUNTER
		//printf("here\n");
		if (counter > 10000) {
			magnet_direction = (magnet_direction == 1) ? 0 : 1;
			*(ec_slave[3].outputs) = (uint8) magnet_direction;
			printf("set magnet direction to: %u\n", *(ec_slave[3].outputs) );
			if (magnet_direction == 1)
				IOmap[0x0029] = (uint8) 0x0F;
			else
				IOmap[0x0029] = (uint8) 0x00;
			
#ifdef PRINT10000
			uint8 *ptr = ec_slave[4].inputs;
			for (int i=0; i<42; i++) 
				printf("data%d: %u\t", i, ptr[i]);
#elif ALTPRINT10000
			for (int i=0; i<60; i++)
				printf("data%d: %u\t", i, IOmap[i]);
#endif
		}		
#endif /* MAGNET_COUNTER */
#ifdef COUNTER
		if (counter > 10000)
			counter = 0;

		counter++;
#endif
#ifdef STEPPER_SII	
		uint8 nSM, iSM, tSM;
		int outputs_bo, inputs_bo, rdl;
		//*((ec_slave[4].outputs)+C1_OFFSET) = CONTROL_BIT_ENABLE | CONTROL_BIT_STOP2_N | CONTROL_BIT_START;
		int wkc = ec_SDOread(4, ECT_SDO_SMCOMMTYPE, 0x00, FALSE, &rdl, &nSM, EC_TIMEOUTRXM);
		/* positive result from slave ? */
		if ((wkc > 0) && (nSM > 2)) {
			nSM--;

			if (nSM > EC_MAXSM)
				nSM = EC_MAXSM;

			/* iterate for every SM type defined */
			for (iSM = 2; iSM <= nSM; iSM++) {
				rdl = sizeof(tSM); tSM=0;
				/* read SyncManager Communication Type */
				wkc = ec_SDOread(4, ECT_SDO_SMCOMMTYPE, iSM + 1, FALSE, &rdl, &tSM, EC_TIMEOUTRXM);
				if (wkc > 0 ) {
					if (tSM == 3) { //outputs
						/* read the assign RXPDO */
						printf("SM%1d outputs\n addr b index: sub bitl data_type name\n", iSM);
						si_PDOassign(4, ECT_SDO_PDOASSIGN + iSM, (int)(ec_slave[4].outputs - (uint8 *)&IOmap[0]), outputs_bo);
					}
				
					if (tSM == 4) //inputs
					{
						/* read the assign TXPDO */
						printf("SM:%1d inputs\n addr b index: sub bitl data_type name\n", iSM);
						si_PDOassign(4, ECT_SDO_PDOASSIGN + iSM, (int)(ec_slave[4].outputs - (uint8 *)&IOmap[0]), inputs_bo);
					}
				}
			}
		}

		uint8 result = IOmap[STATUS_READ_OFFSET];
		//uint8 result = *(ec_slave[4].outputs+STATUS_READ_OFFSET);
		printf("result: %u\n", result);
#endif /* STEPPER_SII */
	}

/*	// list detected slaves by name
	for (int i=1; i<=ec_slavecount; i++) {
		printf("Slave %d: %s\n", i, ec_slave[i].name);
	}*/

	ec_close();	
}

void ethercat_thread( void *ptr )
{
	struct timespec ts;
	struct timeval tp;
	int rc;
	int ht;
	int pcounter = 0;
	int64 cycletime;

	rc = pthread_mutex_lock(&mutex);
	rc = gettimeofday(&tp, NULL);

	/* Convert from timeval to timespec */
	ts.tv_sec = tp.tv_sec;
	ht = (tp.tv_usec/1000) + 1; /* round to nearest ms */
	ts.tv_nsec = ht * 1000000; /* cycletime in ns */
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

		//not needed?
		cycle_count++;

		if ((in_EK1002->counter != pcounter) && (streampos < (MAXSTREAM-1))) {
			/* 
			 *  check if there are timing problems in master
			 * if so, overwrite stream data so it shows up clearly in plots? 
			 */
			if (in_EK1002->counter > (pcounter+1)) {
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
		}
		
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
	while ( (c=getopt(argc, argv, "c:d:")) != -1) {
		switch (c) {
		case 'c':
			cycle_time = atoi(optarg);
			printf("setting cycle time to %d\n", cycle_time);
			break;
		case 'd':
			eth_dev = optarg;
			printf("setting ethernet device to %s\n", eth_dev);
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
	int iret1;
	int ctime;
	struct sched_param param;
	int policy = SCHED_OTHER;

	printf("SOEM (Simple Open EtherCAT Master)\nInput Test\n");

	process_cmd_opts(argc, argv);

	memset(&schedp, 0, sizeof(schedp));
	/* do not set priority above 49, otherwise sockets are starved */
	schedp.sched_priority = 30;
	sched_setscheduler(0, SCHED_FIFO, &schedp);

	usleep(1000);
		
	/* create RealTime thread */
	//iret1 = pthread_create( &ethercat_thread_handle, NULL, (void *) &ethercat_thread, (void *) ctime);

	/* set thread priority */
	//memset(&param, 0, sizeof(param));
	//param.sched_priority = 40;
	//iret1 = pthread_setschedparam(ethercat_thread_handle, policy, &param);

	/* start a cyclic routine? */
	input_test(argv[1]);

	schedp.sched_priority = 0;
	sched_setscheduler(0, SCHED_OTHER, &schedp);

	printf("End Program\n");

	return 0;
}				
