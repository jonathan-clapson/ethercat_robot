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
#include "ethercatdc.h"
#include "ethercatprint.h"

#define NSEC_PER_SEC 1000000000

char IOmap[4096];

/* args */
char *eth_dev = "eth0";
uint32 cycle_time = 1000;

/*typedef struct PACKED
{
	uint8 chan1;
	uint8 chan2;
} in_EK1002t;*/

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
	int64 sec, nsec;
	nsec = addtime % NSEC_PER_SEC;
	sec = (addtime - nsec) / NSEC_PER_SEC;
	ts->tv_sec += sec;
	ts->tv_nsec += nsec;
	if ( ts-> tv_sec > NSEC_PER_SEC )
	{
		nsec = ts->tv_nsec % NSEC_PER_SEC;
		ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
		ts->tv_nsec = nsec;
	}
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

	ec_readstate();
	
	ec_send_processdata();
	int wkc = ec_receive_processdata(EtherCAT_TIMEOUT);

	printf("got wkc of: %d\n", wkc);


	for (int i=0; i<=3; i++) {
		ec_writestate(i);

		ec_statecheck(i, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
		printf("Slave %d operational\n", i);
	}

	int counter = 0;
	int magnet_direction = 0;
	while (1) {
		ec_send_processdata();
		ec_receive_processdata(EtherCAT_TIMEOUT);
		
/*		for (int i=0; i<4; i++) {
			printf("name: %s\n", ec_slave[i].name);
			uint8 *inval = ec_slave[i].inputs;
			uint8 *outval = ec_slave[i].outputs;		
			//printf("Val: %u\n",	*val);
			printf("inpointer: %p\n", inval);
			printf("outpointer: %p\n", outval);

		}
		for (int i=0; i<10; i++) {
			printf("iomap%d: %d\n", i, IOmap[i]);
			printf("ptr: %p\n", &(IOmap[i]));
		}*/
//		printf("val: %u\n", *(ec_slave[2].inputs));
		counter ++;
		if (counter > 10000) {
			magnet_direction = (magnet_direction == 1) ? 0 : 1;
			counter = 0;
			*(ec_slave[3].outputs) = (uint8) magnet_direction;
			printf("set magnet direction to: %u\n", *(ec_slave[3].outputs) );
		}
		//printf("iomap: %p\n", IOmap);
		usleep(100);
	}

	// list detected slaves by name
	for (int i=1; i<=ec_slavecount; i++) {
		printf("Slave %d: %s\n", i, ec_slave[i].name);
	}

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
