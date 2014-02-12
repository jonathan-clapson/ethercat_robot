/** \file
 * \brief State machine used for controlling the system
 *
 * \warning Building this file in visual studio requires setting the /TP build option 
 * (Project->Properties->c/c++->commandline->additional options)
 * This forces this file to be compiled as c++ as otherwise a beckhoff header gets pulled in
 * and treated as c which causes a c++ struct (method overloads) to fail to compile
 * This file is used both in SOEM and TwinCAT3 (which is why there are #ifdef's everywhere :S The ifdefs should really be detecting visual studio vs gcc rather than _win32 and linux though. This needs to be fixed.
 *
 */

#ifdef _WIN32 /* Windows - TwinCAT3 Includes */
#include "stdint.h"
#include "support.h" //fake printf()
#else /* Linux - SOEM Includes */
#include <stdint.h>
#include <stdio.h>
#endif

#include "error.h"
#include "state_machine.h"
#include "wago_steppers.h"

struct wago_stepper_t *wago_steppers[WAGO_NUM_STEPPERS][WAGO_LENGTH_SPACE];

/**
 * State machine
 *
 * The state machine currently only runs the wago stepper motors, it appears to run correctly in TwinCAT3
 * It definately runs correctly in soem. 
 * It would probably be a nice solution to create a queue on which to register these instructions instead.
 * This would significantly simplify the code.
 * Currently debug messages are printed whenever the state changes
 * @param[in]     m_Trace A reference to TwinCAT3's m_Trace object, needed for printf() in TwinCAT3. This argument is not present under SOEM.
 * @return Returns ERR_SUCCESS if the state machine is still executing and ERR_STATE_MACHINE_STOPPED once execution is complete.
 */
#ifdef _WIN32
int state_machine(CTcTrace &m_Trace) {
#else
int state_machine() {
#endif
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
	
	static enum states current_state = set_terminate_operating_mode;
	static enum states last_state = stop;
	uint32_t move_coord = 64*5*200; /* 64 microsteps * 5:1 gear ratio * 360degrees/1.8degreesperstep = 1 full rotation */
	int confirmed;

	switch (current_state) {
	case set_terminate_operating_mode:
		printf("terminating existing\n");

		for (int i=0; i<3; i++){
			wago_terminate_mode(wago_steppers, i);
			/*wago_steppers[i][0]->stat_cont1.bit.enable = 0;
			wago_steppers[i][0]->stat_cont1.bit.stop2_n = 0;
			wago_steppers[i][0]->stat_cont1.bit.start = 0;*/
		}

		last_state = set_terminate_operating_mode;
		current_state = confirm_terminate_operating_mode;
		break;
	case confirm_terminate_operating_mode:
		printf("Terminate Operating Mode\n");
		/*confirmed = 1;
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
	
		}*/
		confirmed = 1;
		for (int i=0; i<3; i++) {
				if (wago_confirm_terminate_mode(wago_steppers, i) < 0) {
					confirmed = 0;
					break;
				}
		}

		last_state = confirm_terminate_operating_mode;
		if (confirmed) 
			current_state=set_setup_mode;
		break;
	case set_setup_mode:
		printf("set setup mode\n");
		for (int i=0; i<3; i++) {
			wago_set_setup_mode(wago_steppers, i);
		}

		last_state = set_setup_mode;
		current_state = confirm_setup_mode;
		break;
	case confirm_setup_mode:
		if (last_state != current_state)
			printf("Confirm Setup Mode\n");
		
		confirmed = 1;
		for (int i=0; i<3; i++) {
			if (wago_confirm_setup_mode(wago_steppers, i) < 0) {
				confirmed = 0;
				break;
			}
/*			if (wago_steppers[i][WAGO_INPUT_SPACE]->stat_cont1.bit.enable != 1) {
				printf("%d: Trying to set enable to: %d Enable is: %d!\n", 
					i, 
					wago_steppers[i][WAGO_OUTPUT_SPACE]->stat_cont1.bit.enable, 
					wago_steppers[WAGO_INPUT_SPACE][1]->stat_cont1.bit.enable
				);
				confirmed = 0;
			}
			if (wago_steppers[i][WAGO_INPUT_SPACE]->stat_cont1.bit.stop2_n != 1) {
				printf("%d: Trying to set stop to: %d Stop is: %d!\n", 
					i, 
					wago_steppers[i][WAGO_OUTPUT_SPACE]->stat_cont1.bit.stop2_n, 
					wago_steppers[i][WAGO_INPUT_SPACE]->stat_cont1.bit.stop2_n
				);
				confirmed = 0;
			}
			if (wago_steppers[i][WAGO_INPUT_SPACE]->stat_cont1.bit.start != 0) {
				printf("%d: Trying to set start to: %d Start is: %d!\n", 
					i, 
					wago_steppers[i][WAGO_OUTPUT_SPACE]->stat_cont1.bit.start, 
					wago_steppers[i][WAGO_INPUT_SPACE]->stat_cont1.bit.start
				);
				confirmed = 0;
			}*/
		}
		last_state = confirm_setup_mode;
		if (confirmed)
			current_state=set_positioning_mode;
		break;
	case set_positioning_mode:
		printf("set positioning mode\n");
		for (int i=0; i<3; i++) {
			wago_set_positioning_mode(wago_steppers, i);
		}

		last_state = set_positioning_mode;
		current_state = confirm_positioning_mode;
		break;
	case confirm_positioning_mode:
		if (last_state != current_state)
			printf("confirm positioning mode\n");
		confirmed = 1;
		for (int i=0; i<3; i++) {
			if (wago_confirm_positioning_mode(wago_steppers, i) < 0) {
				confirmed = 0;
				break;
			}
		}
		/*printf("positioning mode active? s1:%s s2:%s s3:%s\n",
			(wago_steppers[0][1]->stat_cont1.bit.m_positioning)?"y":"n",
			(wago_steppers[1][1]->stat_cont1.bit.m_positioning)?"y":"n",	
			(wago_steppers[2][1]->stat_cont1.bit.m_positioning)?"y":"n"
		);*/
		
		last_state = confirm_positioning_mode;
		if (confirmed)	
			current_state = set_position;
		break;
	case set_position:
		if (last_state != current_state)
			printf("set position\n");
		for (int i=0; i<3; i++) {
			printf("m_positioning? %d\n", wago_steppers[i][1]->stat_cont1.bit.m_positioning);
			
			wago_set_acceleration_limit(wago_steppers, i, (uint16_t) 5000);

			wago_set_velocity_limit(wago_steppers, i, (uint16_t) 5000);
			
			wago_set_position(wago_steppers, i, (uint32_t) move_coord);

			wago_enable_motor(wago_steppers, i);
		}
		last_state = set_position;
		current_state = check_position;
		break;
	case check_position:
		if (last_state != current_state)
			printf("check position\n");
		if (wago_steppers[0][1]->stat_cont2.status_bits.on_target){
			printf("Reached destination!\n");
			current_state = stop;
		}
		last_state = check_position;
		break;

	case stop:
		if (last_state != current_state)
			printf("positioning mode: %d\n", wago_steppers[0][1]->stat_cont1.bit.m_positioning);

		last_state = stop;
		return ERR_STATE_MACHINE_STOPPED;
		break;
	default:
		printf("ERROR: should not be in this state\n");
		break;
	}
	return ERR_SUCCESS;
}
