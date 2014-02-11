/* wago_steppers.h
 * this file defines the structure that is used for accessing the stepper process image
 * it also contains methods for modifying the structure
 * this header file is designed to work with both SOEM and TwinCAT3 
 * 
 * written by: Jonathan Clapson (5 FEB 2014)
 */

#ifdef _WIN32
#pragma once
#endif

#ifndef __WAGO_STEPPERS_H__
#define __WAGO_STEPPERS_H__

#define WAGO_ERR_SUCCESS 0
#define WAGO_ERR_TERMINATE_NOT_SET -1
#define WAGO_ERR_POSITIONING_NOT_SET -2
#define WAGO_ERR_SETUP_NOT_SET -3
#define WAGO_ERR_POSITION_TOO_LARGE -4

#define WAGO_NUM_STEPPERS 3

enum {
	WAGO_OUTPUT_SPACE = 0,
	WAGO_INPUT_SPACE,
	WAGO_LENGTH_SPACE
};

#ifndef _WIN32
#include <pthread.h>
extern pthread_mutex_t io_mutex;
#endif

/* io structures */
#ifdef _WIN32
#include "stdint.h"
__pragma( pack(push, 1) )
struct wago_stepper_t {
#else
#include <stdint.h>
struct __attribute__((__packed__)) wago_stepper_t {
#endif /*_WIN32 */

	union {
		uint8_t value;
		struct {
			uint8_t reserved : 5;
			uint8_t mbx_mode : 1;
			uint8_t error : 1; /* this is read only */
			uint8_t reserved2 : 1;
		} bit;
	} stat_cont0;

	uint8_t reserved;
	
	union {
		struct {
			uint8_t velocity_lbyte;
			uint8_t velocity_hbyte;
			uint8_t acceleration_lbyte;
			uint8_t acceleration_hbyte;
			uint8_t position_lbyte;
			uint8_t position_mbyte;
			uint8_t position_hbyte;
		} positioning;
		struct {
			uint8_t opcode;
			uint8_t control;
			uint8_t mail[4];
			uint8_t reserved;
		} mailbox;
	} message;
	
	
	uint8_t stat_cont3;
	union {
		uint8_t value;
		struct {
			uint8_t on_target : 1;
			uint8_t busy : 1;
			uint8_t standstill : 1;
			uint8_t on_speed : 1;
			uint8_t direction : 1;
			uint8_t reference_ok : 1;
			uint8_t precalc_ack : 1;
			uint8_t error : 1;
		} status_bits;

		struct {
			uint8_t to_be_defined : 8;
		} control_bits;
	} stat_cont2;
	union {
		uint8_t value;
		struct {
			uint8_t enable : 1;
			uint8_t stop2_n : 1;
			uint8_t start : 1;
			uint8_t m_positioning : 1;
			uint8_t m_program : 1;
			uint8_t m_reference : 1;
			uint8_t m_jog : 1;
			uint8_t m_drive_by_mbx : 1;
		} bit;
	} stat_cont1;	
};
#ifdef _WIN32
__pragma( pack(pop) )
#endif /* _WIN32 */ 

int wago_terminate_mode(struct wago_stepper_t *wago_steppers[WAGO_NUM_STEPPERS][WAGO_LENGTH_SPACE], int device);
int wago_confirm_terminate_mode(struct wago_stepper_t *wago_steppers[WAGO_NUM_STEPPERS][WAGO_LENGTH_SPACE], int device);

int wago_set_setup_mode(struct wago_stepper_t *wago_steppers[WAGO_NUM_STEPPERS][WAGO_LENGTH_SPACE], int device);
int wago_confirm_setup_mode(struct wago_stepper_t *wago_steppers[WAGO_NUM_STEPPERS][WAGO_LENGTH_SPACE], int device);

int wago_set_positioning_mode(struct wago_stepper_t *wago_steppers[WAGO_NUM_STEPPERS][WAGO_LENGTH_SPACE], int device);
int wago_confirm_positioning_mode(struct wago_stepper_t *wago_steppers[WAGO_NUM_STEPPERS][WAGO_LENGTH_SPACE], int device);

int wago_set_velocity_limit(struct wago_stepper_t *wago_steppers[WAGO_NUM_STEPPERS][WAGO_LENGTH_SPACE], int device, uint16_t max_vel);
int wago_set_acceleration_limit(struct wago_stepper_t *wago_steppers[WAGO_NUM_STEPPERS][WAGO_LENGTH_SPACE], int device, uint16_t max_accel);

int wago_set_position(struct wago_stepper_t *wago_steppers[WAGO_NUM_STEPPERS][WAGO_LENGTH_SPACE], int device, uint32_t move_coord);
int wago_enable_motor(struct wago_stepper_t *wago_steppers[WAGO_NUM_STEPPERS][WAGO_LENGTH_SPACE], int device);

#endif /* __WAGO_STEPPERS_H__ */
