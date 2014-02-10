/* wago_steppers.cpp
 * this file defines the structure that is used for accessing the stepper process image
 * it also contains methods for modifying the structure
 * this header file is designed to work with both SOEM and TwinCAT3 
 * 
 * written by: Jonathan Clapson (5 FEB 2014)
 */

#include "wago_steppers.h"
#include "stdint.h"

int wago_terminate_mode(struct wago_stepper_t *wago_steppers[WAGO_NUM_STEPPERS][WAGO_LENGTH_SPACE], int device)
{
	wago_steppers[device][WAGO_OUTPUT_SPACE]->stat_cont1.bit.enable = 0;
	wago_steppers[device][WAGO_OUTPUT_SPACE]->stat_cont1.bit.stop2_n = 0;
	wago_steppers[device][WAGO_OUTPUT_SPACE]->stat_cont1.bit.start = 0;

	return WAGO_ERR_SUCCESS;
}
int wago_confirm_terminate_mode(struct wago_stepper_t *wago_steppers[WAGO_NUM_STEPPERS][WAGO_LENGTH_SPACE], int device)
{
	int confirmed = 1;
	if (wago_steppers[device][WAGO_INPUT_SPACE]->stat_cont1.bit.enable != 0) {
		return WAGO_ERR_TERMINATE_NOT_SET;
	}
	if (wago_steppers[device][WAGO_INPUT_SPACE]->stat_cont1.bit.stop2_n != 0) {
		return WAGO_ERR_TERMINATE_NOT_SET;
	}
	if (wago_steppers[device][WAGO_INPUT_SPACE]->stat_cont1.bit.start != 0) {
		return WAGO_ERR_TERMINATE_NOT_SET;
	}
	return WAGO_ERR_SUCCESS;
}

int wago_set_setup_mode(struct wago_stepper_t *wago_steppers[WAGO_NUM_STEPPERS][WAGO_LENGTH_SPACE], int device)
{
	wago_steppers[device][WAGO_OUTPUT_SPACE]->stat_cont1.bit.enable = 1;
	wago_steppers[device][WAGO_OUTPUT_SPACE]->stat_cont1.bit.stop2_n = 1;
	wago_steppers[device][WAGO_OUTPUT_SPACE]->stat_cont1.bit.start = 0;

	return WAGO_ERR_SUCCESS;
}
int wago_confirm_setup_mode(struct wago_stepper_t *wago_steppers[WAGO_NUM_STEPPERS][WAGO_LENGTH_SPACE], int device)
{
	if (wago_steppers[device][WAGO_INPUT_SPACE]->stat_cont1.bit.enable != 1) {
		return WAGO_ERR_SETUP_NOT_SET;
	}
	if (wago_steppers[device][WAGO_INPUT_SPACE]->stat_cont1.bit.stop2_n != 1) {
		return WAGO_ERR_SETUP_NOT_SET;
	}
	if (wago_steppers[device][WAGO_INPUT_SPACE]->stat_cont1.bit.start != 0) {
		return WAGO_ERR_SETUP_NOT_SET;
	}	
	return WAGO_ERR_SUCCESS;
}

int wago_set_positioning_mode(struct wago_stepper_t *wago_steppers[WAGO_NUM_STEPPERS][WAGO_LENGTH_SPACE], int device)
{
	/* TODO: Perform checks to make sure the device is in a state from which positioning mode can be activated */

	wago_steppers[device][WAGO_OUTPUT_SPACE]->stat_cont1.bit.m_positioning = 1;
	
	return WAGO_ERR_SUCCESS;
}
int wago_confirm_positioning_mode(struct wago_stepper_t *wago_steppers[WAGO_NUM_STEPPERS][WAGO_LENGTH_SPACE], int device)
{	
	return (wago_steppers[device][WAGO_INPUT_SPACE]->stat_cont1.bit.m_positioning) ? WAGO_ERR_SUCCESS : WAGO_ERR_POSITIONING_NOT_SET;
}

int wago_set_velocity_limit(struct wago_stepper_t *wago_steppers[WAGO_NUM_STEPPERS][WAGO_LENGTH_SPACE], int device, uint16_t max_vel)
{
	/* TODO: there is a limit on this value, factor it in */

	wago_steppers[device][WAGO_OUTPUT_SPACE]->message.positioning.velocity_lbyte = (uint8_t) ((max_vel>>0)&0xFF);
	wago_steppers[device][WAGO_OUTPUT_SPACE]->message.positioning.velocity_hbyte = (uint8_t) ((max_vel>>8)&0xFF);

	return WAGO_ERR_SUCCESS;
}
int wago_set_acceleration_limit(struct wago_stepper_t *wago_steppers[WAGO_NUM_STEPPERS][WAGO_LENGTH_SPACE], int device, uint16_t max_accel)
{
	/* TODO: there is a limit on this value, factor it in */

	wago_steppers[device][WAGO_OUTPUT_SPACE]->message.positioning.acceleration_lbyte = (uint8_t) ((max_accel>>0)&0xFF);
	wago_steppers[device][WAGO_OUTPUT_SPACE]->message.positioning.acceleration_hbyte = (uint8_t) ((max_accel>>8)&0xFF);

	return WAGO_ERR_SUCCESS;
}
int wago_set_position(struct wago_stepper_t *wago_steppers[WAGO_NUM_STEPPERS][WAGO_LENGTH_SPACE], int device, uint32_t move_coord)
{
	/* TODO: there are limits to this, at least it can't be greater than a 24bit number, check what actual limit is */
	if (move_coord > 0x00ffffff)
		return WAGO_ERR_POSITION_TOO_LARGE;
	wago_steppers[device][WAGO_OUTPUT_SPACE]->message.positioning.position_lbyte = (uint8_t) ((move_coord>>0)&0xff);
	wago_steppers[device][WAGO_OUTPUT_SPACE]->message.positioning.position_mbyte = (uint8_t) ((move_coord>>8)&0xff);
	wago_steppers[device][WAGO_OUTPUT_SPACE]->message.positioning.position_hbyte = (uint8_t) ((move_coord>>16)&0xff);

	return WAGO_ERR_SUCCESS;
}
int wago_enable_motor(struct wago_stepper_t *wago_steppers[WAGO_NUM_STEPPERS][WAGO_LENGTH_SPACE], int device)
{
	wago_steppers[device][WAGO_OUTPUT_SPACE]->stat_cont1.bit.start = 1;
	return WAGO_ERR_SUCCESS;
}