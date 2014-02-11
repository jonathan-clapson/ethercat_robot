/** \file
 * \brief this file defines the functions used for accessing the stepper process image
 * 
 * All functions in this file need sanity checking on the device number
 * \warning Building this file in visual studio requires setting the /TP build option 
 * (Project->Properties->c/c++->commandline->additional options)
 * This forces this file to be compiled as c++ as otherwise a beckhoff header gets pulled in
 * and treated as c which causes a c++ struct (method overloads) to fail to compile
 */

#include "wago_steppers.h"

/* Allow pthread locking using SOEM. Not needed when using TwinCAT3 */
#ifdef _WIN32
#define IO_LOCK NULL
#define IO_UNLOCK NULL
#else
#define IO_LOCK pthread_mutex_lock(&io_mutex)
#define IO_UNLOCK pthread_mutex_unlock(&io_mutex)
pthread_mutex_t io_mutex = PTHREAD_MUTEX_INITIALIZER;
#endif

/**
 * Terminates the current operating mode
 * 
 * according to manual this is done by disabling control bits enable, stop2_n and start
 * @param[in,out]	wago_steppers Pointer to the array holding addresses of wago stepper motor io spaces
 * @param[in]		device The wago stepper to terminate, should start from 0 and go to WAGO_NUM_STEPPERS-1
 * @return WAGO_ERR_SUCCESS on success, WAGO error code on failure
 */
int wago_terminate_mode(struct wago_stepper_t *wago_steppers[WAGO_NUM_STEPPERS][WAGO_LENGTH_SPACE], int device)
{
	IO_LOCK;
	wago_steppers[device][WAGO_OUTPUT_SPACE]->stat_cont1.bit.enable = 0;
	wago_steppers[device][WAGO_OUTPUT_SPACE]->stat_cont1.bit.stop2_n = 0;
	wago_steppers[device][WAGO_OUTPUT_SPACE]->stat_cont1.bit.start = 0;
	IO_UNLOCK;

	return WAGO_ERR_SUCCESS;
}

/**
 * Verifies termination of the current operating mode
 * 
 * according to manual this is reported by status bits enable, stop2_n and start
 * @param[in,out]	wago_steppers Pointer to the array holding addresses of wago stepper motor io spaces
 * @param[in]		device The wago stepper to terminate, should start from 0 and go to WAGO_NUM_STEPPERS-1
 * @return WAGO_ERR_SUCCESS on success, WAGO error code on failure
 */
int wago_confirm_terminate_mode(struct wago_stepper_t *wago_steppers[WAGO_NUM_STEPPERS][WAGO_LENGTH_SPACE], int device)
{
	IO_LOCK;
	if (wago_steppers[device][WAGO_INPUT_SPACE]->stat_cont1.bit.enable != 0) {
		return WAGO_ERR_TERMINATE_NOT_SET;
	}
	if (wago_steppers[device][WAGO_INPUT_SPACE]->stat_cont1.bit.stop2_n != 0) {
		return WAGO_ERR_TERMINATE_NOT_SET;
	}
	if (wago_steppers[device][WAGO_INPUT_SPACE]->stat_cont1.bit.start != 0) {
		return WAGO_ERR_TERMINATE_NOT_SET;
	}
	IO_UNLOCK;
	return WAGO_ERR_SUCCESS;
}

/**
 * Configures a stepper motor to be ready for setting an operating mode
 * 
 * according to manual this is done by disabling control bit start and enabling control bits enable and stop2_n
 * the device must be in the 'terminate mode' state for this to work
 * @param[in,out]	wago_steppers Pointer to the array holding addresses of wago stepper motor io spaces
 * @param[in]		device The wago stepper to terminate, should start from 0 and go to WAGO_NUM_STEPPERS-1
 * @return WAGO_ERR_SUCCESS on success, WAGO error code on failure
 */
int wago_set_setup_mode(struct wago_stepper_t *wago_steppers[WAGO_NUM_STEPPERS][WAGO_LENGTH_SPACE], int device)
{
	IO_LOCK;
	wago_steppers[device][WAGO_OUTPUT_SPACE]->stat_cont1.bit.enable = 1;
	wago_steppers[device][WAGO_OUTPUT_SPACE]->stat_cont1.bit.stop2_n = 1;
	wago_steppers[device][WAGO_OUTPUT_SPACE]->stat_cont1.bit.start = 0;
	IO_UNLOCK;

	return WAGO_ERR_SUCCESS;
}
int wago_confirm_setup_mode(struct wago_stepper_t *wago_steppers[WAGO_NUM_STEPPERS][WAGO_LENGTH_SPACE], int device)
{
	IO_LOCK;
	if (wago_steppers[device][WAGO_INPUT_SPACE]->stat_cont1.bit.enable != 1) {
		return WAGO_ERR_SETUP_NOT_SET;
	}
	if (wago_steppers[device][WAGO_INPUT_SPACE]->stat_cont1.bit.stop2_n != 1) {
		return WAGO_ERR_SETUP_NOT_SET;
	}
	if (wago_steppers[device][WAGO_INPUT_SPACE]->stat_cont1.bit.start != 0) {
		return WAGO_ERR_SETUP_NOT_SET;
	}
	IO_UNLOCK;
	return WAGO_ERR_SUCCESS;
}

/**
 * Enables positioning mode
 * 
 * according to manual this is done by setting the control bit m_positioning
 * the device must be in the 'setup mode' state for this to work
 * @param[in,out]	wago_steppers Pointer to the array holding addresses of wago stepper motor io spaces
 * @param[in]		device The wago stepper to terminate, should start from 0 and go to WAGO_NUM_STEPPERS-1
 * @return WAGO_ERR_SUCCESS on success, WAGO error code on failure
 */
int wago_set_positioning_mode(struct wago_stepper_t *wago_steppers[WAGO_NUM_STEPPERS][WAGO_LENGTH_SPACE], int device)
{
	/* TODO: Perform checks to make sure the device is in a state from which positioning mode can be activated */
	IO_LOCK;
	wago_steppers[device][WAGO_OUTPUT_SPACE]->stat_cont1.bit.m_positioning = 1;
	IO_UNLOCK;

	return WAGO_ERR_SUCCESS;
}

int wago_confirm_positioning_mode(struct wago_stepper_t *wago_steppers[WAGO_NUM_STEPPERS][WAGO_LENGTH_SPACE], int device)
{
	//IO_LOCK;
	int ret;
	if (wago_steppers[device][WAGO_INPUT_SPACE]->stat_cont1.bit.m_positioning)
		ret = WAGO_ERR_SUCCESS;
	else
		ret = WAGO_ERR_POSITIONING_NOT_SET;
	//int ret = (wago_steppers[device][WAGO_INPUT_SPACE]->stat_cont1.bit.m_positioning) ? WAGO_ERR_SUCCESS : WAGO_ERR_POSITIONING_NOT_SET;
	//IO_UNLOCK;
	return ret;
}

/**
 * Sets the maximum allowable velocity of the stepper motor
 * 
 * This function is designed to work in positioning mode only. I have not verified whether it is correct for any other modes.
 * @param[in,out]	wago_steppers Pointer to the array holding addresses of wago stepper motor io spaces
 * @param[in]		device The wago stepper to terminate, should start from 0 and go to WAGO_NUM_STEPPERS-1
 * @param[in]		max_vel The maximum allowable velocity for the motor. Not sure of units
 * @return WAGO_ERR_SUCCESS on success, WAGO error code on failure
 */
int wago_set_velocity_limit(struct wago_stepper_t *wago_steppers[WAGO_NUM_STEPPERS][WAGO_LENGTH_SPACE], int device, uint16_t max_vel)
{
	/* TODO: there is a limit on this value, factor it in */

	IO_LOCK;
	wago_steppers[device][WAGO_OUTPUT_SPACE]->message.positioning.velocity_lbyte = (uint8_t) ((max_vel>>0)&0xFF);
	wago_steppers[device][WAGO_OUTPUT_SPACE]->message.positioning.velocity_hbyte = (uint8_t) ((max_vel>>8)&0xFF);
	IO_UNLOCK;

	return WAGO_ERR_SUCCESS;
}
/**
 * Sets the maximum allowable acceleration of the stepper motor
 * 
 * This function is designed to work in positioning mode only. I have not verified whether it is correct for any other modes.
 * @param[in,out]	wago_steppers Pointer to the array holding addresses of wago stepper motor io spaces
 * @param[in]		device The wago stepper to terminate, should start from 0 and go to WAGO_NUM_STEPPERS-1
 * @param[in]		max_accel The maximum allowable acceleration for the motor. Not sure of units
 * @return WAGO_ERR_SUCCESS on success, WAGO error code on failure
 */
int wago_set_acceleration_limit(struct wago_stepper_t *wago_steppers[WAGO_NUM_STEPPERS][WAGO_LENGTH_SPACE], int device, uint16_t max_accel)
{
	/* TODO: there is a limit on this value, factor it in */
	IO_LOCK;
	wago_steppers[device][WAGO_OUTPUT_SPACE]->message.positioning.acceleration_lbyte = (uint8_t) ((max_accel>>0)&0xFF);
	wago_steppers[device][WAGO_OUTPUT_SPACE]->message.positioning.acceleration_hbyte = (uint8_t) ((max_accel>>8)&0xFF);
	IO_UNLOCK;

	return WAGO_ERR_SUCCESS;
}

/**
 * Sets the 'goto' position
 * 
 * This function is designed to work in positioning mode only. I have not verified whether it is correct for any other modes.
 * @param[in,out]	wago_steppers Pointer to the array holding addresses of wago stepper motor io spaces
 * @param[in]		device The wago stepper to terminate, should start from 0 and go to WAGO_NUM_STEPPERS-1
 * @param[in]		move_coord The position to go to. 64microsteps * 5(5 to 1) gear ratio * 200 (360degrees/1.8degreesperstep) is one full rotation.
 * @return WAGO_ERR_SUCCESS on success, WAGO error code on failure
 */
int wago_set_position(struct wago_stepper_t *wago_steppers[WAGO_NUM_STEPPERS][WAGO_LENGTH_SPACE], int device, uint32_t move_coord)
{
	/* TODO: there are limits to this, at least it can't be greater than a 24bit number, check what actual limit is */
	if (move_coord > 0x00ffffff)
		return WAGO_ERR_POSITION_TOO_LARGE;
	IO_LOCK;
	wago_steppers[device][WAGO_OUTPUT_SPACE]->message.positioning.position_lbyte = (uint8_t) ((move_coord>>0)&0xff);
	wago_steppers[device][WAGO_OUTPUT_SPACE]->message.positioning.position_mbyte = (uint8_t) ((move_coord>>8)&0xff);
	wago_steppers[device][WAGO_OUTPUT_SPACE]->message.positioning.position_hbyte = (uint8_t) ((move_coord>>16)&0xff);
	IO_UNLOCK;

	return WAGO_ERR_SUCCESS;
}

/**
 * Enables the motor
 * 
 * This function tells the motor driver to move the motor to the position that was set.
 * @param[in,out]	wago_steppers Pointer to the array holding addresses of wago stepper motor io spaces
 * @param[in]		device The wago stepper to terminate, should start from 0 and go to WAGO_NUM_STEPPERS-1
 * @return WAGO_ERR_SUCCESS on success, WAGO error code on failure
 */
int wago_enable_motor(struct wago_stepper_t *wago_steppers[WAGO_NUM_STEPPERS][WAGO_LENGTH_SPACE], int device)
{
	IO_LOCK;
	wago_steppers[device][WAGO_OUTPUT_SPACE]->stat_cont1.bit.start = 1;
	IO_UNLOCK;

	return WAGO_ERR_SUCCESS;
}
