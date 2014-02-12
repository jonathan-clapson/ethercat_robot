/* state_machine.h
 * this file defines the state machine used for controlling the system
 * written by: Jonathan Clapson (10 FEB 2014)
 */
#include "wago_steppers.h"

#ifdef _WIN32
int state_machine(CTcTrace &m_Trace);
#else
int state_machine();
#endif

extern struct wago_stepper_t *wago_steppers[WAGO_NUM_STEPPERS][WAGO_LENGTH_SPACE];
