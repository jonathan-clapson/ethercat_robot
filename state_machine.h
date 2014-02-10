/* state_machine.h
 * this file defines the state machine used for controlling the system
 * written by: Jonathan Clapson (10 FEB 2014)
 */
#include "wago_steppers.h"

#ifdef _WIN32
void state_machine(CTcTrace &m_Trace);
#else
void state_machine();
#endif

extern struct wago_stepper_t *wago_steppers[3][2];