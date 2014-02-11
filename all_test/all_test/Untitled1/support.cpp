/** \file
 * \brief Adds support functions for TwinCAT3
 */

#include <TcServices.h>
#include "support.h"

/**
 * Write description of function here.
 * The function should follow these comments.
 * Use of "brief" tag is optional. (no point to it)
 * 
 * The function arguments listed with "param" will be compared
 * to the declaration and verified.
 * @param[in]     _inArg1 Description of first function argument.
 * @param[out]    _outArg2 Description of second function argument.
 * @param[in,out] _inoutArg3 Description of third function argument.
 * @return Description of returned value.
 */

/**
 * Function used to provide printf() functionality.
 * 
 * This function provides printf() functionality. The messages are written to the Visual studio error list. 
 * \warning This function required modifications to the TwinCAT3 CTcTrace class. The LogV method needed to be added. The class can be found in TcInterfaces.h
 * LogV method is:
 *	void LogV(PCHAR szFormat, va_list pArgs)
 *	{
 *		if (m_spSrv != NULL)
 *		{
 *			DWORD msgCtrlMask = ADSLOG_MSGTYPE_STRING;
 *			
 *			m_spSrv->TcReportLogV( msgCtrlMask, szFormat, pArgs );
 *		}
 *	}
 * \warning This function is supported by a printf() macro (in support.h) which allows calling printf without needing to visibly pass m_Trace. m_Trace still needs to be in local scope.
 * @param[in]	m_Trace reference to main module files CTcTrace object (m_Trace)
 * @param[in]	fmt String format, just like printf()
 * @param[in]	... additional arguments, just like printf()
 */
void printf_m_Trace(CTcTrace &m_Trace, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);

	m_Trace.LogV((PCHAR) fmt, args);

	va_end(args);
}