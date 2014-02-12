/** \file
 * \brief
 * Base TwinCAT3 module
 *
 * \warning This program has a bit of a quirky printf(). This printf is one I have created, 
 * implementation can be found in support.h
 * The printf implementation requires that a reference to the m_Trace object (Type CTcTrace)
 * available in this class is locally available. This is why the m_Trace object is passed to state_machine
 * (it allows using printf() via CTcTrace inside of the state_machine function).
 */

#include "TcPch.h"
#pragma hdrstop

#include "Module1.h"

#include "TcInterfaces.h"

/* My Includes */
/* for some reason visual studio 2010 doesn't include this?!
 * I got a copy from here: https://code.google.com/p/msinttypes/downloads/list and put in current directory 
 */
#include "stdint.h" 
#include "support.h" /* provides printf() */
#include "wago_steppers.h" /* shared code with soem */
#include "state_machine.h" /* shared code with soem */

#ifdef _DEBUG
#define new DEBUG_NEW
#endif
DEFINE_THIS_FILE()

///////////////////////////////////////////////////////////////////////////////
// CModule1
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Collection of interfaces implemented by module CModule1
BEGIN_INTERFACE_MAP(CModule1)
	INTERFACE_ENTRY_ITCOMOBJECT()
	INTERFACE_ENTRY(IID_ITcADI, ITcADI)
	INTERFACE_ENTRY(IID_ITcWatchSource, ITcWatchSource)
///<AutoGeneratedContent id="InterfaceMap">
	INTERFACE_ENTRY(IID_ITcCyclic, ITcCyclic)
///</AutoGeneratedContent>
END_INTERFACE_MAP()

IMPLEMENT_ITCOMOBJECT(CModule1)
IMPLEMENT_ITCOMOBJECT_SETSTATE_LOCKOP2(CModule1)
IMPLEMENT_ITCADI(CModule1)
IMPLEMENT_ITCWATCHSOURCE(CModule1)


///////////////////////////////////////////////////////////////////////////////
// Set parameters of CModule1 
BEGIN_SETOBJPARA_MAP(CModule1)
	SETOBJPARA_DATAAREA_MAP()
///<AutoGeneratedContent id="SetObjectParameterMap">
	SETOBJPARA_VALUE(PID_TcTraceLevel, m_TraceLevelMax)
	SETOBJPARA_VALUE(PID_Module1Parameter, m_Parameter)
	SETOBJPARA_ITFPTR(PID_Ctx_TaskOid, m_spCyclicCaller)
///</AutoGeneratedContent>
END_SETOBJPARA_MAP()

///////////////////////////////////////////////////////////////////////////////
// Get parameters of CModule1 
BEGIN_GETOBJPARA_MAP(CModule1)
	GETOBJPARA_DATAAREA_MAP()
///<AutoGeneratedContent id="GetObjectParameterMap">
	GETOBJPARA_VALUE(PID_TcTraceLevel, m_TraceLevelMax)
	GETOBJPARA_VALUE(PID_Module1Parameter, m_Parameter)
	GETOBJPARA_ITFPTR(PID_Ctx_TaskOid, m_spCyclicCaller)
///</AutoGeneratedContent>
END_GETOBJPARA_MAP()

///////////////////////////////////////////////////////////////////////////////
// Get watch entries of CModule1
BEGIN_OBJPARAWATCH_MAP(CModule1)
	OBJPARAWATCH_DATAAREA_MAP()
///<AutoGeneratedContent id="ObjectParameterWatchMap">
///</AutoGeneratedContent>
END_OBJPARAWATCH_MAP()

///////////////////////////////////////////////////////////////////////////////
// Get data area members of CModule1
BEGIN_OBJDATAAREA_MAP(CModule1)
///<AutoGeneratedContent id="ObjectDataAreaMap">
	OBJDATAAREA_VALUE(0, m_Inputs)
	OBJDATAAREA_VALUE(1, m_Outputs)
///</AutoGeneratedContent>
END_OBJDATAAREA_MAP()


///////////////////////////////////////////////////////////////////////////////
CModule1::CModule1()
	: m_Trace(m_TraceLevelMax, m_spSrv)
	, m_TraceLevelMax(tlAlways)
	, m_counter(0)
{
	memset(&m_Parameter, 0, sizeof(m_Parameter)); 
	memset(&m_Inputs, 0, sizeof(m_Inputs)); 
	memset(&m_Outputs, 0, sizeof(m_Outputs)); 
}

///////////////////////////////////////////////////////////////////////////////
CModule1::~CModule1() 
{
}


///////////////////////////////////////////////////////////////////////////////
// State Transitions 
///////////////////////////////////////////////////////////////////////////////
IMPLEMENT_ITCOMOBJECT_SETOBJSTATE_IP_PI(CModule1)


/**
 * State transition from PREOP to SAFEOP
 * 
 * Code in this method will run when the slave goes from PREOP to SAFEOP
 * Beckhoff suggests this function is used to initialise input parameters and allocate memory
 * I use this function to set up useful access pointers to the wago stepper motors
 * I suggest the other devices also be added here
 *
 * @param[in] pInitData Absolutely no idea what this is
 * @return No idea what this is ?some sort of error indicator?
 */

HRESULT CModule1::SetObjStatePS(PTComInitDataHdr pInitData)
{
	m_Trace.Log(tlVerbose, FENTERA);
	HRESULT hr = S_OK;
	IMPLEMENT_ITCOMOBJECT_EVALUATE_INITDATA(pInitData);

	// TODO: Add initialization code
	
	/* 
	 * map wago steppers to the structure of them as used with soem. 
	 * This object is declared in state_machine.c (its extern) 
	 */
	wago_steppers[0][WAGO_INPUT_SPACE] = (struct wago_stepper_t*) &(m_Inputs.wago_stepper_inputs0);
	wago_steppers[1][WAGO_INPUT_SPACE] = (struct wago_stepper_t*) &(m_Inputs.wago_stepper_inputs1);
	wago_steppers[2][WAGO_INPUT_SPACE] = (struct wago_stepper_t*) &(m_Inputs.wago_stepper_inputs2);

	wago_steppers[0][WAGO_OUTPUT_SPACE] = (struct wago_stepper_t*) &(m_Outputs.wago_stepper_outputs0);
	wago_steppers[1][WAGO_OUTPUT_SPACE] = (struct wago_stepper_t*) &(m_Outputs.wago_stepper_outputs1);
	wago_steppers[2][WAGO_OUTPUT_SPACE] = (struct wago_stepper_t*) &(m_Outputs.wago_stepper_outputs2);

	m_Trace.Log(tlVerbose, FLEAVEA "hr=0x%08x", hr);
	return hr;
}

/**
 * State transition from SAFEOP to OP
 * 
 * Code in this method will run as slaves switch from SAFEOP to OP
 * Beckhoff suggests this function is for registering with other TwinCAT objects
 * @return No idea what this is ?some sort of error indicator?
 */
HRESULT CModule1::SetObjStateSO()
{
	m_Trace.Log(tlVerbose, FENTERA);
	HRESULT hr = S_OK;

	// TODO: Add any additional initialization


	// If following call is successful the CycleUpdate method will be called, 
	// eventually even before method has been left.
	hr = FAILED(hr) ? hr : AddModuleToCaller(); 

	// Cleanup if transition failed at some stage
	if ( FAILED(hr) )
	{
		RemoveModuleFromCaller(); 
	}

	m_Trace.Log(tlVerbose, FLEAVEA "hr=0x%08x", hr);
	return hr;
}

/**
 * State transition from OP to SAFEOP
 *
 * Code in this method will run when slaves switch from OP to SAFEOP
 * @return No idea what this is ?some sort of error indicator?
 */
HRESULT CModule1::SetObjStateOS()
{
	m_Trace.Log(tlVerbose, FENTERA);

	HRESULT hr = S_OK;

	RemoveModuleFromCaller(); 

	// TODO: Add any additional deinitialization

	m_Trace.Log(tlVerbose, FLEAVEA "hr=0x%08x", hr);
	return hr;
}

/**
 * State transition from SAFEOP to PREOP
 * 
 * Code in this method will run when slaves switch from SAFEOP to PREOP
 * @return No idea what this is ?some sort of error indicator?
 */
HRESULT CModule1::SetObjStateSP()
{
	HRESULT hr = S_OK;
	m_Trace.Log(tlVerbose, FENTERA);

	// TODO: Add deinitialization code

	m_Trace.Log(tlVerbose, FLEAVEA "hr=0x%08x", hr);
	return hr;
}

/**
 * Cyclic Main Function
 *
 * This function is effectively the 'main' function of a TwinCAT3 based program
 * It is run cyclically at a timing specified in the System,Tasks,Taskname settings
 * Currently this function notifies when the state of the sick laser sensors changes
 * This function also runs a state machine which is used to control the WAGO Stepper motors. 
 * The state machine does not work correctly as there seems to be a mismatch in the size of the data
 * the wago devices report and the size that TwinCAT3 detects.
 * @param[in,out] ipTask Unknown
 * @param[in,out]    ipCaller Unknown
 * @param[in] context Unknown
 * @return No idea what this is ?some sort of error indicator?
 */
///<AutoGeneratedContent id="ImplementationOf_ITcCyclic">
HRESULT CModule1::CycleUpdate(ITcTask* ipTask, ITcUnknown* ipCaller, ULONG_PTR context)
{
	HRESULT hr = S_OK;
	static uint8_t current_sick;
	static uint8_t last_sick = 0;

	/* notify about changes to state of sick sensors */
	current_sick = m_Inputs.EK1002_BITS;
	
	if (current_sick != last_sick)
		printf("sick: changed to %x from %x\n", current_sick, last_sick);
	last_sick = current_sick;

	/* update the state_machine, also causes updates of io's */
	state_machine(m_Trace);

	return hr;
}
///</AutoGeneratedContent>

/**
 * I believe this function is called when the module is connected to the task
 *
 * I would imagine there aren't very many cases where this would need to be modified
 * @return No idea what this is ?some sort of error indicator?
 */
HRESULT CModule1::AddModuleToCaller()
{
	m_Trace.Log(tlVerbose, FENTERA);

	HRESULT hr = S_OK;
	if ( m_spCyclicCaller.HasOID() )
	{
		if ( SUCCEEDED_DBG(hr = m_spSrv->TcQuerySmartObjectInterface(m_spCyclicCaller)) )
		{
			if ( FAILED(hr = m_spCyclicCaller->AddModule(m_spCyclicCaller, THIS_CAST(ITcCyclic))) )
			{
				m_spCyclicCaller = NULL;
			}
		}
	}
	else
	{
		hr = ADS_E_INVALIDOBJID; 
		SUCCEEDED_DBGT(hr, "Invalid OID specified for caller task");
	}

	m_Trace.Log(tlVerbose, FLEAVEA "hr=0x%08x", hr);
	return hr;
}

/**
 * I believe this function is called when the module is disconnected from the task
 *
 * I would imagine there aren't very many cases where this would need to be modified
 * @return No idea what this is ?some sort of error indicator?
 */
VOID CModule1::RemoveModuleFromCaller()
{
	m_Trace.Log(tlVerbose, FENTERA);

	if ( m_spCyclicCaller )
	{
		m_spCyclicCaller->RemoveModule(m_spCyclicCaller);
	}
	m_spCyclicCaller	= NULL;

	m_Trace.Log(tlVerbose, FLEAVEA);
}

