#include <string.h>
#include <stdbool.h>
#include "sdk_inc.h"
#include "options.h"
#include "modules/module_buttons.h"

#define _MOTOR_DEF_C
#include "motor_def.h"
#include "engine_bldc.h"
#include "motor_bldc.h"
#include "pid.h"

/* Debug variables */
uint16_t dbg_M1_speedinput 		;
uint16_t dbg_M2_speedinput 		;
uint16_t dbg_M1_speedtarget 	;
uint16_t dbg_M2_speedtarget		;
uint16_t dbg_M1_speedlimit 		;
uint16_t dbg_M2_speedlimit 		;
uint16_t dbg_M1_speedmeasured	;
uint16_t dbg_M2_speedmeasured 	;
uint16_t dbg_M1_motorpow 		;
uint16_t dbg_M2_motorpow 		;
uint16_t dbg_M1_tdevar 			;
uint16_t dbg_M2_tdevar 			;
uint16_t dbg_M1_current 		;
uint16_t dbg_M2_current 		;
uint16_t dbg_M1_currentlimit 	;
uint16_t dbg_M2_currentlimit 	;
uint16_t dbg_M1_pidtimeout 		;
uint16_t dbg_M2_pidtimeout 		;
uint16_t dbg_M1_currentraw 		;
uint16_t dbg_M2_currentraw 		;
uint16_t dbg_M1_currentlime3 	;
uint16_t dbg_M2_currentlime3 	;


extern int32_t g_VdcMotor;

engine_config_t g_engineconf = {
	.startopencoeff = 2200,			/* -> 1800 */
	.startclosecoeff = 2200,		/* -> 1800 */
	.opencoeff  = 0,
	.closecoeff = 0,
	.lowspeedcoeff = 0,
	.pidstartval= 500, //10
	.pidminval  = 0, //5
	.minopoutpwr = 0,
	.mincloutpwr = 0, //15,
	.startoppwr = 15,
	.startclpwr = 0, //15,
	.pidsresol	= 2,
	.pidtimeout = 25,
	.pidtimeoutstart = 100,
	.starttime  = 150,
	.costa_tstr = 500,
	.costa_tout = 500,
	.ilimtout	= 200,
	.pid_p 		= 5,        //18
	.pid_d 		= 0,//12.0,
	.pid_i 		= 1,      //0.3
	.speedcoeff = 16000,
};
//simulation pid parameter P = 0 and I = 0.05 with Ti = J/b = 0.12 (not good)

#define ENGINE_PID_TIMEOUT_START_MS		g_engineconf.pidtimeoutstart
#define ENGINE_PID_TIMEOUT_RUN_MS		g_engineconf.pidtimeout
#define ENGINE_PID_MAX_VARIATION		g_engineconf.pidstartval
#define ENGINE_PID_MIN_VARIATION		g_engineconf.pidminval
#define ENGINE_PID_SPEED_RESOLUTION		g_engineconf.pidsresol
#define ENGINE_PID_SPEED_COEFF			g_engineconf.speedcoeff

#define ENGINE_PID_FILTER_COEFF_Q		12
#define ENGINE_PID_FILTER_COEFF_N		8
#define ENGINE_PID_FILTER_COEFF_STARTCL	g_engineconf.startclosecoeff
#define ENGINE_PID_FILTER_COEFF_STARTOP	g_engineconf.startopencoeff
#define ENGINE_PID_FILTER_COEFF_LOWSPD	g_engineconf.lowspeedcoeff
#define ENGINE_PID_FILTER_COEFF_CLOSING	g_engineconf.closecoeff
#define ENGINE_PID_FILTER_COEFF_OPENING	g_engineconf.opencoeff
#define ENGINE_PID_MIN_OP_OUTPUT_POWER	g_engineconf.minopoutpwr
#define ENGINE_PID_MIN_CL_OUTPUT_POWER	g_engineconf.mincloutpwr
#define ENGINE_PID_MAX_OUTPUT_POWER		3000 //250
#define ENGINE_PID_MIN_OUTPUT_POWER		0
#define ENGINE_POWER_START_OPEN			g_engineconf.startoppwr
#define ENGINE_POWER_START_CLOSE		g_engineconf.startclpwr

#define ENGINE_REQSPEED_FILTER_COEFF_Q	16
#define ENGINE_REQSPEED_FILTER_COEFF_N	8
//#define ENGINE_REQSPEED_FILTER_COEFFMIN	100
//#define ENGINE_REQSPEED_FILTER_COEFFMAX	2000
#define ENGINE_REQSPEED_FILTER_COEFFDEF	800

//#define ENGINE_ILIM_TIMEOUT				g_engineconf.ilimtout

#define ENGINE_PID_P_WEIGHT				g_engineconf.pid_p
#define ENGINE_PID_D_WEIGHT				g_engineconf.pid_d
#define ENGINE_PID_I_WEIGHT				g_engineconf.pid_i

#define ENGINE_STARTTIME				g_engineconf.starttime
//#define ENGINE_COSTADETECT_STARTTIME	g_engineconf.costa_tstr
//#define ENGINE_COSTADETECT_TIMEOUT		g_engineconf.costa_tout
//#define ENGINE_COSTADETECT_SPEEDPERC	75

#if OPTION_ENGINE_TYPE_GPT
#define ENGINE_APPROACH_CURRENT_LIMIT	20000
#define ENGINE_FIRSTRACE_CURRENT_LIMIT	25000
#else
#define ENGINE_APPROACH_CURRENT_LIMIT	15000
#define ENGINE_FIRSTRACE_CURRENT_LIMIT	15000
#endif

//LOOPS
#define SPEED_LOOP
//#define CURRENT_LOOP
//#define OPEN_LOOP //remember also to remove the pid and set a voltage in foc_controller.c
typedef struct
{
	engine_callback_t eventCallback;
	ENGINE_RACE_MODE_T racemode;
	uint8_t status;

	uint8_t cfg_tipo_asta;			/* Tipologia asta */
	uint16_t cfg_costa_sens;			/* Sensibility costa detector: 25-100 */
	uint32_t encoder_race_steps;	/* Calibrated race encoder steps */

	uint16_t start_power;
	uint16_t gate_moved;
	uint16_t max_power;
	uint16_t elapsed_time;
	uint16_t rumpup_time;
	uint16_t stop_time;
	uint16_t start_time;
	int32_t start_stop_step;
	int32_t  max_stop_steps;

	int32_t filter_req_speed_acc;
	int32_t filter_req_speed;
	int32_t filter_req_speed_coeff;
	float req_speed;

	uint16_t pid_timeout;
	uint16_t pid_ticks;
	int32_t current_speed;
	uint16_t pid_maxvariation;
	uint16_t costa_speed_limit;
	uint16_t costa_speed_timeout;

	int32_t filter_req_power;
	int32_t filter_req_power_acc;
	int32_t filter_reqpow_coeff;

	int32_t filter_imot_acc;
	int32_t filtered_imot;

	int32_t filter_speed_acc;
	int32_t filtered_speed;

	uint16_t tout_costa_speed;
	uint16_t tout_battuta_speed;
	uint16_t overcurrent_ticks;

	bool encoder_fault;
	bool costa_detected;
	bool break_detected;
	bool pid_disabled;

	uint16_t pow_in;
	uint16_t pow_out;

} engine_t;

static engine_t g_engine;
static bool g_engTest;
float dbg_engine_pid;



//! Suspend and resume the time base
#define ENGINE_LOCK()				basetimer_suspend(tick1ms_work, NULL)
#define ENGINE_UNLOCK()				basetimer_resume(tick1ms_work, NULL)



/* Prototypes */
//static bool _monitor_costa(void);

static void _update_debug_vars(void)
{
	dbg_M1_speedinput = g_engine.req_speed;
	dbg_M1_speedtarget = g_engine.filter_req_speed;
	dbg_M1_speedlimit = g_engine.costa_speed_limit;
	dbg_M1_speedmeasured = g_engine.current_speed;
	dbg_M1_motorpow = MOTOR_GetImot();
	dbg_M1_current = MOTOR_GetImot();
	dbg_M1_currentlimit = MOTOR_GetILim();
	//dbg_M1_encstep = ENCODER_GetPulse();

	dbg_M2_speedinput = dbg_M1_speedinput;
	dbg_M2_speedtarget = dbg_M1_speedtarget;
	dbg_M2_speedlimit = dbg_M1_speedlimit;
	dbg_M2_speedmeasured = dbg_M1_speedmeasured;
	dbg_M2_motorpow = dbg_M1_motorpow;
	dbg_M2_current = dbg_M1_current;
	dbg_M2_currentlimit = dbg_M1_currentlimit;
	//dbg_M2_encstep = dbg_M1_encstep;
}



static int32_t _speed_control(uint16_t current_speed, uint16_t requested_speed, uint16_t current_power, uint16_t max_power)
{
	float p, ctrl, req, rpm;
	int32_t po;

	uint16_t minoutpwr = 0;
	if( g_engine.status == ENGINE_STATE_OPENING )
		minoutpwr = ENGINE_PID_MIN_OP_OUTPUT_POWER;
	else
		minoutpwr = ENGINE_PID_MIN_CL_OUTPUT_POWER;

//	if( g_engine.pid_disabled )
//	{
//		/* Open loop mode:
//		 *
//		 */
//		req = (float)requested_speed;
//		p = (req / 2400) * (ENGINE_PID_MAX_OUTPUT_POWER - minoutpwr) * 0.75 + minoutpwr;
//
//		/* Limiti di potenza applicabile */
//		po = (uint16_t)(p);
//		if( po > max_power)
//			po = max_power;
//		if ( po < minoutpwr)
//			po = minoutpwr;
//		return po;
//	}

	if( g_engine.overcurrent_ticks != 0 )
		return current_power;

	if( g_engTest )
	{
		req = (float)requested_speed;
		rpm = (float)current_speed;
	}
	else
	{
		/* Ignora i bit meno significativi della velocit� */
		requested_speed /= ENGINE_PID_SPEED_RESOLUTION;
		requested_speed *= ENGINE_PID_SPEED_RESOLUTION;
		current_speed /= ENGINE_PID_SPEED_RESOLUTION;
		current_speed *= ENGINE_PID_SPEED_RESOLUTION;

		req = (float)requested_speed;
		rpm = (float)current_speed;

		/* Limit PID variation */
		if( rpm < req + 30 && rpm > req - 30 )
		{
			//g_engine.pid_disabled = true;
			if( g_engine.pid_maxvariation > ENGINE_PID_MIN_VARIATION )
			{
				g_engine.pid_maxvariation--;
			}
			if( g_engine.pid_timeout == ENGINE_PID_TIMEOUT_START_MS )
			{
				g_engine.pid_timeout = ENGINE_PID_TIMEOUT_RUN_MS;
				if( rpm < 300 )
					g_engine.filter_reqpow_coeff = ENGINE_PID_FILTER_COEFF_LOWSPD;
				else if( g_engine.status == ENGINE_STATE_CLOSING )
					g_engine.filter_reqpow_coeff = ENGINE_PID_FILTER_COEFF_CLOSING;
				else
					g_engine.filter_reqpow_coeff = ENGINE_PID_FILTER_COEFF_OPENING;
			}
		}
	}

	/* Calculate PID */
	ctrl = PID_Pro(req, rpm, 20000);
	dbg_engine_pid = ctrl;

	/* Adjust driver power with PID result */
	//p = (float)current_power;
	p = (float)ctrl;
//	p += 0.5;
//	if( p < 0.0)
//		p = 0.0;
	po = (int32_t)(p);

	/* Limiti di potenza applicabile */
	if( po > max_power)
		po = max_power;
	if ( po < minoutpwr)
		po = minoutpwr;
	return po;
}


uint8_t loop = 0;
static basetimer_return_t tick1ms_work(void* p)
{
	int32_t pow_in, pow_out;
	static int tick_speed = 0;
	static int tick_power = 0;

    if ( g_engine.elapsed_time < 0xFFFF)
    	g_engine.elapsed_time++;

	switch (g_engine.status)
	{
//		case ENGINE_STATE_TEST:

		case ENGINE_STATE_CLOSING:
		case ENGINE_STATE_OPENING:
			if( ++tick_speed > 200 ) /*tick used for wait the state machine to go in run state*/
				{

			g_engine.max_power = ENGINE_PID_MAX_OUTPUT_POWER;

			if( ++g_engine.pid_ticks >= g_engine.pid_timeout)// || MOTOR_NewStepIn() )
			{
//				g_engine.pid_ticks = 0;
//				if( !g_engine.costa_detected && !g_engine.encoder_fault )
//				{
					/* Run PID */
//					if(loop == 2)
//					{
						pow_in = MOTOR_GetImot();
						g_engine.pow_in = pow_in;
						g_engine.pow_out = pow_out;
//						g_engine.filter_req_speed = (int)g_engine.req_speed;
						g_engine.filter_req_speed_acc += (g_engine.filter_req_speed_coeff * ((int)g_engine.req_speed - g_engine.filter_req_speed)) >> (ENGINE_REQSPEED_FILTER_COEFF_Q-ENGINE_REQSPEED_FILTER_COEFF_N);
						g_engine.filter_req_speed = (g_engine.filter_req_speed_acc>>ENGINE_REQSPEED_FILTER_COEFF_N);
						g_engine.current_speed = MOTOR_GetSpeed();
						pow_out = _speed_control(g_engine.current_speed, g_engine.filter_req_speed, pow_in, g_engine.max_power);
//						loop = 0;
//					}
//				}
			}
//			loop += 1;

			/* Limiti di potenza applicabile */
			if( pow_out > g_engine.max_power)
				pow_out = g_engine.max_power;
			if ( pow_out < ENGINE_PID_MIN_OUTPUT_POWER)
				pow_out = ENGINE_PID_MIN_OUTPUT_POWER;

			/* Motor power */

			/*Impose current reference*/
			if( g_engine.pow_out != g_engine.filter_req_power)
			{
				MOTOR_SetIref(pow_out);
				g_engine.pow_out = pow_out;
				//g_engine.pow_out = g_engine.filter_req_power;
			}

            /*Impose max reference current for 3 second*/
			if ((pow_out > (g_engine.max_power - 200)) || (pow_out < (-g_engine.max_power + 200)))
			{
				if( ++tick_power > 3000 )
					ENGINE_Request(ENGINE_REQUEST_STOP, 0, 0);
			}
			}
			break;

		case ENGINE_STATE_STOPPING:
			if( MOTOR_IsOn() )
			{
				if( MOTOR_Stopping() )
				{
					/* wait */
					break;
				}

				/* I'm braking */
				pow_in = MOTOR_GetImot();
				pow_out = 0;

//				/* Get max stop power */
//				if( g_engine.max_stop_steps > 0 )
//				{
//					g_engine.max_power = g_engine.start_power - ((g_engine.start_power * ENCODER_GetElapsedSteps(g_engine.start_stop_step)) / g_engine.max_stop_steps);
//				}

				/* Get rump down power */
				if( pow_in > 10 && g_engine.filter_reqpow_coeff != 0 )
				{
					if( g_engine.filter_reqpow_coeff != 0 )
					{
						g_engine.filter_req_power_acc += (g_engine.filter_reqpow_coeff * ((int)pow_out - g_engine.filter_req_power)) >> (ENGINE_PID_FILTER_COEFF_Q-ENGINE_PID_FILTER_COEFF_N);
						g_engine.filter_req_power = (g_engine.filter_req_power_acc>>ENGINE_PID_FILTER_COEFF_N);
					}
					else
					{
						g_engine.filter_req_power = pow_out;
						g_engine.filter_req_power_acc = (pow_out<<ENGINE_PID_FILTER_COEFF_N);
					}
				}
				else
				{
					if( pow_in > 10 )
						g_engine.filter_req_power = pow_in - 10;
					else
						g_engine.filter_req_power = 0;
				}

				/* Check max power */
				if( g_engine.filter_req_power > g_engine.max_power )
					g_engine.filter_req_power = g_engine.max_power;

				/* Set motor power */
				if( pow_in != g_engine.filter_req_power)
				{
//					if( g_engine.filter_req_power == 0 )
//						MOTOR_Stop(g_engine.stop_time == 0);
//					else
						MOTOR_SetIref(g_engine.filter_req_power);
				}
				else
				{
					if( pow_in == 0 )
						MOTOR_Stop(g_engine.stop_time == 0);
				}
			}
			else
			{
				if( MOTOR_GetSpeed() == 0 || g_engine.elapsed_time > g_engine.stop_time + 2000 )
				{
					g_engine.status = ENGINE_STATE_STOP;
					if( g_engine.eventCallback != NULL )
						g_engine.eventCallback(ENGINE_EVENT_MOTOR_STOPPED);
				}
			}
			break;

		default:
			break;
	}
	_update_debug_vars();
	return BASETIMER_RETURN_RESTART;
}

void ENGINE_Init(engine_callback_t cb)
{
	memset(&g_engine, 0, sizeof(g_engine));
	g_engine.eventCallback = cb;
	//g_engine.cfg_costa_sens = 100;

	MOTOR_Init();
	//ENCODER_SetPulse(0);
	ENGINE_UpdateMotorType();

	/* Create a base tick function */
	basetimer_start(tick1ms_work, NULL, 1);
}

void ENGINE_UpdateMotorType(void)
{
	ENGINE_LOCK();
	PID_SetParam(MPID_P_IDX, P_LIMIT, ENGINE_PID_P_WEIGHT);
	PID_SetParam(MPID_D_IDX, D_LIMIT, ENGINE_PID_D_WEIGHT);
	PID_SetParam(MPID_I_IDX, I_LIMIT, ENGINE_PID_I_WEIGHT);
	PID_SetParam(MPID_O_IDX, O_LIMIT, O_RULE_WEIGHT);
	ENGINE_UNLOCK();
}

void ENGINE_Request(ENGINE_REQUEST_T request, uint16_t speed_factor, uint16_t actiontime)
{
	uint16_t request_speed = MOTOR_SPEED_RPM(speed_factor);
	MOTOR_DIRECTION_T open_direction;
	MOTOR_DIRECTION_T close_direction;

	g_engine.encoder_fault = false;

	open_direction = MOTOR_DIRECTION_CW;
	close_direction = MOTOR_DIRECTION_ACW;
	//ENCODER_SetOpenDir(open_direction);

	switch (g_engine.status)
	{
		case ENGINE_STATE_STOPPING:
			if( request == ENGINE_REQUEST_STOP )
			{
				ENGINE_LOCK();
				if( actiontime == 0 )
				{
					g_engine.filter_reqpow_coeff = 0;
				}
				else if( g_engine.filter_reqpow_coeff != 0 && g_engine.elapsed_time + actiontime < g_engine.stop_time )
				{
					/* Update stop coeff. */
					g_engine.filter_reqpow_coeff = ( ((1u<<ENGINE_PID_FILTER_COEFF_Q) * 2) / (g_engine.elapsed_time + actiontime));
					if( g_engine.stop_time != 0 )
						g_engine.stop_time = g_engine.elapsed_time + actiontime;
				}
				ENGINE_UNLOCK();
			}
			break;

		case ENGINE_STATE_STOP:
			if(request == ENGINE_REQUEST_OPEN)
			{
				ENGINE_UpdateMotorType();
				ENGINE_LOCK();
				g_engine.pid_disabled = false;
				g_engine.pid_timeout = ENGINE_PID_TIMEOUT_START_MS;
				g_engine.break_detected = false;
				g_engine.pid_maxvariation = ENGINE_PID_MAX_VARIATION;
				g_engine.filter_reqpow_coeff = ENGINE_PID_FILTER_COEFF_STARTOP;
				if( actiontime <= 250 )
					g_engine.filter_req_speed_coeff = ENGINE_REQSPEED_FILTER_COEFFDEF;
				else
					g_engine.filter_req_speed_coeff = ( ((1u<<ENGINE_REQSPEED_FILTER_COEFF_Q) * 3) / actiontime);
				g_engine.req_speed = request_speed;
				g_engine.elapsed_time = 0;
				g_engine.rumpup_time = ENGINE_STARTTIME;	/* Rump-up of max request power */
				g_engine.start_power = ENGINE_POWER_START_OPEN;
				g_engine.filter_req_power = (int32_t)g_engine.start_power;
				g_engine.gate_moved = 0;
				PID_Reset();
				DEBUG_PRINTL("ENG", "Motor start -> open");
//				MOTOR_Start(open_direction);
//				if( g_engine.racemode != ENGINE_RACE_NORMAL && g_engine.racemode != ENGINE_RACE_CALIBRATION_ENCODER )
//					MOTOR_SetIlim(ENGINE_FIRSTRACE_CURRENT_LIMIT);
				MOTOR_SetIref(g_engine.start_power);
				g_engine.status = ENGINE_STATE_OPENING;
				ENGINE_UNLOCK();
			}

			if(request == ENGINE_REQUEST_CLOSE)
			{
				ENGINE_UpdateMotorType();
				ENGINE_LOCK();
				g_engine.pid_disabled = false;
				g_engine.pid_timeout = ENGINE_PID_TIMEOUT_START_MS;
				g_engine.break_detected = false;
				g_engine.pid_maxvariation = ENGINE_PID_MAX_VARIATION;
				g_engine.filter_reqpow_coeff = ENGINE_PID_FILTER_COEFF_STARTCL;
				if( actiontime <= 250 )
					g_engine.filter_req_speed_coeff = ENGINE_REQSPEED_FILTER_COEFFDEF;
				else
					g_engine.filter_req_speed_coeff = ( ((1u<<ENGINE_REQSPEED_FILTER_COEFF_Q) * 3) / actiontime);
				g_engine.req_speed = request_speed;
				g_engine.elapsed_time = 0;
				g_engine.rumpup_time = ENGINE_STARTTIME;	/* Rump-up of max request power */
				g_engine.start_power = ENGINE_POWER_START_CLOSE;
				g_engine.filter_req_power = (int32_t)g_engine.start_power;
				g_engine.gate_moved = 0;
				PID_Reset();
				DEBUG_PRINTL("ENG", "Motor start -> close");
//				MOTOR_Start(close_direction);
//				if( g_engine.racemode != ENGINE_RACE_NORMAL )
//					MOTOR_SetIlim(ENGINE_FIRSTRACE_CURRENT_LIMIT);
				MOTOR_SetIref(g_engine.start_power);
				g_engine.status = ENGINE_STATE_CLOSING;
				ENGINE_UNLOCK();
			}

			if( request == ENGINE_REQUEST_STOP )
			{
				g_engine.req_speed = 0;
				g_engine.filter_req_power = 0;
				MOTOR_Stop(false);
			}
			break;

		case ENGINE_STATE_OPENING:
			if((request == ENGINE_REQUEST_CLOSE) || (request == ENGINE_REQUEST_STOP) || request_speed == 0 )
			{
				/* Stop motor */
				ENGINE_LOCK();
				g_engine.req_speed = 0;					/* Start braking */
				g_engine.elapsed_time = 0;
				g_engine.stop_time = actiontime;
				if( actiontime < 100 )
					g_engine.filter_reqpow_coeff = 0;
				else
					g_engine.filter_reqpow_coeff = ( ((1u<<ENGINE_PID_FILTER_COEFF_Q) * 2) / actiontime);
				g_engine.start_power = MOTOR_GetImot();
				DEBUG_PRINTL("ENG", "Motor stopping...");
				g_engine.status = ENGINE_STATE_STOPPING;
				ENGINE_UNLOCK();
			}
			else if( request == ENGINE_REQUEST_OPEN && request_speed != g_engine.req_speed )
			{
				ENGINE_LOCK();
				if( actiontime <= 250 )
					g_engine.filter_req_speed_coeff = ENGINE_REQSPEED_FILTER_COEFFDEF;
				else
					g_engine.filter_req_speed_coeff = ( ((1u<<ENGINE_REQSPEED_FILTER_COEFF_Q) * 3) / actiontime);
				g_engine.req_speed = request_speed;
				ENGINE_UNLOCK();
			}
			break;
		case ENGINE_STATE_CLOSING:
			if((request == ENGINE_REQUEST_OPEN) || (request == ENGINE_REQUEST_STOP) || request_speed == 0 )
			{
				/* Stop motor */
				ENGINE_LOCK();
				g_engine.req_speed = 0;					/* Start braking */
				g_engine.elapsed_time = 0;
				g_engine.stop_time = actiontime;
				if( actiontime < 100 )
					g_engine.filter_reqpow_coeff = 0;
				else
					g_engine.filter_reqpow_coeff = ( ((1u<<ENGINE_PID_FILTER_COEFF_Q) * 2) / actiontime);
//				g_engine.max_stop_steps = ENCODER_GetRemeaningSteps(g_engine.encoder_race_steps);
//				g_engine.start_stop_step = ENCODER_GetPulse();
				g_engine.start_power = MOTOR_GetImot();
				DEBUG_PRINTL("ENG", "Motor stopping...");
				g_engine.status = ENGINE_STATE_STOPPING;
				ENGINE_UNLOCK();
			}
			else if( request == ENGINE_REQUEST_CLOSE && request_speed != g_engine.req_speed )
			{
				ENGINE_LOCK();
				if( actiontime <= 250 )
					g_engine.filter_req_speed_coeff = ENGINE_REQSPEED_FILTER_COEFFDEF;
				else
					g_engine.filter_req_speed_coeff = ( ((1u<<ENGINE_REQSPEED_FILTER_COEFF_Q) * 3) / actiontime);
				g_engine.req_speed = request_speed;
				ENGINE_UNLOCK();
			}
			break;

		default:
			break;
	}
}



static ENGINE_EVENTS_T _debug_event;
static void engine_debug_callback(uint8_t event)
{
	_debug_event = (ENGINE_EVENTS_T)event;
}
void ENGINE_Debug(void)
{
	static bool initialized;
	static ENGINE_REQUEST_T operation = ENGINE_REQUEST_ANY;
	static MOTOR_DIRECTION_T direction;
	static uint16_t speed_rpm;
	static int32_t current_mA;

	BUTTON_KEY_T k = BUTTON_Get();

	if( !initialized )
	{
		initialized = true;
		g_engTest = true;
		ENGINE_Init(engine_debug_callback);
//		PID_SetParam(MPID_P_IDX, P_LIMIT, 0.5);
//		PID_SetParam(MPID_D_IDX, D_LIMIT, 0.2);
//		PID_SetParam(MPID_I_IDX, I_LIMIT, 0.01);
//		PID_SetParam(MPID_O_IDX, O_LIMIT, 0.05);
		g_engine.filter_reqpow_coeff = 0;
		speed_rpm = 0;
		current_mA = 0;
		operation = ENGINE_REQUEST_ANY;
		direction = MOTOR_DIRECTION_CW;
	}

	switch( _debug_event )
	{
	case ENGINE_EVENT_MOTOR_STOPPED:
		DEBUG_PRINTL("ENG", "STOPPED");
		break;
	case ENGINE_EVENT_OBSTACLE_DETECTI:
		DEBUG_PRINTL("ENG", "OBSTACLE ILIM");
		ENGINE_Request(ENGINE_REQUEST_STOP, 0, 0);
		operation = ENGINE_REQUEST_ANY;
		break;
	case ENGINE_EVENT_OBSTACLE_DETECTC:
		DEBUG_PRINTL("ENG", "OBSTACLE COSTA");
		ENGINE_Request(ENGINE_REQUEST_STOP, 0, 0);
		operation = ENGINE_REQUEST_ANY;
		break;
	case ENGINE_EVENT_OBSTACLE_DETECTB:
		DEBUG_PRINTL("ENG", "OBSTACLE BREAK");
		ENGINE_Request(ENGINE_REQUEST_STOP, 0, 0);
		operation = ENGINE_REQUEST_ANY;
		break;
	case ENGINE_EVENT_ENCODER_FAULT:
		DEBUG_PRINTL("ENG", "ENCODER FAULT");
		ENGINE_Request(ENGINE_REQUEST_STOP, 0, 0);
		operation = ENGINE_REQUEST_ANY;
		break;
	default:
		break;
	}
	_debug_event = ENGINE_EVENT_ANY;


	if( k == BUTTON_HOLDED_ENTER )
	{
#ifdef SPEED_LOOP
		if( operation == ENGINE_REQUEST_CLOSE || operation == ENGINE_REQUEST_OPEN )
		{
			ENGINE_Request(ENGINE_REQUEST_STOP, 0, 0);
			operation = ENGINE_REQUEST_ANY;
		}
#endif

#ifdef CURRENT_LOOP
		if( operation == ENGINE_REQUEST_CLOSE || operation == ENGINE_REQUEST_OPEN )
		{
			MOTOR_Stop(false);
			operation = ENGINE_REQUEST_ANY;
		}
#endif

		/* change next direction */
		if( direction == MOTOR_DIRECTION_CW )
			direction = MOTOR_DIRECTION_ACW;
		else
			direction = MOTOR_DIRECTION_CW;
	}

	if( k == BUTTON_ENTER )
	{
		if( operation == ENGINE_REQUEST_ANY )
		{
			MOTOR_Start(direction);
#ifdef SPEED_LOOP
			//ENGINE_Request(ENGINE_REQUEST_CLOSE, MOTOR_SPEED_RPM_TO_FACTOR(speed_rpm), 0);
#endif
			operation = (direction == MOTOR_DIRECTION_CW ? ENGINE_REQUEST_CLOSE : ENGINE_REQUEST_OPEN);
			current_mA = 800; //for Current_loop, if defined
			speed_rpm = 600;
		}
#ifdef CURRENT_LOOP
		if( operation == ENGINE_REQUEST_CLOSE || operation == ENGINE_REQUEST_OPEN )
		{
			current_mA += 200;
			if( current_mA >= 4000)
				current_mA = 1000;
			MOTOR_SetIref(current_mA);
		}
#endif
#ifdef SPEED_LOOP
		if( operation == ENGINE_REQUEST_CLOSE || operation == ENGINE_REQUEST_OPEN )
		{
			speed_rpm += 200;
			if( speed_rpm >= 3000)
				speed_rpm = 1000;
			ENGINE_Request(operation, MOTOR_SPEED_RPM_TO_FACTOR(speed_rpm), 1000);
		}
#endif
	}
}


//void ENGINE_EmergencyStop(void)
//{
//	MOTOR_TurnOff();
//}
