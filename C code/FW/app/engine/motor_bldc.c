/*
 * motor.c
 *
 *  Created on: 11 mag 2018
 *      Author: fpasquetto
 */
#include <string.h>
#include "sdk_inc.h"
#include "options.h"
#include "motor_bldc.h"
#include "motor_def.h"
#include "driver/inc/driver_bldc.h"

extern volatile int32_t g_VdcMotor;

/* Struttura Controllo Motore  */
typedef struct
{
    uint8_t  			enable;
	#define MOT_EN_NONE   	0
	#define MOT_EN_CW     	1
	#define MOT_EN_CCW    	2
	#define MOT_EN_HOLD   	3
    BLDC_MOTOR_DIRECTION_T 	dir;			/* Motor direction 		*/
    int32_t				i_lim;
    int32_t 			rpm;
    int32_t 			rpm_coeff;
	int32_t 			rpm_acc;
	int32_t 			rpm_filtered;
    int32_t 			rpm_filtered_acc;
    uint16_t 			break_state;
    uint8_t				state;
	#define MOT_STATE_INIT				0
	#define MOT_STATE_CALIBOFFS_START	1
	#define MOT_STATE_CALIBOFFS_WAIT	2
	#define MOT_STATE_IDLE				3
	#define MOT_STATE_PREPARERUN_START	4
	#define MOT_STATE_PREPARERUN_WAIT	5
	#define MOT_STATE_RUN				6
	#define MOT_STATE_STOP				7
	#define MOT_STATE_STOPWAIT			8
	#define MOT_STATE_ERROR				9

} motor_t;

static motor_t g_mot;
static bldc_handler_t g_bldc;

static void _drive(BLDC_EVENT_T ev)
{
	if( ev == BLDC_EVENT_BREAK )
	{
		g_mot.break_state = 1;
	}
	if( ev == BLDC_EVENT_I_SAMPLED )
	{

	}
}
//uint32_t potentiometer;
uint8_t g_first = 0;

static basetimer_return_t tick1ms_work(void* p)
{
	static int tick = 0;
	static int intialized = 0;

	bldc_sample_speed(&g_bldc);

	switch(g_mot.state)
	{
	case MOT_STATE_INIT:
		if( ++tick > 500 )
			g_mot.state = MOT_STATE_CALIBOFFS_START;
		break;
	case MOT_STATE_CALIBOFFS_START:
		if( bldc_offset_calibration_wait(&g_bldc, true) )
			g_mot.state = MOT_STATE_CALIBOFFS_WAIT;
		break;
	case MOT_STATE_CALIBOFFS_WAIT:
		if( bldc_offset_calibration_wait(&g_bldc, false) )
		{
			if( intialized == 0 )
				bldc_bootcap_charge(&g_bldc);
			g_mot.state = MOT_STATE_IDLE;
		}
		break;

	case MOT_STATE_IDLE:
		if( bldc_error_occurred(&g_bldc) )
			g_mot.state = MOT_STATE_ERROR;
		if(	g_mot.enable == MOT_EN_CW )
		{
			g_mot.state = MOT_STATE_PREPARERUN_START;
			g_mot.dir = BLDC_MOTOR_DIRECTION_CW;
		}
		if(	g_mot.enable == MOT_EN_CCW )
		{
			g_mot.state = MOT_STATE_PREPARERUN_START;
			g_mot.dir = BLDC_MOTOR_DIRECTION_ACW;
		}
		break;

	case MOT_STATE_PREPARERUN_START:
		if( bldc_offset_calibration_wait(&g_bldc, true) )
			g_mot.state = MOT_STATE_PREPARERUN_WAIT;
		break;
	case MOT_STATE_PREPARERUN_WAIT:
		if( bldc_offset_calibration_wait(&g_bldc, false) )
		{
			bldc_start(&g_bldc, g_mot.dir);
			g_mot.state = MOT_STATE_RUN;
		}
		break;

	case MOT_STATE_RUN:
		if(	g_mot.enable == MOT_EN_NONE )
		{
			g_mot.state = MOT_STATE_STOP;
		}
		break;

	case MOT_STATE_STOP:
		tick = 0;
		bldc_stop(&g_bldc);
		g_mot.state = MOT_STATE_STOPWAIT;
		break;
	case MOT_STATE_STOPWAIT:
		if( ++tick > 400 )
		{
			g_mot.state = MOT_STATE_IDLE;
		}
		break;
	case MOT_STATE_ERROR:
		break;
	}
	return BASETIMER_RETURN_RESTART;
}

void MOTOR_Init(void)
{
	bldc_init(&g_bldc, BLDC_INSTANCE_1);
	bldc_settings_t settings;
	bldc_init_settings(&settings);
	settings.adc_instance = MOTOR_CURRENT_ADC_INSTANCE;
	settings.tim_instance = MOTOR_DRIVER_TIMER_INSTANCE;
	settings.priority = 0;
	settings.adc_ch_phase1 = MOTOR_CURRENT_ADC_CHANNEL_A;
	settings.adc_ch_phase2 = MOTOR_CURRENT_ADC_CHANNEL_B;
	settings.adc_ch_phase3 = MOTOR_CURRENT_ADC_CHANNEL_C;
	settings.pwm_freq = MOTOR2_PWM_FREQ;
	settings.break_enabled = true;
	bldc_set_callback(&g_bldc, _drive);
	bldc_open(&g_bldc, &settings);

	memset(&g_mot, 0, sizeof(g_mot));
	/* Create a base tick function */
	basetimer_start(tick1ms_work, NULL, 1);
}

void MOTOR_Start(MOTOR_DIRECTION_T dir)
{
	if ( dir == MOTOR_DIRECTION_CW )
		g_mot.enable = MOT_EN_CW;
	else
		g_mot.enable = MOT_EN_CCW;
}

void MOTOR_Stop(bool release)
{
	g_mot.enable = MOT_EN_NONE;
}

bool MOTOR_Stopping(void)
{
	return ( g_mot.enable == MOT_EN_NONE && (g_mot.state == MOT_STATE_STOPWAIT || g_mot.state == MOT_STATE_STOP) );
}

bool MOTOR_IsOn(void)
{
	return (g_mot.enable != MOT_EN_NONE);
}

bool MOTOR_NewStepIn(void)
{
	bool step = false;
	return step;
}

void MOTOR_SetIlim(int32_t curlimit)
{
	if( curlimit > 0 )
		g_mot.i_lim = curlimit;
}

int32_t MOTOR_GetImot(void)
{
	int32_t iq, id;
	bldc_get_current(&g_bldc, &iq, &id);
	return (iq);
}

uint32_t MOTOR_GetILim(void)
{
	return g_mot.i_lim;
}

void MOTOR_SetIref(int32_t mA)
{
	bldc_set_reference(&g_bldc, mA, 0);
}

void MOTOR_SetSpeedFilterCoeff(int32_t coeff)
{
    g_mot.rpm_coeff = coeff;
}

int32_t MOTOR_GetSpeed(void)
{
	int32_t speed = bldc_get_speed(&g_bldc);
	return( speed );
}

uint16_t MOTOR_GetDirection(void)
{
	if( g_mot.dir == BLDC_MOTOR_DIRECTION_ACW )
		return MOTOR_DIRECTION_ACW;
	else if( g_mot.dir == BLDC_MOTOR_DIRECTION_CW )
		return MOTOR_DIRECTION_CW;
	else
		return MOTOR_DIRECTION_ANY;
}

uint32_t MOTOR_GetIline(void)
{
	return 0;
}
