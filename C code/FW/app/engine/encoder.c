/*
 * encoder.c
 *
 */
#include "sdk_inc.h"
#include "encoder.h"

typedef struct {
	int32_t steps;
	int32_t pos_dir;
	uint32_t period;
	int32_t partialsteps;
	bool lock;
} encoder_t;

static encoder_t g_enc;

void ENCODER_SetOpenDir(MOTOR_DIRECTION_T dir)
{
    if ( dir == MOTOR_DIRECTION_ACW )
    	g_enc.pos_dir = -1;
    else
    	g_enc.pos_dir = 1;
    g_enc.partialsteps = 0;
}

void ENCODER_Update(int32_t stp, uint32_t period)
{
	if( g_enc.lock )
		return;
    if ( stp == 0 )
        return;
    if ( stp == 1)
    {
    	g_enc.steps += g_enc.pos_dir;
    	g_enc.partialsteps += g_enc.pos_dir;
    }
    else
    {
    	g_enc.steps -= g_enc.pos_dir;
    	g_enc.partialsteps -= g_enc.pos_dir;
    }
    g_enc.period = period;
}

void ENCODER_SetPulse(int32_t num)
{
	g_enc.lock = true;
	g_enc.steps = num;
	g_enc.lock = false;
}

int32_t ENCODER_GetPulse(void)
{
	return g_enc.steps;
}

int32_t ENCODER_GetPartialPulse(void)
{
	if( g_enc.partialsteps >= 0 )
		return g_enc.partialsteps;
	return -g_enc.partialsteps;
}

int32_t ENCODER_GetRemeaningSteps(int32_t max_steps)
{
	if( max_steps > 0 )
	{
		if( g_enc.pos_dir > 0 )
		{
			if( max_steps > g_enc.steps )
				return (max_steps-g_enc.steps);
		}
		else
		{
			if(g_enc.steps > 0)
				return g_enc.steps;
		}
	}
	return 0;
}

int32_t ENCODER_GetElapsedSteps(int32_t from_step)
{
	if( from_step > g_enc.steps )
		return (from_step-g_enc.steps);
	else
		return (g_enc.steps-from_step);
}

void ENCODER_Test_ResetData(void)
{
	ENCODER_SetPulse(0);
}

void ENCODER_Test_GettData(int32_t* pOutPulses, uint32_t* pOutTime)
{
	if( pOutPulses != NULL )
		*pOutPulses = g_enc.steps;

	/* Return encoder period in us */
	if( pOutTime != NULL )
		*pOutTime = (g_enc.period * 1000000) / MOTOR_PWM_FREQ;
}
