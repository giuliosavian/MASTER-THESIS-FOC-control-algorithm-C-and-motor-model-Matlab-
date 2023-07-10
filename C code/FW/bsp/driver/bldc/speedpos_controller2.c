/*
 * motor_hallsens.c
 *
 *  Created on: 11 mag 2018
 *      Author: fpasquetto
 */
#include <string.h>
#include "sdk_inc.h"
#include "bldc_def.h"
#include "mc_type.h"
#include "foc_controller.h"
#include "pmsm_motor_parameters.h"
#include "parameters_conversion.h"

#define HALL_ERROR_CORRECTION_STEPS			0
#define MOTOR_HALLSENS_ANGLE_RANGE_BIT		16		/* es: 10bit -> angle 0 - 1023 */
#define MOTOR_HALLSENS_6STEP_START_TICKS	6
#define MOTOR_HALLSENS_ANGLE(x)				((x) * (1<<MOTOR_HALLSENS_ANGLE_RANGE_BIT) / 360)
#define MOTOR_HALLSENS_ANGLE_120			MOTOR_HALLSENS_ANGLE(120)
#define MOTOR_HALLSENS_ANGLE_240			MOTOR_HALLSENS_ANGLE(240)
#define MOTOR_HALLSENS_ANGLE_OFFS			MOTOR_HALLSENS_ANGLE(348)

/*
 * Corrispondenza tra HALL code & Sector
 * -------------------------------------------
 * hall     CW:  2   6   4   5   1   3	 -> il motore in realtà gira in senso antiorario guardandolo da sopra
 * sector   CW:  0   1   2   3   4   5   W-U-V
 * angle    CW:  0  60  120 180 240 300
 * angle16  CW: 180 -120-60  0  60  120
 * -------------------------------------------
 * hall    CCW:  3   1   5   4   6   2	 -> il motore in realtà gira in senso orario guardandolo da sopra
 * sector  CCW:  0   1   2   3   4   5   W-V-U
 * angle   CCW:  0  60  120 180 240 300
 * angle16 CCW: 180 120 60   0  -60-120
 */

const uint8_t g_hall_table_cw[]=
{
    0xFF,   /// 0
    4,   	/// 1
    0,   	/// 2
    5,   	/// 3
    2,   	/// 4
    3,   	/// 5
    1,   	/// 6
    0xFF   	/// 7
};

const uint8_t g_hall_table_ccw[]=
{
    0xFF,   /// 0
    1,   	/// 1
    5,   	/// 2
    0,   	/// 3
    3,   	/// 4
    2,   	/// 5
    4,   	/// 6
    0xFF   	/// 7
};


/* Tabella per incremento/decremento encoder posizionale */
const int32_t g_hall_step_table[] =
{
		0,       /// 0   000 -> 000
		0,       /// 1   000 -> 001
		0,       /// 2   000 -> 010
		0,       /// 3   000 -> 011
		0,       /// 4   000 -> 100
		0,       /// 5   000 -> 101
		0,       /// 6   000 -> 110
		0,       /// 7   000 -> 111
		0,       /// 8   001 -> 000
		0,       /// 9   001 -> 001
		0,       /// 10  001 -> 010
		1,       /// 11  001 -> 011		+1
		0,       /// 12  001 -> 100
		-1,      /// 13  001 -> 101		-1
		0,       /// 14  001 -> 110
		0,       /// 15  001 -> 111
		0,       /// 16  010 -> 000
		0,       /// 17  010 -> 001
		0,       /// 18  010 -> 010
		-1,      /// 19  010 -> 011		-1
		0,       /// 20  010 -> 100
		0,       /// 21  010 -> 101
		1,       /// 22  010 -> 110		+1
		0,       /// 23  010 -> 111
		0,       /// 24  011 -> 000
		-1,      /// 25  011 -> 001		-1
		1,       /// 26  011 -> 010		+1
		0,       /// 27  011 -> 011
		0,       /// 28  011 -> 100
		0,       /// 29  011 -> 101
		0,       /// 30  011 -> 110
		0,       /// 31  011 -> 111
		0,       /// 32  100 -> 000
		0,       /// 33  100 -> 001
		0,       /// 34  100 -> 010
		0,       /// 35  100 -> 011
		0,       /// 36  100 -> 100
		1,       /// 37  100 -> 101		+1
		-1,      /// 38  100 -> 110		-1
		0,       /// 39  100 -> 111
		0,       /// 40  101 -> 000
		1,       /// 41  101 -> 001		+1
		0,       /// 42  101 -> 010
		0,       /// 43  101 -> 011
		-1,      /// 44  101 -> 100		-1
		0,       /// 45  101 -> 101
		0,       /// 46  101 -> 110
		0,       /// 47  101 -> 111
		0,       /// 48  110 -> 000
		0,       /// 49  110 -> 001
		-1,      /// 50  110 -> 010		-1
		0,       /// 51  110 -> 011
		1,       /// 52  110 -> 100		+1
		0,       /// 53  110 -> 101
		0,       /// 54  110 -> 110
		0,       /// 55  110 -> 111
		0,       /// 56  111 -> 000
		0,       /// 57  111 -> 001
		0,       /// 58  111 -> 010
		0,       /// 59  111 -> 011
		0,       /// 60  111 -> 100
		0,       /// 61  111 -> 101
		0,       /// 62  111 -> 110
		0,       /// 63  111 -> 111
};


typedef struct {
	uint8_t	h_state;
	uint8_t h_state_prev;
	int8_t  h_dir;
	uint8_t sector;
	int32_t phase_shift;

	/* counters */
	uint32_t phase_counter;
	uint32_t start_counter;

	/* electrical phase */
	uint32_t phase_period;
	uint32_t phase_period_step;
	uint32_t  el_angle;
	uint32_t  el_angle_measured;
	uint32_t  el_angle_step;
	int32_t  el_angle_err;

	int32_t el_speed_rpm;
	int16_t el_angle16;

	/* incremental encoder */
	int32_t space_step;
	int32_t space_step_prev;
	int32_t space_counter;

	/* phase measure average */
	uint32_t phase_v[6];
	uint32_t phase_v_acc;
	uint8_t  phase_v_index;
	uint8_t  phase_v_navg;

	/* electrical speed average */
    uint32_t speed_v[6];
    uint32_t speed_v_acc;
    uint8_t speed_v_index;
    uint8_t speed_v_navg;

} hall_sensor_instance_t;


static bool g_hallsensor_disconnected;
static hall_sensor_instance_t g_hall;

static uint8_t _ReadHall(void)
{
	uint8_t hall = 0;
	if( gpio_read_pin_input(kEncHall1) == 0 ) hall |= 0x01;
	if( gpio_read_pin_input(kEncHall2) == 0 ) hall |= 0x02;
	if( gpio_read_pin_input(kEncHall3) == 0 ) hall |= 0x04;
	return hall;
}

static void _CheckDisconnect(uint8_t hall_value)
{
	static uint16_t hall_disc_tout = 0;
    if ( (hall_value == 0) || (hall_value == 0x07) )
    {
        if ( hall_disc_tout < 200)   // 10 mS
            hall_disc_tout++;
        else
        	g_hallsensor_disconnected = true;
    }
    else
    {
        hall_disc_tout = 0;
        g_hallsensor_disconnected = false;
    }
}

static uint8_t _GetSector(uint8_t hall_value, int8_t dir)
{
	static uint8_t sector = 0;
	uint8_t m = 0xFF;

	if( dir == 1 )
		m = g_hall_table_cw[hall_value];
	else
		m = g_hall_table_ccw[hall_value];

	if( m != 0xFF )
		sector = m;

    return sector;
}

static void _UpdateSpeed(void)
{
	/* Estimate average speed */
	if( g_hall.start_counter > 6 )
	{
		g_hall.speed_v_acc -= g_hall.speed_v[g_hall.speed_v_index];
		g_hall.speed_v[g_hall.speed_v_index] = g_hall.phase_period;
		g_hall.speed_v_acc += g_hall.phase_period;
		if (++g_hall.speed_v_index >= g_hall.speed_v_navg	)
			g_hall.speed_v_index = 0;
	}
	if( g_hall.speed_v_acc > 0 )
		g_hall.el_speed_rpm = (PWM_FREQUENCY * 60) / ((g_hall.speed_v_acc * POLE_PAIR_NUM) / g_hall.speed_v_navg);
	else
		g_hall.el_speed_rpm = 0;
}


void SPEEDPOS_Clear(void)
{
	g_hall.h_state = _ReadHall();
	g_hall.sector = _GetSector(g_hall.h_state, g_hall.h_dir);
	g_hall.phase_counter = 0;
	g_hall.start_counter = 0;

	g_hall.phase_period = 0;
	g_hall.phase_period_step = 0;
	g_hall.el_angle_step = 0;
	g_hall.el_angle = g_hall.el_angle_measured = (((g_hall.sector * (1<<MOTOR_HALLSENS_ANGLE_RANGE_BIT)) / 6 ) << (32-MOTOR_HALLSENS_ANGLE_RANGE_BIT));
	g_hall.el_angle = g_hall.el_angle_measured += MOTOR_HALLSENS_ANGLE(HALL_PHASE_SHIFT_ACW);
	g_hall.el_angle_err = 0;

	g_hall.phase_v_acc = 0;
	g_hall.phase_v_index = 0;
	memset(g_hall.phase_v, 0, sizeof(g_hall.phase_v));
	g_hall.speed_v_acc = 0;
	g_hall.speed_v_index = 0;
	memset(g_hall.speed_v, 0, sizeof(g_hall.speed_v));
	g_hall.el_speed_rpm = 0;
	g_hall.space_step = 0;
	g_hall.space_step_prev = 0;
	g_hall.space_counter = 0;
}

void SPEEDPOS_Init(void)
{
	g_hall.phase_v_navg = 3;
	g_hall.speed_v_navg = 2;
	SPEEDPOS_Clear();
//	g_hall.phase_shift = MOTOR_HALLSENS_ANGLE(HALL_PHASE_SHIFT_ACW);
//	g_hall.el_angle = g_hall.el_angle_measured = += (g_hall.space_step * g_hall.phase_shift);
	//Init as stm32 code
//	g_hall.h_state = _ReadHall();
//	    switch (g_hall.h_state)
//	    {
//	      case STATE_5:
//	      {
//	        pHandle->_Super.hElAngle = (int16_t)(pHandle->PhaseShift + (S16_60_PHASE_SHIFT / 2));
//	        break;
//	      }
//
//	      case STATE_1:
//	      {
//	        pHandle->_Super.hElAngle = (int16_t)(pHandle->PhaseShift + S16_60_PHASE_SHIFT + (S16_60_PHASE_SHIFT / 2));
//	        break;
//	      }
//
//	      case STATE_3:
//	      {
//	        pHandle->_Super.hElAngle = (int16_t)(pHandle->PhaseShift + S16_120_PHASE_SHIFT + (S16_60_PHASE_SHIFT / 2));
//	        break;
//	      }
//
//	      case STATE_2:
//	      {
//	        pHandle->_Super.hElAngle = (int16_t)(pHandle->PhaseShift - S16_120_PHASE_SHIFT - (S16_60_PHASE_SHIFT / 2));
//	        break;
//	      }
//
//	      case STATE_6:
//	      {
//	        pHandle->_Super.hElAngle = (int16_t)(pHandle->PhaseShift  - S16_60_PHASE_SHIFT - (S16_60_PHASE_SHIFT / 2));
//	        break;
//	      }
//
//	      case STATE_4:
//	      {
//	        pHandle->_Super.hElAngle = (int16_t)(pHandle->PhaseShift - (S16_60_PHASE_SHIFT / 2));
//	        break;
//	      }
//
//	      default:
//	      {
//	        /* Bad hall sensor configutarion so update the speed reliability */
//	        pHandle->SensorIsReliable = false;
//	        break;
//	      }
//	    }
//
//	    /* Initialize the measured angle */
//	    pHandle->MeasuredElAngle = pHandle->_Super.hElAngle;
}

void SPEEDPOS_SampleSpeed(void)
{
}
//
//uint32_t MOTOR_HALLSENS_PhaseCounter(void)
//{
//	return g_hall.start_counter;
//}
//
//int MOTOR_HALLSENS_GetEncoderStep(uint32_t* pSectorTime)
//{
//	if( g_hall.el_speed_rpm > 0 )
//	{
//		if( pSectorTime ) *pSectorTime = g_hall.phase_period_step;
//		return g_hall.space_step;
//	}
//	else
//	{
//		if( pSectorTime ) *pSectorTime = 0;
//		return 0;
//	}
//}

void SPEEDPOS_SetDir(uint8_t dir)
{
	int8_t newd = 0;
	if( dir == MOTOR_DIRECTION_CW )
		newd = 1;
	if( dir == MOTOR_DIRECTION_ACW )
		newd = -1;
	if( g_hall.h_dir != newd )
		g_hall.h_dir = newd;
	if( dir == MOTOR_DIRECTION_ACW )
		g_hall.phase_shift = MOTOR_HALLSENS_ANGLE(HALL_PHASE_SHIFT_ACW);
	else
		g_hall.phase_shift = MOTOR_HALLSENS_ANGLE(HALL_PHASE_SHIFT_CW);
}

uint32_t SPEEDPOS_GetElAngle(void)
{
	return g_hall.el_angle16;
}

//uint32_t SPEEDPOS_GetAngleSteps(void)
//{
//	return (g_hall.el_angle_step>>(32-MOTOR_HALLSENS_ANGLE_RANGE_BIT));
//}

//uint8_t SPEEDPOS_GetSector(void)
//{
//	return g_hall.sector;
//}

int32_t SPEEDPOS_GetSpeedRpm(void)
{
	return g_hall.el_speed_rpm;
}

bool SPEEDPOS_Task(void)
{
	bool hallchanged = false;
	uint32_t sectortime = 0;
	int32_t angle = 0;

	g_hall.h_state = _ReadHall();

	if( g_hall.h_state != g_hall.h_state_prev )
	{
		sectortime = g_hall.phase_counter + 1;
		g_hall.phase_counter = 0;

		g_hall.space_step = g_hall_step_table[((g_hall.h_state_prev << 3)|g_hall.h_state ) & 0x3F];
		g_hall.sector = _GetSector(g_hall.h_state, g_hall.space_step);
		g_hall.space_counter += g_hall.space_step;

		if( g_hall.h_dir == 0 && g_hall.space_counter > 32 )
			g_hall.h_dir = g_hall.space_step;

		/* 60 gradi step su una scala di Nbit punti -> pos = sector * [Nbit]/6
		 * All'angolo viene aggiunto l'offset (sottratto se il verso è negativo)
		 * Poi viene allineato sui 32bit a sinistra
		 */
		angle = ((g_hall.sector * (1<<MOTOR_HALLSENS_ANGLE_RANGE_BIT)) / 6 );
		angle += (g_hall.space_step * g_hall.phase_shift);
		angle += (1<<MOTOR_HALLSENS_ANGLE_RANGE_BIT);
		angle %= (1<<MOTOR_HALLSENS_ANGLE_RANGE_BIT);
		g_hall.el_angle_measured = angle << (32-MOTOR_HALLSENS_ANGLE_RANGE_BIT);
#if HALL_ERROR_CORRECTION_STEPS
		if( g_hall.start_counter < MOTOR_HALLSENS_6STEP_START_TICKS || g_hall.sector == 0 )
		{
			g_hall.el_angle = g_hall.el_angle_measured;
			g_hall.el_angle_err = 0;
			g_hall.el_angle_step = 0;
		}
		else
			g_hall.el_angle += (g_hall.el_angle_step + g_hall.el_angle_err);
#else
		g_hall.el_angle = g_hall.el_angle_measured;
		g_hall.el_angle_err = 0;
		g_hall.el_angle_step = 0;
#endif
		
		g_hall.phase_v_acc -= g_hall.phase_v[g_hall.phase_v_index];
		g_hall.phase_v[g_hall.phase_v_index] = sectortime;
		g_hall.phase_v_acc += g_hall.phase_v[g_hall.phase_v_index];
		if (++g_hall.phase_v_index >= g_hall.phase_v_navg	)
			g_hall.phase_v_index = 0;

		/* Estimated period time: is the time of 6 sectors */
		g_hall.phase_period = (g_hall.phase_v_acc * 6) / g_hall.phase_v_navg;
		g_hall.phase_period_step = (g_hall.phase_period / 6);
		if( g_hall.start_counter > MOTOR_HALLSENS_6STEP_START_TICKS )
		{
			/*
			 * DDS: T[periodo ripetizione] = Tp[tempo campionamento] * 2^32 / K
			 * Tp = periodo interrupt (48.75us)
			 * T = NTp = g_mot.phase_period * Tp
			 * Calcolo il fattore K di incremento per il DDS: K = Tp * 2^32 / T = 2^32 / g_mot.phase_period
			 */
			if( g_hall.phase_period > 0 )
				g_hall.el_angle_step = (0xFFFFFFFF / g_hall.phase_period);
			if( g_hall.el_angle_measured > g_hall.el_angle )
				g_hall.el_angle_err = (g_hall.el_angle_measured - g_hall.el_angle) / (g_hall.phase_period_step);
			else
				g_hall.el_angle_err = -(g_hall.el_angle - g_hall.el_angle_measured) / (g_hall.phase_period_step);
		}

		if( g_hall.start_counter == 0 /*&& g_hall.space_step == g_hall.h_dir*/ )
		{
			/* Primo cambio di settore */
			hallchanged = true;
			g_hall.start_counter++;
		}
		else if( g_hall.space_step == g_hall.space_step_prev /*&& g_hall.space_step == g_hall.h_dir*/ )
		{
			/* Sequenza settori corretta */
			hallchanged = true;
			if( g_hall.start_counter < 0xFFFFFFFF )
				g_hall.start_counter++;
		}
		else
		{
			/* Motor breaked or oscillating */
			if( g_hall.start_counter >= MOTOR_HALLSENS_6STEP_START_TICKS )
				SPEEDPOS_Clear();
		}

		_UpdateSpeed();

	    g_hall.h_state_prev = g_hall.h_state;
	    g_hall.space_step_prev = g_hall.space_step;
	}
	else
	{
		if ( g_hall.phase_counter < 0x7FFFFFFF)
			g_hall.phase_counter++;

#if HALL_ERROR_CORRECTION_STEPS
		if ( g_hall.phase_counter < (g_hall.phase_period_step-(g_hall.phase_period_step>>2)) )
		{
			g_hall.el_angle_measured += g_hall.el_angle_step;
			g_hall.el_angle += (g_hall.el_angle_step + g_hall.el_angle_err);
		}
#else
		if( g_hall.phase_counter < (g_hall.phase_period_step-(g_hall.phase_period_step>>4)) )
		{
			g_hall.el_angle_measured += g_hall.el_angle_step;
		}
		g_hall.el_angle = g_hall.el_angle_measured;
#endif
	}

	/* Timeout Encoder! */
	if( g_hall.phase_counter >= 10000 )
	{
		/* Motor breaked or very slow */
		SPEEDPOS_Clear();
	}
	/* Encoder slow */
	else if( g_hall.phase_counter >= 2000 )
	{
		/* Motor breaked or very slow */
		g_hall.phase_period = 12000;
		_UpdateSpeed();
	}


	if( g_hall.space_step > 0 )
		g_hall.el_angle16 = (int16_t)((int)(g_hall.el_angle>>16) - (int)0x8000);
	else
		g_hall.el_angle16 = (int16_t)((int)0x8000 - (int)(g_hall.el_angle>>16));
	_CheckDisconnect(g_hall.h_state);

	return hallchanged;
}
//
//uint8_t MOTOR_HALLSENS_GetSectorEx(uint8_t hall_value, uint8_t dir)
//{
//	static uint16_t hall_disc_tout = 0;
//	static uint8_t sector = 0;
//	uint8_t m = g_hall_table[hall_value];
//
//    if ( m != 0xFF)
//    {
//        if ( dir == MOTOR_DIRECTION_CW  )
//        	sector = m;
//        else
//        	sector = g_hall_cw2ccw_table[m];
//    }
//
//    /// detect if Hall Sensor is DIsconencted!
//    if ( (hall_value == 0) || (hall_value == 0x07) )
//    {
//        if ( hall_disc_tout < 200)   // 10 mS
//            hall_disc_tout++;
//        else
//        	g_hallsensor_disconnected = true;
//    }
//    else
//    {
//        hall_disc_tout = 0;
//        g_hallsensor_disconnected = false;
//    }
//
//    return sector;
//}
