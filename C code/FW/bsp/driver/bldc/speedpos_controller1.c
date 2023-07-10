/*
 * hall_controller.c
 */
#include "sdk_inc.h"
#include "bldc_def.h"
#include "mc_type.h"
#include "foc_controller.h"
#include "pmsm_motor_parameters.h"
#include "parameters_conversion.h"

TIM_HandleTypeDef htim2;

HALL_Handle_t HALL_M1 =
{
		._Super = {
				.bElToMecRatio                     =	POLE_PAIR_NUM,
				.hMaxReliableMecSpeedUnit          =	(uint16_t)(1.15*MAX_APPLICATION_SPEED_UNIT),
				.hMinReliableMecSpeedUnit          =	(uint16_t)(MIN_APPLICATION_SPEED_UNIT),
				.bMaximumSpeedErrorsNumber         =	MEAS_ERRORS_BEFORE_FAULTS,
				.hMaxReliableMecAccelUnitP         =	65535,
				.hMeasurementFrequency             =	TF_REGULATION_RATE_SCALED,
				.DPPConvFactor                     =  DPP_CONV_FACTOR,

		},
		.SensorPlacement     = HALL_SENSORS_PLACEMENT,
		.PhaseShift          = (int16_t)(HALL_PHASE_SHIFT_ACW * 65536/360),
		.SpeedSamplingFreqHz = SPEED_LOOP_FREQUENCY_HZ,
		.SpeedBufferSize     = HALL_AVERAGING_FIFO_DEPTH,
		.TIMClockFreq        = HALL_TIM_CLK,
		.TIMx                = TIM2,

		.ICx_Filter          = M1_HALL_IC_FILTER,

		.PWMFreqScaling      = PWM_FREQ_SCALING,
		.HallMtpa            = HALL_MTPA,

		.H1Port             =  GPIOA,
		.H1Pin              =  15,
		.H2Port             =  GPIOB,
		.H2Pin              =  3,
		.H3Port             =  GPIOB,
		.H3Pin              =  10,
};

HALL_Handle_t *pHALL_M1 = { &HALL_M1 };

extern void driver_bldc_error_handler(void);

static void _tim2_init(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_HallSensor_InitTypeDef sConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = M1_HALL_TIM_PERIOD;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		driver_bldc_error_handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		driver_bldc_error_handler();
	}
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = M1_HALL_IC_FILTER;
	sConfig.Commutation_Delay = 0;
	if (HAL_TIMEx_HallSensor_Init(&htim2, &sConfig) != HAL_OK)
	{
		driver_bldc_error_handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		driver_bldc_error_handler();
	}
}

void SPEEDPOS_Init(void)
{
	peripheral_enable(P_TIM2);
	_tim2_init();
	HALL_Init(&HALL_M1);
	HALI_SET_IRQ_PRIORITY(TIM2_IRQn, 2);
	HALI_ENABLE_IRQ(TIM2_IRQn);
}

void SPEEDPOS_Clear(void)
{
	HALL_Clear(&HALL_M1);
}

void SPEEDPOS_SampleSpeed(void)
{
	int16_t wAux = 0;
	//HALL_CalcAvrgMecSpeedUnit(&HALL_M1, &wAux) ;
}

int32_t SPEEDPOS_GetSpeedRpm(void)
{
	int16_t wAux = HALL_M1._Super.hAvrMecSpeedUnit;
	if(wAux < 0)
	{
		wAux *= -1;
	}
	return (int32_t)SPEED_UNIT_2_RPM(wAux);
}

int16_t SPEEDPOS_GetElAngle(void)
{
	return HALL_M1._Super.hElAngle;
}

void SPEEDPOS_SetDir(uint8_t dir)
{
	if( dir == MOTOR_DIRECTION_ACW )
		HALL_M1.PhaseShift = (int16_t)((HALL_PHASE_SHIFT_ACW * 65536)/360);
	else
		HALL_M1.PhaseShift = (int16_t)((HALL_PHASE_SHIFT_CW * 65536)/360);
}

void SPEEDPOS_Task(void)
{
	/* High frequency tick - same as FOC */
	HALL_CalcElAngle(&HALL_M1);
}

void TIM2_IRQHandler(void)
{
	/* HALL Timer Update IT always enabled, no need to check enable UPDATE state */
	if (LL_TIM_IsActiveFlag_UPDATE(TIM2))
	{
		LL_TIM_ClearFlag_UPDATE(TIM2);
		HALL_TIMx_UP_IRQHandler(&HALL_M1);
	}
	else
	{
		/* Nothing to do */
	}
	/* HALL Timer CC1 IT always enabled, no need to check enable CC1 state */
	if (LL_TIM_IsActiveFlag_CC1 (TIM2))
	{
		LL_TIM_ClearFlag_CC1(TIM2);
		HALL_TIMx_CC_IRQHandler(&HALL_M1);
	}
	else
	{
		/* Nothing to do */
	}
}
