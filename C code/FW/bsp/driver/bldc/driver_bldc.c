/*
 * bldc_driver.c
 *
 *  Created on: 11 mag 2018
 *      Author: fpasquetto
 */
#include <string.h>
#include <stdbool.h>
#include "sdk_inc.h"
#include "driver/inc/driver_bldc.h"
#include "platform/platform.h"

#include "r3_1_f4xx_pwm_curr_fdbk.h"
//#include "hall_speed_pos_fdbk.h"
#include "foc_controller.h"
#include "speedpos_controller.h"
//#include "pid_regulator.h"
#include "drive_parameters.h"
//#include "pwm_curr_fdbk.h"
//#include "circle_limitation.h"
#include "parameters_conversion.h"
#include "pwm_common.h"



#define SIGN_DRIVER_BLDC		0x7A01
#define DEFAULT_PRIORITY		0	/* The max priority level */
#define BLDC_STATUS_CLOSED		0
#define BLDC_STATUS_OPENED		1
#define BLDC_USE_DAC_FOR_DEBUG	0
#define OFFCALIBRWAIT_MS     0

typedef struct {
	bldc_handler_t *Handler;
} bldc_driver_t;


ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim1;

static bldc_driver_t g_pBLDC[BLDC_MAX_INSTANCES];

R3_1_Params_t R3_1_ParamsM1 =
{
		/* Current reading A/D Conversions initialization -----------------------------*/
		.ADCx = ADC1,

		/* PWM generation parameters --------------------------------------------------*/
		.RepetitionCounter = REP_COUNTER,
		.hTafter            = TW_AFTER,
		.hTbefore           = TW_BEFORE_R3_1,
		.TIMx               = TIM1,
		.Tsampling                  = (uint16_t)SAMPLING_TIME,
		.Tcase2                     = (uint16_t)SAMPLING_TIME + (uint16_t)TDEAD + (uint16_t)TRISE,
		.Tcase3                     = ((uint16_t)TDEAD + (uint16_t)TNOISE + (uint16_t)SAMPLING_TIME)/2u,

		/* PWM Driving signals initialization ----------------------------------------*/
		.LowSideOutputs = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING,
		.pwm_en_u_port = MC_NULL,
		.pwm_en_u_pin  = (uint16_t) 0,
		.pwm_en_v_port = MC_NULL,
		.pwm_en_v_pin  = (uint16_t) 0,
		.pwm_en_w_port = MC_NULL,
		.pwm_en_w_pin  = (uint16_t) 0,

		.ADCConfig = {
				( uint32_t )( MC_ADC_CHANNEL_11<<ADC_JSQR_JSQ3_Pos ) | MC_ADC_CHANNEL_10<<ADC_JSQR_JSQ4_Pos | 1<<ADC_JSQR_JL_Pos,
				( uint32_t )( MC_ADC_CHANNEL_0<<ADC_JSQR_JSQ3_Pos ) | MC_ADC_CHANNEL_10<<ADC_JSQR_JSQ4_Pos | 1<<ADC_JSQR_JL_Pos,
				( uint32_t )( MC_ADC_CHANNEL_10<<ADC_JSQR_JSQ3_Pos ) | MC_ADC_CHANNEL_0<<ADC_JSQR_JSQ4_Pos | 1<<ADC_JSQR_JL_Pos,
				( uint32_t )( MC_ADC_CHANNEL_11<<ADC_JSQR_JSQ3_Pos ) | MC_ADC_CHANNEL_0<<ADC_JSQR_JSQ4_Pos | 1<<ADC_JSQR_JL_Pos,
				( uint32_t )( MC_ADC_CHANNEL_0<<ADC_JSQR_JSQ3_Pos ) | MC_ADC_CHANNEL_11<<ADC_JSQR_JSQ4_Pos | 1<<ADC_JSQR_JL_Pos,
				( uint32_t )( MC_ADC_CHANNEL_10<<ADC_JSQR_JSQ3_Pos ) | MC_ADC_CHANNEL_11<<ADC_JSQR_JSQ4_Pos | 1<<ADC_JSQR_JL_Pos,
		},

		.ADCDataReg1 = {
				&ADC1->JDR1,
				&ADC1->JDR1,
				&ADC1->JDR2,
				&ADC1->JDR2,
				&ADC1->JDR1,
				&ADC1->JDR2,
		},
		.ADCDataReg2 = {
				&ADC1->JDR2,
				&ADC1->JDR2,
				&ADC1->JDR1,
				&ADC1->JDR1,
				&ADC1->JDR2,
				&ADC1->JDR1,
		},

		/* Emergency input (BKIN2) signal initialization -----------------------------*/
		.EmergencyStop = DISABLE,
};

PWMC_R3_1_Handle_t PWM_Handle_M1 =
{
		{
				.pFctGetPhaseCurrents       = &R3_1_GetPhaseCurrents,
				.pFctSwitchOffPwm           = &R3_1_SwitchOffPWM,
				.pFctSwitchOnPwm            = &R3_1_SwitchOnPWM,
				.pFctCurrReadingCalib       = &R3_1_CurrentReadingCalibration,
				.pFctTurnOnLowSides         = &R3_1_TurnOnLowSides,
				.pFctSetADCSampPointSectX   = &R3_1_SetADCSampPointSectX,
				.pFctSetOffsetCalib         = &R3_1_SetOffsetCalib,
				.pFctGetOffsetCalib         = &R3_1_GetOffsetCalib,
				.pFctIsOverCurrentOccurred  = &R3_1_IsOverCurrentOccurred,
				.pFctOCPSetReferenceVoltage = MC_NULL,
				.pFctRLDetectionModeEnable  = &R3_1_RLDetectionModeEnable,
				.pFctRLDetectionModeDisable = &R3_1_RLDetectionModeDisable,
				.pFctRLDetectionModeSetDuty = &R3_1_RLDetectionModeSetDuty,
				.hT_Sqrt3 = (PWM_PERIOD_CYCLES*SQRT3FACTOR)/16384u,
				.Sector = 0,
				.CntPhA = 0,
				.CntPhB = 0,
				.CntPhC = 0,
				.SWerror = 0,
				.TurnOnLowSidesAction = false,
				.OffCalibrWaitTimeCounter = 0,
				.Motor = M1,
				.RLDetectionMode = false,
				.Ia = 0,
				.Ib = 0,
				.Ic = 0,
				.DTTest = 0,
				.DTCompCnt = DTCOMPCNT,
				.PWMperiod          = PWM_PERIOD_CYCLES,
				.OffCalibrWaitTicks = OFFCALIBRWAIT_MS,
				.Ton                 = TON,
				.Toff                = TOFF
		},
		.PhaseAOffset = 0,
		.PhaseBOffset = 0,
		.PhaseCOffset = 0,
		.Half_PWMPeriod = PWM_PERIOD_CYCLES/2u,
		.PolarizationCounter = 0,
		.ADC_ExternalTriggerInjected = 0,
		.ADCTriggerEdge = 0,
		.OverCurrentFlag = false,
		.OverVoltageFlag = false,
		.BrakeActionLock = false,

		.pParams_str = &R3_1_ParamsM1,
};

void driver_bldc_error_handler(void)
{
	while(1) {
		__asm("nop");
	}
}

static void _adc_init(void)
{
	ADC_InjectionConfTypeDef sConfigInjected = {0};
	ADC_ChannelConfTypeDef sConfig = {0};

	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_LEFT;
	hadc1.Init.NbrOfConversion = 2;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	HAL_ADC_Init(&hadc1);

	sConfigInjected.InjectedChannel = ADC_CHANNEL_0;
	sConfigInjected.InjectedRank = 1;
	sConfigInjected.InjectedNbrOfConversion = 3;
	sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_15CYCLES;
	sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
	sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_CC4;
	sConfigInjected.AutoInjectedConv = DISABLE;
	sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
	sConfigInjected.InjectedOffset = 0;
	HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected);

	sConfigInjected.InjectedChannel = ADC_CHANNEL_11;
	sConfigInjected.InjectedRank = 2;
	HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected);

	sConfigInjected.InjectedChannel = ADC_CHANNEL_10;
	sConfigInjected.InjectedRank = 3;
	HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected);

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
//	sConfig.Channel = ADC_CHANNEL_1;
//	sConfig.Rank = 1;
//	sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
//	sConfig.Offset = 0;
//
//	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//	{
//		driver_bldc_error_handler();
//	}
//	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//	 */
//	sConfig.Channel = ADC_CHANNEL_12;
//	sConfig.Rank = 2;
//
//	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//	{
//		driver_bldc_error_handler();
//	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */

	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	sConfig.Offset = 0;


	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		driver_bldc_error_handler();
	}
}


static void _tim1_init(void)
{
	TIM_SlaveConfigTypeDef sSlaveConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = ((TIM_CLOCK_DIVIDER) - 1);
	htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
	htim1.Init.Period = ((PWM_PERIOD_CYCLES) / 2);
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
	htim1.Init.RepetitionCounter = (REP_COUNTER);
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
		driver_bldc_error_handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
	{
		driver_bldc_error_handler();
	}
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
	sSlaveConfig.InputTrigger = TIM_TS_ITR1;
	if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
	{
		driver_bldc_error_handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		driver_bldc_error_handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		driver_bldc_error_handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		driver_bldc_error_handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		driver_bldc_error_handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM2;
	sConfigOC.Pulse = (((PWM_PERIOD_CYCLES) / 2) - (HTMIN));
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		driver_bldc_error_handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_1;
	sBreakDeadTimeConfig.DeadTime = ((DEAD_TIME_COUNTS) / 2);
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	{
		driver_bldc_error_handler();
	}
}


#if BLDC_USE_DAC_FOR_DEBUG
static DAC_HandleTypeDef hdac;
static void _init_dac_debug(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	__HAL_RCC_DAC_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	DAC_ChannelConfTypeDef sConfig;
	hdac.Instance = DAC;
	HAL_DAC_Init(&hdac);

	/* DAC channel OUT1 config */
	sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1);
	/* DAC channel OUT2 config */
	HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2);
}

static void _start_dac_debug(void)
{
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1); // START DAC : DEBUG
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2); // START DAC : DEBUG
}

static void _set_dac_debug(uint16_t cha, uint16_t chb)
{
	HAL_DACEx_DualSetValue(&hdac, DAC_ALIGN_12B_R, cha, chb);
	hdac.Instance->SWTRIGR |= (uint32_t)(DAC_SWTRIGR_SWTRIG1 | DAC_SWTRIGR_SWTRIG2);
}
#else
static void _init_dac_debug(void) {}
static void _start_dac_debug(void) {}
void _set_dac_debug(uint16_t cha, uint16_t chb) {}
#endif


void bldc_init(bldc_handler_t* handler, bldc_instance_t instance)
{
	if( handler != NULL && instance < BLDC_MAX_INSTANCES )
	{
		/* Valid parameters */

		/* write a sign that means the handler was initialized */
		handler->sign = SIGN_DRIVER_BLDC;

		/* save the instance number */
		handler->instance = (uint8_t)instance;

		/* Default priority */
		handler->priority = DEFAULT_PRIORITY;

		/* initialize to zero */
		handler->pCallback = NULL;
		handler->status = BLDC_STATUS_CLOSED;
		handler->direction = 0;

		/* Save singleton in memory */
		g_pBLDC[instance].Handler = handler;
	}
}

void bldc_deinit(bldc_handler_t* handler)
{
	if( handler != NULL && handler->sign == SIGN_DRIVER_BLDC )
	{
		/* Valid handler */

		if( handler->status == BLDC_STATUS_OPENED )
		{
			/* Close it before remove the driver */
			bldc_close(handler);
		}

		/* remove the sign */
		handler->sign = 0;

		/* remove the singleton */
		g_pBLDC[handler->instance].Handler = NULL;
	}
}

bldc_handler_t* bldc_get_handler(bldc_instance_t instance)
{
	if( instance < BLDC_MAX_INSTANCES )
	{
		/* return the singleton */
		return g_pBLDC[instance].Handler;
	}
	return NULL;
}

void bldc_init_settings(bldc_settings_t* settings)
{
}

T_ERR bldc_set_callback(bldc_handler_t *handler, bldc_callback_t callback)
{
	if( handler != NULL && handler->sign == SIGN_DRIVER_BLDC )
	{
		handler->pCallback = callback;
		return TSUCC;
	}
	return TFAIL;
}

T_ERR bldc_open(bldc_handler_t* handler, bldc_settings_t *settings)
{
	/* Check valid handler */
	if( handler == NULL || handler->sign != SIGN_DRIVER_BLDC )
		return TFAIL;

	/* Check parameters */
	if( settings == NULL || settings->priority > HALI_MAX_PRIORITY_VALUE )
		return TFAIL;

	/* Stop and close the driver if it is open */
	if( handler->status == BLDC_STATUS_OPENED )
		bldc_close(handler);

	/* Save IRQ priority */
	handler->priority = settings->priority;

	/* Enable the peripheral */
	peripheral_enable(P_ADC1);
	peripheral_enable(P_TIM1);

	_adc_init();
	_tim1_init();

	/* Start DAC for debug */
	_init_dac_debug();
	_start_dac_debug();

	R3_1_Init(&PWM_Handle_M1);
	SPEEDPOS_Init();

	HALI_SET_IRQ_PRIORITY(TIM1_UP_TIM10_IRQn, 0);
	HALI_ENABLE_IRQ(TIM1_UP_TIM10_IRQn);
	HALI_SET_IRQ_PRIORITY(ADC_IRQn, 0);
	HALI_ENABLE_IRQ(ADC_IRQn);
//	HALI_SET_IRQ_PRIORITY(TIM1_BRK_TIM9_IRQn, 4);
//	HALI_ENABLE_IRQ(TIM1_BRK_TIM9_IRQn);
	startTimers();

	FOC_Init(&PWM_Handle_M1._Super);

	/* The driver is open */
	handler->status = BLDC_STATUS_OPENED;
	handler->error = 0;

	return TSUCC;
}

T_ERR bldc_close(bldc_handler_t* handler)
{
	/* Check valid handler */
	if( handler == NULL || handler->sign != SIGN_DRIVER_BLDC )
		return TFAIL;

	handler->status = BLDC_STATUS_CLOSED;
	return TSUCC;
}

void bldc_get_current( bldc_handler_t* handler, int32_t* pIq_mA, int32_t* pId_mA )
{
	FOC_GetI(pIq_mA, pId_mA);
}

int32_t bldc_get_speed(bldc_handler_t* handler)
{
	return SPEEDPOS_GetSpeedRpm();
}

void bldc_sample_speed(bldc_handler_t* handler)
{
	SPEEDPOS_SampleSpeed();
}

bool bldc_offset_calibration_wait(bldc_handler_t* handler, bool start)
{
	bool ret = false;
	if( start )
	{
		if (PWM_Handle_M1._Super.offsetCalibStatus == false)
		{
		  if( PWMC_CurrentReadingCalibr(&PWM_Handle_M1._Super, CRC_START) )
			  ret = true;
		}
		else
		{
		  /* calibration already done. Enables only TIM channels */
	      PWM_Handle_M1._Super.OffCalibrWaitTimeCounter = 1u;
		  PWMC_CurrentReadingCalibr(&PWM_Handle_M1._Super, CRC_EXEC);
		  ret = true;
		}
	}
	else
	{
		if (PWMC_CurrentReadingCalibr(&PWM_Handle_M1._Super, CRC_EXEC))
			ret = true;
	}
	if( PWM_Handle_M1._Super.SWerror != 0 )
		handler->error = 1;
	return ret;
}

bool bldc_error_occurred(bldc_handler_t* handler)
{
	return (bool)handler->error;
}

void bldc_bootcap_charge(bldc_handler_t* handler)
{
	R3_1_TurnOnLowSides(&PWM_Handle_M1._Super);
}

void bldc_start(bldc_handler_t* handler, BLDC_MOTOR_DIRECTION_T direction)
{
	handler->error = 0;
	handler->direction = (direction&0x03);
	SPEEDPOS_SetDir(direction);
	SPEEDPOS_Clear();
	R3_1_SwitchOffPWM(&PWM_Handle_M1._Super);
	FOC_Clear( M1 );
	FOC_SetRefI(0, 0);
	PWMC_SwitchOnPWM(&PWM_Handle_M1._Super);
}

void bldc_stop(bldc_handler_t* handler)
{
	R3_1_SwitchOffPWM(&PWM_Handle_M1._Super);
	FOC_Clear(M1);
}

void bldc_set_reference(bldc_handler_t* handler, int32_t IqRef_mA, int32_t IdRef_mA)
{
	if(handler->direction == BLDC_MOTOR_DIRECTION_ACW)
	{
		FOC_SetRefI(IqRef_mA, IdRef_mA);
	}
	else if (handler->direction == BLDC_MOTOR_DIRECTION_CW)
	{
		FOC_SetRefI(-IqRef_mA, -IdRef_mA);
	}
}



/*
 * IRQ
 */

void ADC_IRQHandler(void)
{
	if(LL_ADC_IsActiveFlag_JEOS(ADC1))
	{
		// Clear Flags
		ADC1->SR &= ~(uint32_t)(LL_ADC_FLAG_JEOS | LL_ADC_FLAG_JSTRT);

		/* High frequency task */
		SPEEDPOS_Task();
		FOC_Task();
	}
}

/* Update EVENT */
void TIM1_UP_TIM10_IRQHandler(void)          // TIM1 Update and TIM10
{
	LL_TIM_ClearFlag_UPDATE(TIM1);
	R3_1_TIMx_UP_IRQHandler(&PWM_Handle_M1);
}

/* TIM1 Break and TIM9 */
void TIM1_BRK_TIM9_IRQHandler(void)
{
	if (LL_TIM_IsActiveFlag_BRK(TIM1))
	{
		LL_TIM_ClearFlag_BRK(TIM1);
		R3_1_BRK_IRQHandler(&PWM_Handle_M1);
	}
}






