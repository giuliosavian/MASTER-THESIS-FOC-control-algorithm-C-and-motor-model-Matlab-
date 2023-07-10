#include "options.h"
#include "module_adc.h"

static adc_handler_t g_Adc1Handler;
static int32_t g_CurrentValues[AI_SIGNALS_COUNT];
static int32_t g_filterVdcMotor;
static int32_t g_filterTemp;

/* Global ADC values */
int32_t g_VdcMotor;
int32_t g_Temp;

static basetimer_return_t tickAdcSample_work(void* p)
{
	static uint16_t tick;
	tick += 5;

#if ANALOG_IN_TRIGGER_TIMER == ADC_CONF_TRIGGER_MODE_MANUAL
	adc_trigger(&g_Adc1Handler);
#endif

	/* Update global ADC values */
	ANALOG_Get(AI_MOTOR_VOLTAGE);
	ANALOG_Get(AI_INPUT_TEMP);

	return BASETIMER_RETURN_RESTART;
}

void ANALOG_Init(void)
{
	adc_settings_t adc_settings;

	/* Init ADC peripheral */
	adc_init(&g_Adc1Handler, BOARD_ADC_INSTANCE);
	adc_init_settings(&adc_settings);

	/* Enable all the channels used by the board */
	adc_settings.enabled_channels = (ADC_GET_CHANNEL_MASK(BOARD_ADC_MOTOR_VOLTAGE_CHANNEL) 	|
									 ADC_GET_CHANNEL_MASK(BOARD_ADC_INPUT_TEMP_CHANNEL)	 	);
	adc_settings.sample_speed = ADC_CONF_CHANNEL_SPEED_MEDIUM;
	adc_settings.trigger_mode = BOARD_ADC_TRIGGER_TIMER;
	adc_settings.priority = IRQ_PRIORITY_ADC;
	adc_open(&g_Adc1Handler, &adc_settings);

	/* Initialize values */
	g_filterVdcMotor = 0;
	g_filterTemp = 0;
	g_Temp = -100;

	/* Start 5ms timer */
	basetimer_start(tickAdcSample_work, NULL, 5);
}

void ANALOG_Deinit(void)
{
	basetimer_remove(tickAdcSample_work, NULL);
	adc_close(&g_Adc1Handler);
	adc_deinit(&g_Adc1Handler);
}

int32_t ANALOG_Get(AI_SIGNAL_T signal)
{
	int32_t val = 0;
	uint16_t r;
	switch(signal)
	{
		case AI_MOTOR_VOLTAGE:
			/* return Motor voltage in mV */
			if( TSUCC == adc_read(&g_Adc1Handler, BOARD_ADC_MOTOR_VOLTAGE_CHANNEL, &r) )
			{
				val = ((int32_t)r * 3300) >> 12;
				val *= 6003;
				val >>= 8;

				/* Filter:
				 * alpha = Tp/LPdelay = 5ms / 250ms = 0.005
				 * Q = 16, N = 8
				 * alpha_scaled = 2^Q * 0.01 = 1310
				 */
				g_filterVdcMotor += (1310 * (val - g_VdcMotor)) >> (16-8);
				g_VdcMotor = (g_filterVdcMotor>>8);
				g_CurrentValues[signal] = g_VdcMotor;
			}
			break;
		case AI_INPUT_TEMP:
			/* return data in �C format */
			if( TSUCC == adc_read(&g_Adc1Handler, BOARD_ADC_INPUT_TEMP_CHANNEL, &r) )
			{
				val = ((int32_t)r * 3300) >> 12;
				if( g_Temp == -100 )
				{
					/* First conversion */
					g_Temp = val;
					g_filterTemp = (val<<8);
				}
				/* Filter:
				 * alpha = Tp/LPdelay = 1s / 10s = 0.1
				 * Q = 16, N = 8
				 * alpha_scaled = 2^Q * 0.1 = 13107
				 */
				g_filterTemp += (6250 * (val - g_Temp)) >> (16-8);
				g_Temp = (g_filterTemp>>8);
				g_CurrentValues[signal] = g_Temp;
				//DEBUG_PRINTL("TEMP:", "[%d] -> %d", val, g_Temp);
			}
			break;
		default:
			break;
	}
	return g_CurrentValues[signal];
}


