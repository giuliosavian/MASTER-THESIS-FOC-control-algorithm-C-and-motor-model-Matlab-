#ifndef _MODULE_ADC_H_
#define _MODULE_ADC_H_

#include "sdk_inc.h"


typedef enum {
	AI_MOTOR_VOLTAGE,
	AI_INPUT_TEMP,

	AI_SIGNALS_COUNT
} AI_SIGNAL_T;

/**
 **********************************************************************
 * Exported prototypes
 **********************************************************************
 */
void ANALOG_Init(void);
void ANALOG_Deinit(void);
int32_t ANALOG_Get(AI_SIGNAL_T signal);

 
#endif /* _MODULE_ADC_H_ */

