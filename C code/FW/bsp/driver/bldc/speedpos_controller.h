/*
 * speed&pos-controller.h
 */


#ifndef BSP_DRIVER_BLDC_SPEEDPOS_CONTROLLER_H_
#define BSP_DRIVER_BLDC_SPEEDPOS_CONTROLLER_H_

#include "hall_speed_pos_fdbk.h"
#include "pwm_curr_fdbk.h"
#include "bldc_def.h"

void SPEEDPOS_Init(void);
void SPEEDPOS_Clear(void);
void SPEEDPOS_Task(void);
void SPEEDPOS_SampleSpeed(void);
void SPEEDPOS_SetDir(uint8_t dir);

int32_t SPEEDPOS_GetSpeedRpm(void);
int16_t SPEEDPOS_GetElAngle(void);

#endif /* BSP_DRIVER_BLDC_SPEEDPOS_CONTROLLER_H_ */
