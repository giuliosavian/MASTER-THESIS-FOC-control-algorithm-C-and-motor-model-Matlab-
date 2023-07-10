/*
 * foc-controller.h
 *
 *  Created on: 2 mag 2022
 *      Author: fpasquetto
 */


#ifndef BSP_DRIVER_BLDC_FOC_CONTROLLER_H_
#define BSP_DRIVER_BLDC_FOC_CONTROLLER_H_

#include "pid_regulator.h"
#include "hall_speed_pos_fdbk.h"
#include "circle_limitation.h"
#include "pwm_curr_fdbk.h"
#include "bldc_def.h"

//extern PID_Handle_t PIDIqHandle_M1;
//extern PID_Handle_t PIDIdHandle_M1;
//extern CircleLimitation_Handle_t CircleLimitationM1;
//extern HALL_Handle_t HALL_M1;
//
//extern HALL_Handle_t *pHALL_M1;
//extern PID_Handle_t *pPIDIq[NBR_OF_MOTORS];
//extern PID_Handle_t *pPIDId[NBR_OF_MOTORS];

void FOC_Init(PWMC_Handle_t* pwmhandler);
void FOC_Clear(uint8_t bMotor);
uint8_t FOC_Task(void);

void FOC_SetRefI(int32_t IqRef_mA, int32_t IdRef_mA);
void FOC_GetI(int32_t* pIq_mA, int32_t* pId_mA);

#endif /* BSP_DRIVER_BLDC_FOC_CONTROLLER_H_ */
