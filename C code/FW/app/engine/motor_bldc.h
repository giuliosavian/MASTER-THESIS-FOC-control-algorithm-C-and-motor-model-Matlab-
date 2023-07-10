/*
 * motor.c
 *
 *  Created on: 11 mag 2018
 *      Author: fpasquetto
 */

#ifndef APP_ENGINE_MOTOR_BLDC_H_
#define APP_ENGINE_MOTOR_BLDC_H_

#include "engine/motor_def.h"

void MOTOR_Init(void);
void MOTOR_Start(MOTOR_DIRECTION_T dir);
void MOTOR_Stop(bool release);

int32_t MOTOR_GetImot(void);
void 	MOTOR_SetIref(int32_t mA);
int32_t MOTOR_GetSpeed(void);

bool 	 MOTOR_NewStepIn(void);
bool 	 MOTOR_HallDisconnected(void);
bool 	 MOTOR_IlimIsActive(void);
uint16_t MOTOR_GetSampledSpeed(void);
uint32_t MOTOR_GetVmot(void);
uint32_t MOTOR_GetIline(void);
uint16_t MOTOR_GetDirection(void);
void 	 MOTOR_SetIlim(int32_t curlimit);
uint32_t MOTOR_GetILim(void);
bool 	 MOTOR_IsOn(void);
void 	 MOTOR_TurnOff(void);
bool 	 MOTOR_IsOff(void);
bool MOTOR_Stopping(void);
void MOTOR_SetSpeedFilterCoeff(int32_t coeff);
void MOTOR_Debug(void);
void MOTOR_Tick(void);


#endif /* APP_ENGINE_MOTOR_BLDC_H_ */
