/*
 * encoder.h
 *
 *  Created on: 11 mag 2018
 *      Author: fpasquetto
 */

#ifndef APP_ZLB30_ENGINE_ENCODER_H_
#define APP_ZLB30_ENGINE_ENCODER_H_

#include <stdint.h>
#include "engine/motor_def.h"

void ENCODER_SetOpenDir(MOTOR_DIRECTION_T dir);
void ENCODER_Update(int32_t stp, uint32_t period);
void ENCODER_SetPulse(int32_t num);
int32_t ENCODER_GetPartialPulse(void);
int32_t ENCODER_GetPulse(void);
int32_t ENCODER_GetRemeaningSteps(int32_t max_steps);
int32_t ENCODER_GetElapsedSteps(int32_t from_step);

void ENCODER_Test_ResetData(void);
void ENCODER_Test_GettData(int32_t* pOutPulses, uint32_t* pOutTime);

#endif /* APP_ZLB30_ENGINE_ENCODER_H_ */
