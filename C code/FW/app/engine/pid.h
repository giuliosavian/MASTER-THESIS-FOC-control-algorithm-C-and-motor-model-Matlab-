/*
 * pid.h
 *
 *  Created on: 22 mag 2018
 */

#ifndef _ENGINE_PID_H_
#define _ENGINE_PID_H_

#define PID_VALUES		1
typedef enum {
	MPID_P_IDX = 0,    /// prop
	MPID_I_IDX = 1,    /// integrative
	MPID_D_IDX = 2,    /// derivative
	MPID_O_IDX = 3,    /// Output
} PID_PARAM_T;

/* default parameters */

// Membership bounds on input and output levels
#if PID_VALUES == 1
	#define  P_LIMIT         	20000.0
	#define  I_LIMIT         	400000.0  //168525824
	#define  D_LIMIT          	10000.0
	#define  O_LIMIT          	20000.0
	#define  P_RULE_WEIGHT     	6.00
	#define  I_RULE_WEIGHT     	0.001   //// 0.0001
	#define  D_RULE_WEIGHT     	0 //6.01  /// 3.00
	#define  O_RULE_WEIGHT     	0.125
#endif

#if PID_VALUES == 2
	#define  P_LIMIT         	20000.0
	#define  I_LIMIT         	400000.0
	#define  D_LIMIT          	10000.0
	#define  O_LIMIT          	20000.0
	#define  P_RULE_WEIGHT     	10.00
	#define  I_RULE_WEIGHT     	0.1
	#define  D_RULE_WEIGHT     	6.01  /// 3.00
	#define  O_RULE_WEIGHT     	0.125
#endif

#if PID_VALUES == 3
/*
 * Ku = 3.0
 * Tu = 0.065s
 * kp = 0.6 ku = 2.88,Ti = 0.5 Tu = 1.11, Td = 0.125Tu = 0.277
 * -----------------------------------------------------------
 * Ki = Kp/Ti = Kp/(0.5Tu)
 * Kd = Kp*Td = Kp*0.125Tu
 */
#define Ku	8.0
#define Tu  50

	#define  P_LIMIT			5000.0
	#define  D_LIMIT			5000.0
	#define  I_LIMIT			40000.0
	#define  O_LIMIT			20000.0
	#define  P_RULE_WEIGHT		(0.6*Ku)
	#define  D_RULE_WEIGHT		(P_RULE_WEIGHT*(Tu*0.125))
	#define  I_RULE_WEIGHT		(P_RULE_WEIGHT/(Tu*0.5))
	#define  O_RULE_WEIGHT		0.125
#endif

/*
 * Ku = 0.5
 * Tu = 0.065s
 * kp = 0.6 ku = 2.88,Ti = 0.5 Tu = 1.11, Td = 0.125Tu = 0.277
 * -----------------------------------------------------------
 * Ki = Kp/Ti = Kp/(0.5Tu)
 * Kd = Kp*Td = Kp*0.125Tu
 */
#if PID_VALUES == 4
	#define  P_LIMIT         	20000.0
	#define  I_LIMIT         	400000.0  //168525824
	#define  D_LIMIT          	10000.0
	#define  O_LIMIT          	20000.0
	#define  P_RULE_WEIGHT     	6.00
	#define  I_RULE_WEIGHT     	0.001   //// 0.0001
	#define  D_RULE_WEIGHT     	0 //6.01  /// 3.00
	#define  O_RULE_WEIGHT     	0.125
#endif

#define PLANT_GAIN_SCALE		200		// 156		(output : -O_LIMIT  +O_LIMIT: 20000 / 128 = 156)


void  PID_SetParam(PID_PARAM_T index, float limit, float weighting);
float PID_Pro(float request, float measured, float maxout);
void  PID_Reset(void);


#endif /* _ENGINE_PID_H_ */
