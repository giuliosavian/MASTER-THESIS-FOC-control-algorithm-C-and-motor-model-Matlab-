/*
 * pid.c
 *
 *  Created on: 22 mag 2018
 *      Author: fpasquetto
 */
#include "sdk_inc.h"
#include "pid.h"
#include "motor_def.h"


/* Define rule base */
typedef struct
{
	float        lowlimit;        // low limit, membership 0.0
	float        highlimit;       // high limit, membership 1.0
	float        weighting;       // weighting factor for inference combination
} pid_rule_t;

/*------------------------------------------------------------------*/
typedef struct {
	float pid_last;
	float pid_ctrl;
	float pid_accu;
	float pid_p_startweight;
	float pid_d_startweight;
} pid_handler_t;


pid_handler_t g_pid;
pid_rule_t   P_rule = { -P_LIMIT, P_LIMIT, P_RULE_WEIGHT };
pid_rule_t   I_rule = { -I_LIMIT, I_LIMIT, I_RULE_WEIGHT };
pid_rule_t   D_rule = { -D_LIMIT, D_LIMIT, D_RULE_WEIGHT };
pid_rule_t   O_rule = { -O_LIMIT, O_LIMIT, O_RULE_WEIGHT };


/*------------------------------------------------------------------*/
/*Core */
/*------------------------------------------------------------------*/
// Evaluate "is member" and "is not member" of input value, given rule
// to apply.  Accumulates results with previous results. Takes advantage
// of special complementation properties of this rule set.
void Fuzzy_PID_evaluate_support(float variable, pid_rule_t *pRule, float *isMember, float *isNotMember)
{
	float  in_set;
	if( variable >= pRule->highlimit )
		in_set = 1.0;
	else if( variable <= pRule->lowlimit )
		in_set = 0.0;
	else
		in_set = (variable - pRule->lowlimit) /	(pRule->highlimit - pRule->lowlimit);
	*isMember    += in_set * pRule->weighting ;
	*isNotMember += (1.0 - in_set) * pRule->weighting ;
}

// Normalize and evaluate output
float Fuzzy_PID_defuzzify(pid_rule_t *pRule, float isMember, float isNotMember)
{
	float  output;
	output = (isMember * pRule->highlimit) + (isNotMember * pRule->lowlimit);
	if( output > pRule->highlimit )  output = pRule->highlimit;
	if( output < pRule->lowlimit )   output = pRule->lowlimit;
	return output;
}

/*
 PID core !
  input :
   Err = Input Error                   [ Vref - Vmeas]
   AccumErr = Integrative error        [AccumErr += Err]
   Speed  = derivative error           [ Speed = Vmeas - Vmeas[-1] ]
 */
float Fuzzy_PID_compute(float err, float accumErr, float diffErr)
{
	float isMember = 0.0;
	float isNotMember = 0.0;
	Fuzzy_PID_evaluate_support( err,      &P_rule, &isMember, &isNotMember );
	Fuzzy_PID_evaluate_support( accumErr, &I_rule, &isMember, &isNotMember );
	//Fuzzy_PID_evaluate_support( diffErr,  &D_rule, &isMember, &isNotMember );
	return Fuzzy_PID_defuzzify( &O_rule, isMember, isNotMember );
}


/*------------------------------------------------------------------*/
/*Interface */
/*------------------------------------------------------------------*/
void PID_SetParam(PID_PARAM_T param, float limit, float weighting)
{
	switch (param)
	{
	case MPID_P_IDX :
		P_rule.lowlimit = -limit;
		P_rule.highlimit = limit;
		P_rule.weighting= weighting;
		break;
	case MPID_I_IDX :
		I_rule.lowlimit = -limit;
		I_rule.highlimit = limit;
		I_rule.weighting= weighting;
		break;
	case MPID_D_IDX :
		D_rule.lowlimit = -limit;
		D_rule.highlimit = limit;
		D_rule.weighting= weighting;
		break;
	case MPID_O_IDX :
		O_rule.lowlimit = -limit;
		O_rule.highlimit = limit;
		O_rule.weighting = weighting;
		break;
	}
}

void PID_Reset(void)
{
	g_pid.pid_ctrl = 0;
	g_pid.pid_last =0;
	g_pid.pid_accu = 0;
	g_pid.pid_p_startweight = P_rule.weighting;
	g_pid.pid_d_startweight = D_rule.weighting;
}


#define MAX_PID_ERROR   (float)500
#define MIN_PID_ERROR   (float)100
float PID_Pro(float request, float measured, float maxout)
{
	float pid_err;
	float pid_d;
	float pid_out;

	pid_err = request - measured;

//	if ( pid_err > MAX_PID_ERROR)
//		pid_err = MAX_PID_ERROR;
//	else if ( pid_err < -MAX_PID_ERROR)
//		pid_err = -MAX_PID_ERROR;

	if( g_pid.pid_ctrl >= maxout || g_pid.pid_ctrl <= -maxout )
	{
		/* Saturated */
		g_pid.pid_accu += 0;
//		if( P_rule.weighting > 3.0 )
//			P_rule.weighting -= 1;
//		if( D_rule.weighting > 5.0 )
//			D_rule.weighting -= 1;
	}
	else
	{
		g_pid.pid_accu += pid_err;
	}
	pid_d = pid_err - g_pid.pid_last;
	g_pid.pid_last = pid_err;

	pid_out = Fuzzy_PID_compute(pid_err, g_pid.pid_accu, pid_d);
	g_pid.pid_ctrl = pid_out; // / (float)PLANT_GAIN_SCALE;

	if ( g_pid.pid_ctrl > maxout)
		g_pid.pid_ctrl = maxout;
	else  if ( g_pid.pid_ctrl < -maxout)
		g_pid.pid_ctrl = -maxout;

	return( g_pid.pid_ctrl);
}
