#ifndef _ENGINE_ENGINE_H_
#define _ENGINE_ENGINE_H_

#include "engine/motor_def.h"

// Request defines
typedef enum {
	ENGINE_REQUEST_ANY = 0,
	ENGINE_REQUEST_OPEN,
	ENGINE_REQUEST_CLOSE,
	ENGINE_REQUEST_STOP,
	ENGINE_REQUEST_HOLD,
} ENGINE_REQUEST_T;

// Action time defines for inversions
typedef enum {
	ENGINE_RACE_UNKNOWN = 0,				/* Unknown position */
	ENGINE_RACE_NORMAL,						/* Normal race with current profile monitor */
	ENGINE_RACE_WITHOUT_ENCODER,			/* Race without encoder */
	ENGINE_RACE_WITHOUT_PROFILE,			/* Normal race without current profile monitor */
	ENGINE_RACE_CALIBRATION_ENCODER,		/* Encoder calibration race */
	ENGINE_RACE_SAVE_CLOSE_PROFILE,			/* Not real race mode: only save current profile */
	ENGINE_RACE_SAVE_OPEN_PROFILE,			/* Not real race mode: only save current profile */
	ENGINE_RACE_ENDRACE_APPROACH,			/* Accostamento al finecorsa meccanico */
} ENGINE_RACE_MODE_T;

// Motor driver events
typedef enum {
	ENGINE_EVENT_ANY = 0,
	ENGINE_EVENT_MOTOR_STOPPED = 0x01,		/* signal the motor is actually stopped */
	ENGINE_EVENT_ENCODER_FAULT = 0x02,		/* signal an encoder fault detected */
	ENGINE_EVENT_OBSTACLE_DETECTI = 0x04,	/* signal an obstacle has been detected by motor current */
	ENGINE_EVENT_OBSTACLE_DETECTC = 0x08,	/* signal an obstacle has been detected by costa detector */
	ENGINE_EVENT_OBSTACLE_DETECTB = 0x10,	/* signal an obstacle has been detected by break detector */
} ENGINE_EVENTS_T;

// Callback function
typedef void (*engine_callback_t)(uint8_t event);

//! GATE main states
typedef enum {
	ENGINE_STATE_STOP = 0,
	ENGINE_STATE_OPENING,
	ENGINE_STATE_CLOSING,
	ENGINE_STATE_STOPPING,
//	ENGINE_STATE_TEST,
} ENGINE_STATE_T;

//! GATE main states
typedef enum {
	ENGINE_TEST_ENTER = 0,
	ENGINE_TEST_EXIT,
	ENGINE_TEST_STOP,
	ENGINE_TEST_START_CW,
	ENGINE_TEST_START_CCW,
	ENGINE_TEST_SET_RPM,
	ENGINE_TEST_SET_OFFSET_FIXED,
	ENGINE_TEST_SET_OFFSET_CW,
	ENGINE_TEST_SET_OFFSET_CCW,
} ENGINE_TEST_CMD_T;

/*
 * Configurations
 */
typedef struct {
	uint16_t startopencoeff;
	uint16_t startclosecoeff;
	uint16_t opencoeff;
	uint16_t closecoeff;
	uint16_t lowspeedcoeff;
	uint16_t pidstartval;
	uint16_t pidminval;
	uint16_t minopoutpwr;
	uint16_t mincloutpwr;
	uint16_t startoppwr;
	uint16_t startclpwr;
	uint16_t pidsresol;
	uint16_t pidtimeoutstart;
	uint16_t pidtimeout;
	uint16_t starttime;			/* time starting an open or close operation when the maximum power is limited */
	uint16_t costa_tstr;		/* time starting an open or close operation when the checking obstacles by speed change is disabled */
	uint16_t costa_tout;		/* timeout on speed change in ms to detect an obstacle (at 100% sensibility) */
	uint16_t ilimtout;			/* timeout on current reduction in ms to detect an obstacle (at 100% sensibility) */
	uint16_t speedcoeff;
	float    pid_p;
	float    pid_d;
	float    pid_i;
} engine_config_t;

/*
 * @brief Initialize the motor driver
 * @param cb Callback function to signal motor events
 */
void ENGINE_Init(engine_callback_t cb);

/*
 * @brief Set the current profile sensibility.
 */
void ENGINE_SetObstacleSens(uint8_t obstacle_sens);

/*
 * @brief Return the current set speed as a value from 0 to 1024
 * @return The speed percent
 */
uint16_t ENGINE_GetSpeedFactor(void);
uint16_t ENGINE_GetSpeedRpm(void);
uint16_t ENGINE_GetMotorDirection(void);

/**
 * Return the last event error (ENGINE_EVENTS_T)
 */
uint16_t ENGINE_GetLastErrorEvent(void);

/*
 * @brief Set a race mode
 * @param mode The race mode
 */
void ENGINE_SetRaceMode(ENGINE_RACE_MODE_T mode);

/*
 * @brief Set the encoder pulses
 * @param num The encoder pulses to set
 */
void ENGINE_SetEncoder(int32_t num);

/*
 * @brief Get the current encoder pulses
 * @return The encoder pulses
 */
int32_t ENGINE_GetEncoder(void);

/*
 * @brief Request an action on the motor driver
 *
 * The actions can be:
 * - GATE_REQUEST_OPENING Tell to the driver to open the gate
 * - GATE_REQUEST_CLOSING Tell to the driver to close the gate
 * - GATE_REQUEST_STOP Tell to the driver to stop the gate
 *
 * @param request The action to perform
 * @param speed_factor The gate speed as a value from 0 to 1024
 */
void ENGINE_Request(ENGINE_REQUEST_T request, uint16_t speed_factor, uint16_t actiontime);

/*
 * @brief Return current engine state
 */
ENGINE_STATE_T ENGINE_GetState(void);

/*
 * @brief Return if encoder is not working or disconnected
 */
bool ENGINE_EncoderFault(void);

/*
 * @brief Return current motor current in mA
 */
uint32_t ENGINE_GetMotorCurrent(void);

/*
 * @brief Return current motor voltage in mV
 */
uint32_t ENGINE_GetMotorVoltage(void);

/*
 * @brief Return current line current in mA
 */
uint32_t ENGINE_GetLineCurrent(void);

/*
 * @brief Emergency stop
 */
void ENGINE_EmergencyStop(void);

/*
 * Update the motor type value from the configuration
 */
void ENGINE_UpdateMotorType(void);



/* DEBUG */
void ENGINE_Debug(void);
void ENGINE_DebugTraceEvent(uint16_t position);

#endif /* _ENGINE_ENGINE_H_ */
