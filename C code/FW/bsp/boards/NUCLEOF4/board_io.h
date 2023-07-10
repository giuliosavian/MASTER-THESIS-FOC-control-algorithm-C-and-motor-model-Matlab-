#ifndef __BOARD_IO_NUCLEOF4_H
#define __BOARD_IO_NUCLEOF4_H

#include "driver/inc/driver_gpio.h"

enum _gpio_pins
{
    kDbgLed				= GPIO_MAKE_PIN(GPIO_PORT_A, 5),	// Debug led
	kMotCurW			= GPIO_MAKE_PIN(GPIO_PORT_C, 0),	// ADC1_IN10
	kMotCurV			= GPIO_MAKE_PIN(GPIO_PORT_C, 1),	// ADC1_IN11
	kMotCurU			= GPIO_MAKE_PIN(GPIO_PORT_A, 0),	// ADC1_IN0
	kMotTemp			= GPIO_MAKE_PIN(GPIO_PORT_C, 2),	// ADC1_IN12
	kBusVolt			= GPIO_MAKE_PIN(GPIO_PORT_A, 1),	// ADC1_IN1
    kSrvComTx  			= GPIO_MAKE_PIN(GPIO_PORT_A, 2),  	// UART2_TX
    kSrvComRx  			= GPIO_MAKE_PIN(GPIO_PORT_A, 3),  	// UART2_RXPA4
	kButton				= GPIO_MAKE_PIN(GPIO_PORT_C, 13),  	// USER BUTTON
	kPotent				= GPIO_MAKE_PIN(GPIO_PORT_A, 4),  	// ADC1_IN4

	kMotOcp				= GPIO_MAKE_PIN(GPIO_PORT_A, 6),  	// TIM1_BKIN
	kMotPwmUL			= GPIO_MAKE_PIN(GPIO_PORT_A, 7),  	// TIM1_CH1N
	kMotPwmVL			= GPIO_MAKE_PIN(GPIO_PORT_B, 0),  	// TIM1_CH2N
	kMotPwmWL			= GPIO_MAKE_PIN(GPIO_PORT_B, 1),  	// TIM1_CH3N
	kMotPwmUH			= GPIO_MAKE_PIN(GPIO_PORT_A, 8),  	// TIM1_CH1
	kMotPwmVH			= GPIO_MAKE_PIN(GPIO_PORT_A, 9),  	// TIM1_CH2
	kMotPwmWH			= GPIO_MAKE_PIN(GPIO_PORT_A, 10),  	// TIM1_CH3

	kEncHall1			= GPIO_MAKE_PIN(GPIO_PORT_A, 15),  	// TIM2_CH1
	kEncHall2			= GPIO_MAKE_PIN(GPIO_PORT_B, 3),  	// TIM2_CH2
	kEncHall3			= GPIO_MAKE_PIN(GPIO_PORT_B, 10),  	// TIM2_CH3
};

extern const gpio_pin_user_config_t gpioStartupPinsConfiguration[];

/**
 * @brief Uart instance to use for debug trace
 */
#define DEBUG_TRACE_UART_INSTANCE		UART_MAX_INSTANCES
#define DEBUG_TRACE_UART_BAUDRATE		500000

/**
 * @brief Timer instance to use for the base timer
 */
#define BASE_TIMER_MANAGER_INSTANCE		TIMER_INSTANCE_3

#define MOTOR_DRIVER_TIMER_INSTANCE		TIMER_INSTANCE_1

#define MOTOR_CURRENT_ADC_INSTANCE		ADC_INSTANCE_1
#define MOTOR_CURRENT_ADC_CHANNEL_A		ADC_CH_10
#define MOTOR_CURRENT_ADC_CHANNEL_B		ADC_CH_11
#define MOTOR_CURRENT_ADC_CHANNEL_C		ADC_CH_0

#define BOARD_ADC_INSTANCE				ADC_INSTANCE_1
#define BOARD_ADC_MOTOR_VOLTAGE_CHANNEL	ADC_CH_1
#define BOARD_ADC_INPUT_TEMP_CHANNEL	ADC_CH_12
#define BOARD_ADC_TRIGGER_TIMER			ADC_CONF_TRIGGER_MODE_MANUAL

#define BOARD_ADC_INSTANCE				ADC_INSTANCE_1
#define BOARD_ADC_POTENTIOMETER      	ADC_CH_4


#endif /* __BOARD_IO_NUCLEOF4_H */
