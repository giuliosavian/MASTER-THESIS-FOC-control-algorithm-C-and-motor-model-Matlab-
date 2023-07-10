#ifndef __BOARD_DEF_NUCLEOF4_H
#define __BOARD_DEF_NUCLEOF4_H

#include "platform/cpu_def.h"
#include "rtos/rtos_def.h"

/**
 * @brief RTOS to use in the SDK
 *
 * Define the RTOS to use :
 * RTOS_nortos -> do not use any RTOS
 * RTOS_freertos -> use FreeRTOS
 */
#define RTOS RTOS_nortos
//#define RTOS RTOS_freertos

/**
 * @brief External Oscillator frequency
 *
 * Define the external oscillator frequency in Hz
 * Set to 0 if no external oscillator is used
 */
#define BOARD_EXTERNAL_OSC_VALUE		8000000

/**
 * @brief Platform name
 *
 * Define the platform name using one of the
 * defines in platform/cpu_def.h
 */
#define BOARD_PLATFORM					PLATFORM_STM32F4

/**
 * @brief CPU name
 *
 * Define the cpu name using one of the
 * defines in platform/cpu_def.h
 */
#define BOARD_CPU						CPU_STM32F401xE

/**
 * @brief CPU core
 *
 * Define the cpu core using one of the
 * defines in platform/cpu_def.h
 */
#define BOARD_CPU_CORE					CORE_CORTEX_M4


/**
 * @brief CPU manufacturer brand and part-number
 *
 * Define the cpu manufacturer brand and the complete part-number
 * using free text (these information are not used by the compiler)
 */
#define BOARD_CPU_BRAND		        	ST
#define BOARD_CPU_PARTNUMBER        	STM32F401RE

/**
 * @brief Use of bootloader
 *
 * Define to 1 if the bootloader is used
 */
#define BOARD_USE_BOOTLOADER			0
#define BOARD_BOOTLOADER_SIZE			0x2000

/**
 * @brief Debugger definitions: use of semihosting
 *
 * Define to 1 if the debug trace will not use a peripheral UART
 * but it will use the debugger semihosting channel.
 */
#define BOARD_DEBUG_USE_SEMIHOSTING		0

/**
 * @brief Use of classB self-test routines
 *
 * Define to 1 if the self-test routines must be used
 */
#define BOARD_USE_CLASSB_SELFTEST		0



#endif /* __BOARD_DEF_NUCLEOF3_H */
