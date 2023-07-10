#include "sdk_config.h"
#include "driver/inc/driver_gpio.h"
#include "platform/STM32F4/hll/STM32F4xx_HAL/Inc/stm32f4xx_hal_gpio_ex.h"

const gpio_pin_user_config_t gpioStartupPinsConfiguration[] = {
	{	.Name = kDbgLed,	.Mode = GPIO_CONF_OUTPUT,											.Alternate = 0					},
	{	.Name = kSrvComTx,	.Mode = (GPIO_CONF_ALTERNATE|GPIO_CONF_PULLUP),						.Alternate = GPIO_AF7_USART2	},
	{	.Name = kSrvComRx,	.Mode = (GPIO_CONF_ALTERNATE|GPIO_CONF_PULLUP),						.Alternate = GPIO_AF7_USART2	},
	{	.Name = kButton,	.Mode = GPIO_CONF_INPUT|GPIO_CONF_PULLUP,							.Alternate = 0					},
	{	.Name = kPotent,	.Mode = GPIO_CONF_ANALOG,									        .Alternate = 0					},

	{	.Name = kMotCurW,	.Mode = GPIO_CONF_ANALOG, 											.Alternate = 0	},
	{	.Name = kMotCurV,	.Mode = GPIO_CONF_ANALOG, 											.Alternate = 0	},
	{	.Name = kMotCurU,	.Mode = GPIO_CONF_ANALOG, 											.Alternate = 0	},
	{	.Name = kMotTemp,	.Mode = GPIO_CONF_ANALOG, 											.Alternate = 0	},
	{	.Name = kBusVolt,	.Mode = GPIO_CONF_ANALOG, 											.Alternate = 0	},

	{	.Name = kMotOcp,	.Mode = GPIO_CONF_ALTERNATE|GPIO_CONF_PULLUP,						.Alternate = GPIO_AF1_TIM1	},
	{	.Name = kMotPwmUL,	.Mode = GPIO_CONF_ALTERNATE|GPIO_CONF_PULLUP|GPIO_CONF_SPEED_HIGH,	.Alternate = GPIO_AF1_TIM1	},
	{	.Name = kMotPwmVL,	.Mode = GPIO_CONF_ALTERNATE|GPIO_CONF_PULLUP|GPIO_CONF_SPEED_HIGH,	.Alternate = GPIO_AF1_TIM1	},
	{	.Name = kMotPwmWL,	.Mode = GPIO_CONF_ALTERNATE|GPIO_CONF_PULLUP|GPIO_CONF_SPEED_HIGH,	.Alternate = GPIO_AF1_TIM1	},
	{	.Name = kMotPwmUH,	.Mode = GPIO_CONF_ALTERNATE|GPIO_CONF_PULLDOWN|GPIO_CONF_SPEED_HIGH,.Alternate = GPIO_AF1_TIM1	},
	{	.Name = kMotPwmVH,	.Mode = GPIO_CONF_ALTERNATE|GPIO_CONF_PULLDOWN|GPIO_CONF_SPEED_HIGH,.Alternate = GPIO_AF1_TIM1	},
	{	.Name = kMotPwmWH,	.Mode = GPIO_CONF_ALTERNATE|GPIO_CONF_PULLDOWN|GPIO_CONF_SPEED_HIGH,.Alternate = GPIO_AF1_TIM1	},

	{	.Name = kEncHall1,	.Mode = GPIO_CONF_ALTERNATE|GPIO_CONF_SPEED_HIGH,					.Alternate = GPIO_AF1_TIM2	},
	{	.Name = kEncHall2,	.Mode = GPIO_CONF_ALTERNATE|GPIO_CONF_SPEED_HIGH,					.Alternate = GPIO_AF1_TIM2	},
	{	.Name = kEncHall3,	.Mode = GPIO_CONF_ALTERNATE|GPIO_CONF_SPEED_HIGH,					.Alternate = GPIO_AF1_TIM2	},
//	{	.Name = kEncHall1,	.Mode = GPIO_CONF_INPUT,											.Alternate = 0	},
//	{	.Name = kEncHall2,	.Mode = GPIO_CONF_INPUT,											.Alternate = 0	},
//	{	.Name = kEncHall3,	.Mode = GPIO_CONF_INPUT,											.Alternate = 0	},

  //------------------------------------------------------------------------------------------------------
	{
		.Name = GPIO_OUT_OF_RANGE,
	}
};



