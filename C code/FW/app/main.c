#include "sdk_inc.h"
#include "options.h"
#include "powermanager.h"

#include "engine/engine_bldc.h"

void HardFault_Handler(void)
{
	DEBUG_ERROR("", "H");
#ifdef DEBUG
	while(1)
	{
	}
#else
	pwr_reset();
#endif
}

bool StartApp(void)
{
	//DEBUG_PRINTL("main", "RUN");
	ENGINE_Debug();
	//time_delay(250);
	return true;
}

int main(void)
{
	pwr_reset_mode_t rm = pwr_get_reset_mode();
	board_init();

#ifdef DEBUG
	//debug_enable_trace(DEBUG_TRACE_UART_INSTANCE, DEBUG_TRACE_UART_BAUDRATE, NULL, 0);
#endif
	basetimer_init(BASE_TIMER_MANAGER_INSTANCE, 1000000, 2000, IRQ_PRIORITY_TIMEBASE);
	DEBUG_PRINTL("main", "START (rm=%d)", rm);

	PowerInit(StartApp);
	rtos_start();
	return 0;
}

