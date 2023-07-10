/*
 * buttons.c
 *
 *  Created on: 10 ott 2017
 *      Author: fpasquetto
 */
#include "sdk_inc.h"
#include "module_buttons.h"
#include "powermanager.h"

/* Hold press time */
#define PRESS_TIME_MS		50
#define RELEASE_TIME_MS		50
#define HOLD_TIME_MS		500

/* Key state */
#define KSTATE_IDLE			0x00
#define KSTATE_EVENT		0x01
#define KSTATE_PRESSED		0x02
#define KSTATE_HOLDED		0x03
#define KSTATE_RELEASED		0x10

#define KEY_ENTER 			0x01       	/* ENTER	*/
#define BUTTON_KEY_COUNT	1
#define BUTTON_KEY_BUFF		2

static uint32_t g_first_time[BUTTON_KEY_COUNT];
static uint8_t g_press_state[BUTTON_KEY_COUNT];

static BUTTON_KEY_T g_keybuff[BUTTON_KEY_BUFF];
static uint8_t g_keybuffindexw;
static uint8_t g_keybuffindexr;

static void signal_button_status_changed(void)
{
	PowerTaskSignalEvent(PWM_TASK_EVENT_BUTTONS);
}

static void _setkey_press(int i)
{
	if( g_keybuffindexw < BUTTON_KEY_BUFF )
		g_keybuff[g_keybuffindexw++] = (BUTTON_KEY_T)(BUTTON_ENTER + i);
	signal_button_status_changed();
}

static void _setkey_release(int i)
{
	if( g_keybuffindexw < BUTTON_KEY_BUFF )
		g_keybuff[g_keybuffindexw++] = (BUTTON_KEY_T)(BUTTON_RELEASED_ENTER + i);
	signal_button_status_changed();
}

static void _setkey_hold(int i)
{
	if( g_keybuffindexw < BUTTON_KEY_BUFF )
		g_keybuff[g_keybuffindexw++] = (BUTTON_KEY_T)(BUTTON_HOLDED_ENTER + i);
	signal_button_status_changed();
}

static BUTTON_KEY_T _getkey(void)
{
	BUTTON_KEY_T k = BUTTON_ANY;
	if( g_keybuffindexr < g_keybuffindexw )
	{
		k = g_keybuff[g_keybuffindexr++];
		if( g_keybuffindexr == g_keybuffindexw )
		{
			g_keybuffindexr = g_keybuffindexw = 0;
		}
	}
	return k;
}

/*
 * Da evitare le seguenti combinazioni:
 * - ENTER + KEFT
 * - ESC + RIGHT
 */

static basetimer_return_t tick_button_work(void* p)
{
	int i;
	uint8_t key = 0;
	int32_t k = gpio_read_pin_input(kButton);

	if( k == 0 )
		key |= KEY_ENTER;

	for( i = 0; i < BUTTON_KEY_COUNT; i++ )
	{
		if( g_press_state[i] == KSTATE_IDLE )
		{
			/* inactive */
			if( key & (0x01<<i) )
			{
				g_first_time[i] = time_get();
				g_press_state[i] = KSTATE_EVENT;
			}
		}
		else if( (g_press_state[i]&0xF0) == KSTATE_RELEASED )
		{
			/* Check if actually released */
			if( key & (0x01<<i) )
			{
				/* Not released - return to previous state */
				g_press_state[i] &= 0x0F;
			}
			else
			{
				if( time_elapsed(g_first_time[i]) >= RELEASE_TIME_MS )
				{
					/* Actually released */
					g_press_state[i] = KSTATE_IDLE;
					_setkey_release(i);
				}
			}
		}
		else
		{
			/* pressed */
			if( (key & (0x01<<i)) == 0 )
			{
				if( g_press_state[i] == KSTATE_PRESSED || g_press_state[i] == KSTATE_HOLDED )
				{
					g_first_time[i] = time_get();
					g_press_state[i] |= KSTATE_RELEASED;
				}
				else
					g_press_state[i] = KSTATE_IDLE;
			}
			else if( g_press_state[i] == KSTATE_EVENT )
			{
				if( time_elapsed(g_first_time[i]) >= PRESS_TIME_MS )
				{
					g_press_state[i] = KSTATE_PRESSED;
					_setkey_press(i);
				}
			}
			else if( g_press_state[i] == KSTATE_PRESSED )
			{
				if( time_elapsed(g_first_time[i]) >= HOLD_TIME_MS )
				{
					g_press_state[i] = KSTATE_HOLDED;
					_setkey_hold(i);
				}
			}
		}
	}
	return BASETIMER_RETURN_RESTART;
}

BUTTON_KEY_T BUTTON_Get(void)
{
	return _getkey();
}

void BUTTON_Init(void)
{
	basetimer_start(tick_button_work, NULL, 10);
}

void BUTTON_Deinit(void)
{
	basetimer_remove(tick_button_work, NULL);
}

