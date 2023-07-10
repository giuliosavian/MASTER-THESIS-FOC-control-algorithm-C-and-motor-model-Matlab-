#ifndef _MODULE_BUTTONS_H_
#define _MODULE_BUTTONS_H_

/* Value associated with the buttons for controlling the display  */
typedef enum {
	BUTTON_ANY = 0,

	BUTTON_ENTER = 			0x0001,     		/* Pressed key ENTER	*/
	BUTTON_RELEASED_ENTER = 0x1001,
	BUTTON_HOLDED_ENTER = 	0x2001,
} BUTTON_KEY_T;

void BUTTON_Init(void);
void BUTTON_Deinit(void);
bool BUTTON_Scan(void);
BUTTON_KEY_T BUTTON_Get(void);
uint32_t BUTTON_DoubleArrowPressedTime(void);

#endif /* _MODULE_BUTTONS_H_ */
