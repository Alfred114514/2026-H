#ifndef __KEY_H
#define __KEY_H

#include "main.h"
#include "watch.h"

typedef enum
{
	None = 0,
	SW2 = 1,
	SW3 = 2,
	SW4 = 3,
	SW2_L,
	SW3_L,
	SW4_L
} KeyNameTypeDef;

KeyNameTypeDef Key_GetKeyNum(void);

#endif
