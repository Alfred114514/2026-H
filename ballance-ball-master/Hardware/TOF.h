#ifndef __TOF_H
#define __TOF_H

#include "main.h"

#define TOF_SCL_GPIO_PORT			GPIOA
#define TOF_SCL_GPIO_PIN			GPIO_PIN_2
#define TOF_SDA_GPIO_PORT			GPIOA
#define TOF_SDA_GPIO_PIN			GPIO_PIN_3

#define TOF_SCL_WRITE(n)			HAL_GPIO_WritePin(TOF_SCL_GPIO_PORT, \
																								TOF_SCL_GPIO_PIN, \
																								n ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define TOF_SDA_WRITE(n)			HAL_GPIO_WritePin(TOF_SDA_GPIO_PORT, \
																								TOF_SDA_GPIO_PIN, \
																								n ? GPIO_PIN_SET : GPIO_PIN_RESET)

#define SYSRANGE__MAX_CONVERGENCE_TIME 			0x01C
#define SYSRANGE__PART_TO_PART_RANGE_OFFSET 0x024
#define SYSRANGE__RANGE_IGNORE_VALID_HEIGHT 0x025
#define SYSRANGE__RANGE_CHECK_ENABLES 			0x02D
#define SYSRANGE__THRESH_LOW								0x01A
#define SYSTEM__INTERRUPT_CONFIG_GPIO				0x014
#define SYSTEM__INTERRUPT_CLEAR							0x015
#define SYSRANGE__START											0x018
#define RESULT__RANGE_VAL										0x062
#define RESULT__RANGE_RAW										0x064
#define RESULT__INTERRUPT_STATUS_GPIO				0x04F

uint8_t TOF_ReadReg(uint16_t RegisterAdd);
void TOF_WriteReg(uint16_t RegisterAdd, uint8_t Data);

void TOF_Init(void);
void TOF_SetOffset(int8_t NewOffset);
uint16_t TOF_Measure(uint8_t FilterTime, uint32_t Runout);

#endif
