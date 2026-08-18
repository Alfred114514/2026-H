#include "TOF.h"

#define DELAY_TIME		20

int8_t offset;

void TOF_Delay(uint16_t DelayTime)
{
	for(;DelayTime > 0; DelayTime--);
}

void TOF_I2C_Start(void)
{
	TOF_SDA_WRITE(1);
	TOF_Delay(DELAY_TIME);
	TOF_SCL_WRITE(1);
	TOF_Delay(DELAY_TIME);
	TOF_SDA_WRITE(0);
	TOF_Delay(DELAY_TIME);
	TOF_SCL_WRITE(0);
	TOF_Delay(DELAY_TIME);
}

void TOF_I2C_Stop(void)
{
	TOF_SDA_WRITE(0);
	TOF_Delay(DELAY_TIME);
	TOF_SCL_WRITE(1);
	TOF_Delay(DELAY_TIME);
	TOF_SDA_WRITE(1);
	TOF_Delay(DELAY_TIME);
}

void TOF_I2C_SendByte(uint8_t Byte)
{
	uint8_t i = 0;
	for(; i < 8; i++)
	{
		if((Byte & (0x80 >> i)) == 0)
			TOF_SDA_WRITE(0);
		else
			TOF_SDA_WRITE(1);
		TOF_Delay(DELAY_TIME);
		TOF_SCL_WRITE(1);
		TOF_Delay(DELAY_TIME);
		TOF_SCL_WRITE(0);
		TOF_Delay(DELAY_TIME);
	}
}

uint8_t TOF_I2C_ReceiveByte(void)
{
	uint8_t RecVal = 0, i = 0;
	TOF_SDA_WRITE(1);
	TOF_Delay(DELAY_TIME);
	for(; i < 8; i++)
	{
		TOF_SCL_WRITE(1);
		TOF_Delay(DELAY_TIME);
		if(HAL_GPIO_ReadPin(TOF_SDA_GPIO_PORT, TOF_SDA_GPIO_PIN) == GPIO_PIN_SET)
			RecVal |= (0x80 >> i);
		TOF_SCL_WRITE(0);					//接收数据时仍然是主机控制SCL
		TOF_Delay(DELAY_TIME);
	}
	return RecVal;
}

uint8_t TOF_I2C_WaitACK(void)
{
	uint8_t ACK = 0;
	TOF_SDA_WRITE(1);
	TOF_Delay(DELAY_TIME);
	TOF_SCL_WRITE(1);
	TOF_Delay(DELAY_TIME);
	if(HAL_GPIO_ReadPin(TOF_SDA_GPIO_PORT, TOF_SDA_GPIO_PIN) == GPIO_PIN_SET)
		ACK = 1;
	TOF_SCL_WRITE(0);
	TOF_Delay(DELAY_TIME);
	return ACK;
}

void TOF_I2C_SendACK(void)
{
	TOF_SDA_WRITE(0);
	TOF_Delay(DELAY_TIME);
	TOF_SCL_WRITE(1);
	TOF_Delay(DELAY_TIME);
	TOF_SCL_WRITE(0);
	TOF_Delay(DELAY_TIME);
}	

void TOF_I2C_SendNACK(void)
{
	TOF_SDA_WRITE(1);
	TOF_Delay(DELAY_TIME);
	TOF_SCL_WRITE(1);
	TOF_Delay(DELAY_TIME);
	TOF_SCL_WRITE(0);
	TOF_Delay(DELAY_TIME);
}	

void TOF_WriteReg(uint16_t RegisterAdd, uint8_t Data)
{
	TOF_I2C_Start();
	TOF_I2C_SendByte(0x52);
	TOF_I2C_WaitACK();
	TOF_I2C_SendByte(RegisterAdd / 256);
	TOF_I2C_WaitACK();
	TOF_I2C_SendByte(RegisterAdd % 256);
	TOF_I2C_WaitACK();
	TOF_I2C_SendByte(Data);
	TOF_I2C_WaitACK();
	TOF_I2C_Stop();
}

uint8_t TOF_ReadReg(uint16_t RegisterAdd)
{
	uint8_t Val = 0;
	TOF_I2C_Start();
	TOF_I2C_SendByte(0x52);
	TOF_I2C_WaitACK();
	TOF_I2C_SendByte(RegisterAdd / 256);
	TOF_I2C_WaitACK();
	TOF_I2C_SendByte(RegisterAdd % 256);
	TOF_I2C_WaitACK();
	TOF_I2C_Stop();
	TOF_I2C_Start();
	TOF_I2C_SendByte(0x53);
	TOF_I2C_WaitACK();
	Val = TOF_I2C_ReceiveByte();
	TOF_I2C_SendNACK();
	TOF_I2C_Stop();
	return Val;
}

void TOF_Init(void)
{
	if((TOF_ReadReg(SYSRANGE__START) & 0x01) == 1)
		TOF_WriteReg(SYSRANGE__START, 0x00);
	TOF_WriteReg(SYSRANGE__START, 0x01);
	TOF_WriteReg(SYSTEM__INTERRUPT_CONFIG_GPIO, 0x04);
	TOF_WriteReg(SYSRANGE__RANGE_CHECK_ENABLES, 0x11);
	TOF_WriteReg(SYSRANGE__PART_TO_PART_RANGE_OFFSET, 15);
	TOF_WriteReg(SYSRANGE__MAX_CONVERGENCE_TIME, 0x3F);
}

//设置偏移量
void TOF_SetOffset(int8_t NewOffset)
{
	offset = NewOffset;
}

//输入滤波次数,最小为1，输出测距结果
uint16_t TOF_Measure(uint8_t FilterTime, uint32_t RunOut)
{
	uint32_t TickStart = HAL_GetTick();
	uint16_t Distance = 0;
	uint8_t i = FilterTime;
	while(i--)
	{
		while(TOF_ReadReg(RESULT__INTERRUPT_STATUS_GPIO) != 4)
		{
			if(HAL_GetTick() - TickStart > RunOut)
				break;
		}
		TOF_WriteReg(SYSTEM__INTERRUPT_CLEAR, 0x01);
		Distance += TOF_ReadReg(RESULT__RANGE_VAL) + offset;
		TOF_WriteReg(SYSRANGE__START, 0x01);
	}
	return Distance / FilterTime;
}
