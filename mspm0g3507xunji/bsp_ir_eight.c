#include "Delay.h"
#include "bsp_ir_eight.h"
#include "Delay.h"
void GPIO_setPins(GPIO_Regs* gpio, uint32_t pins,uint8_t val)
{
	if(val != 0)
	{
			 DL_GPIO_setPins(gpio, pins);
	}
	else
				DL_GPIO_clearPins(gpio, pins);
}


void ReadEightIR(uint8_t ir_results[8]) {

    for (int channel = 0; channel < 8; channel++) {
        // ���㵱ǰͨ����Ӧ�� C��B��A ֵ��C �����λ��A �����λ��
        uint8_t c = (channel >> 2) & 0x01;
        uint8_t b = (channel >> 1) & 0x01;
        uint8_t a = channel & 0x01;       
        SET_CHANNEL(c, b, a); // ����ͨ��ѡ������  
        Delay_ms(1); 
        
        // ��ȡ��ǰͨ���ĺ���״̬������������
        ir_results[channel] = READ_IR_OUT() ? 1 : 0;
    }
}