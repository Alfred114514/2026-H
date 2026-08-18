#ifndef __BSP_IR_EIGHT_H__
#define __BSP_IR_EIGHT_H__
#include "ti_msp_dl_config.h"
#include "ti_msp_dl_config.h"
// ��·�����·ѡ���������Ŷ���
#define EIGHT_IR_PORT        Eight_IR_PORT   // �˿�
#define EIGHT_IR_AD0_PIN     Eight_IR_AD0_X1_PIN    // ͨ��ѡ��A�����λ��
#define EIGHT_IR_AD1_PIN     Eight_IR_AD1_X2_PIN    // ͨ��ѡ��B
#define EIGHT_IR_AD2_PIN     Eight_IR_AD2_X3_PIN    // ͨ��ѡ��C�����λ��
#define EIGHT_IR_OUT_PIN     Eight_IR_OUT_X4_PIN    // �������������


// ����ͨ��ѡ������ C��B��A �ĵ�ƽ��0 �� 1��
#define SET_CHANNEL(c_val, b_val, a_val) \
    do { \
        GPIO_setPins(EIGHT_IR_PORT, EIGHT_IR_AD2_PIN, c_val); \
        GPIO_setPins(EIGHT_IR_PORT, EIGHT_IR_AD1_PIN, b_val); \
        GPIO_setPins(EIGHT_IR_PORT, EIGHT_IR_AD0_PIN, a_val); \
    } while(0)


// ��ȡ�����������״̬������ 1 �� 0��
#define READ_IR_OUT()  DL_GPIO_readPins(EIGHT_IR_PORT, EIGHT_IR_OUT_PIN)

		
		
void GPIO_setPins(GPIO_Regs* gpio, uint32_t pins,uint8_t val);
void ReadEightIR(uint8_t ir_results[8]);
		

#endif






