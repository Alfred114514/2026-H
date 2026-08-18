#ifndef __OPENMV_H
#define __OPENMV_H

#include "main.h"
#include "stdbool.h"

/* OpenMV 串口通信协议:
 * 报文格式: # + 偏移厘米数(±xx.xx) + $ + \r\n
 * 示例: #2.35$\r\n  (偏左2.35cm)
 *       #-4.12$\r\n (偏右4.12cm)
 *       #0.00$\r\n  (正中)
 */

/* OpenMV 数据帧结构体 */
typedef struct
{
    float ball_x_cm;    /* 小球偏移量(cm)，正=偏左，负=偏右，0=正中 */
    bool is_valid;      /* 数据是否有效 */
} OpenMV_DataTypeDef;

/* 函数声明 */
void OpenMV_Init(void);
bool OpenMV_ParseFrame(uint8_t *data, uint8_t len, OpenMV_DataTypeDef *result);
void OpenMV_ProcessFrame(uint8_t *data, uint8_t len);

/* 获取最新解析的球位置 */
float OpenMV_GetBallPosition(void);
bool OpenMV_IsDataFresh(void);

#endif /* __OPENMV_H */
