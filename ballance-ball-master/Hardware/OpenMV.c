#include "OpenMV.h"
#include "usart.h"
#include <string.h>

/* 全局变量 - 存储最新解析的球位置 */
static float g_BallPosition = 0.0f;
static bool  g_DataFresh = false;

/* 手动字符串转浮点数（避免依赖 stdlib atof，嵌入式友好） */
static float OpenMV_StrToFloat(char *str, uint8_t len)
{
    float result = 0.0f;
    float decimal_part = 0.0f;
    float decimal_div = 1.0f;
    uint8_t is_negative = 0;
    uint8_t i = 0;
    uint8_t in_decimal = 0;

    if (len == 0) return 0.0f;

    /* 处理符号 */
    if (str[0] == '-')
    {
        is_negative = 1;
        i = 1;
    }
    else if (str[0] == '+')
    {
        i = 1;
    }

    /* 解析整数和小数部分 */
    for (; i < len; i++)
    {
        if (str[i] == '.')
        {
            in_decimal = 1;
            continue;
        }

        if (str[i] >= '0' && str[i] <= '9')
        {
            if (in_decimal)
            {
                decimal_div *= 10.0f;
                decimal_part = decimal_part * 10.0f + (float)(str[i] - '0');
            }
            else
            {
                result = result * 10.0f + (float)(str[i] - '0');
            }
        }
    }

    result += decimal_part / decimal_div;

    if (is_negative)
        result = -result;

    return result;
}

/**
  * @brief  初始化 OpenMV 通信
  *         使能 USART3 IDLE 中断并启动 DMA 接收
  * @retval None
  */
void OpenMV_Init(void)
{
    /* 清除 IDLE 标志 */
    __HAL_UART_CLEAR_IDLEFLAG(&huart3);

    /* 使能 UART3 IDLE 中断 */
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    /* 启动 DMA 循环接收，数据存入 rxCmd3 */
    HAL_UART_Receive_DMA(&huart3, (uint8_t *)rxCmd3, CMD_LEN);
}

/**
  * @brief  解析 OpenMV 发来的数据帧
  *         格式: #±xx.xx$\r\n
  * @param  data:   接收到的原始数据缓冲区
  * @param  len:    数据长度
  * @param  result: 解析结果输出
  * @retval true:  解析成功
  *         false: 解析失败（格式错误）
  */
bool OpenMV_ParseFrame(uint8_t *data, uint8_t len, OpenMV_DataTypeDef *result)
{
    if (data == NULL || result == NULL || len < 5)
        return false;

    /* 查找起始标志 '#' */
    uint8_t start_idx = 0;
    for (start_idx = 0; start_idx < len; start_idx++)
    {
        if (data[start_idx] == '#')
            break;
    }

    if (start_idx >= len)
        return false;

    /* 查找结束标志 '$' */
    uint8_t end_idx = 0;
    for (end_idx = start_idx + 1; end_idx < len; end_idx++)
    {
        if (data[end_idx] == '$')
            break;
    }

    if (end_idx >= len || end_idx <= start_idx + 1)
        return false;

    /* 提取 '#' 和 '$' 之间的数据 */
    uint8_t val_len = end_idx - start_idx - 1;
    if (val_len > 16)  /* 数值部分不应太长 */
        return false;

    char val_str[17] = {0};
    memcpy(val_str, &data[start_idx + 1], val_len);

    /* 验证字符串是否为合法的浮点数字格式 */
    bool has_dot = false;
    bool has_digit = false;
    uint8_t i = 0;

    if (val_str[0] == '-' || val_str[0] == '+')
        i = 1;

    for (; i < val_len; i++)
    {
        if (val_str[i] == '.')
        {
            if (has_dot) return false;  /* 多个小数点 */
            has_dot = true;
        }
        else if (val_str[i] >= '0' && val_str[i] <= '9')
        {
            has_digit = true;
        }
        else
        {
            return false;  /* 非法字符 */
        }
    }

    if (!has_digit)
        return false;

    /* 手动转换为浮点数（不依赖 atof） */
    result->ball_x_cm = OpenMV_StrToFloat(val_str, val_len);
    result->is_valid = true;

    return true;
}

/**
  * @brief  处理接收到的 OpenMV 数据帧
  *         调用 OpenMV_ParseFrame 解析并更新全局位置
  * @param  data: 接收到的原始数据缓冲区
  * @param  len:  数据长度
  * @retval None
  */
void OpenMV_ProcessFrame(uint8_t *data, uint8_t len)
{
    OpenMV_DataTypeDef result = {0.0f, false};

    if (OpenMV_ParseFrame(data, len, &result))
    {
        g_BallPosition = result.ball_x_cm;
        g_DataFresh = true;
    }
}

/**
  * @brief  获取最新解析的球偏移位置
  * @retval 球偏移量(cm)
  */
float OpenMV_GetBallPosition(void)
{
    g_DataFresh = false;
    return g_BallPosition;
}

/**
  * @brief  检查是否有新的数据到达
  * @retval true:  有新数据
  *         false: 无新数据
  */
bool OpenMV_IsDataFresh(void)
{
    return g_DataFresh;
}
