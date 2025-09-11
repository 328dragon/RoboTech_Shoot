/*
 * @Author: Elaina
 * @Date: 2024-07-11 16:06:19
 * @LastEditors: chaffer-cold 1463967532@qq.com
 * @LastEditTime: 2024-10-13 21:43:48
 * @FilePath: \MDK-ARM\Hardware\connect.cpp
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#include "connect.h"
using namespace Connect;
void Connect::Serial_Printf(UART_HandleTypeDef *huart, char *fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    HAL_UART_Transmit(huart, (uint8_t *)buf, strlen(buf), 0xffff);
    va_end(args);
}