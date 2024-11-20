/// \file log.h
/// \brief This file is part of LSM303DLHC Library for STM32 Nucleo L4
/// \copyright &copy; https://github.com/Ilushenko Oleksandr Ilushenko
///	\author Oleksandr Ilushenko
///	\date 2024
#ifndef __LOG_H__
#define __LOG_H__

#include "stm32l4xx_hal.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef* huart_;
extern char printbuf[128];

void setlog(UART_HandleTypeDef* uart);

#define __FILENAME__ (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)

#define xTrace(fmt, ...) if (huart_ != 0) {                                                                 \
        const uint16_t sz = sprintf(&printbuf[0], fmt, ##__VA_ARGS__);                                      \
        HAL_UART_Transmit(huart_, (uint8_t*)&printbuf[0], sz, 1000);                                        \
    }

#define xDebug(fmt, ...) if (huart_ != 0) {                                                                 \
        uint16_t sz = sprintf(&printbuf[0], "[%s %s: %u] DEBUG ", __FUNCTION__, __FILENAME__, __LINE__);    \
        HAL_UART_Transmit(huart_, (uint8_t*)&printbuf[0], sz, 1000);                                        \
        sz = sprintf(&printbuf[0], fmt, ##__VA_ARGS__);                                                     \
        HAL_UART_Transmit(huart_, (uint8_t*)&printbuf[0], sz, 1000);                                        \
    }

#define xWarning(fmt, ...) if (huart_ != 0) {                                                               \
        uint16_t sz = sprintf(&printbuf[0], "[%s %s: %u] WARNING ", __FUNCTION__, __FILENAME__, __LINE__);  \
        HAL_UART_Transmit(huart_, (uint8_t*)&printbuf[0], sz, 1000);                                        \
        sz = sprintf(&printbuf[0], fmt, ##__VA_ARGS__);                                                     \
        HAL_UART_Transmit(huart_, (uint8_t*)&printbuf[0], sz, 1000);                                        \
    }

#define xError(fmt, ...) if (huart_ != 0) {                                                                 \
        uint16_t sz = sprintf(&printbuf[0], "[%s %s: %u] ERROR ", __FUNCTION__, __FILENAME__, __LINE__);    \
        HAL_UART_Transmit(huart_, (uint8_t*)&printbuf[0], sz, 1000);                                        \
        sz = sprintf(&printbuf[0], fmt, ##__VA_ARGS__);                                                     \
        HAL_UART_Transmit(huart_, (uint8_t*)&printbuf[0], sz, 1000);                                        \
    }

#endif // __LOG_H__