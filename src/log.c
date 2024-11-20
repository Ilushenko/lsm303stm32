/// \file log.c
/// \brief This file is part of LSM303DLHC Library for STM32 Nucleo L4
/// \copyright &copy; https://github.com/Ilushenko Oleksandr Ilushenko
///	\author Oleksandr Ilushenko
///	\date 2024
#include "log.h"

UART_HandleTypeDef* huart_ = 0;
char printbuf[128] = { 0 };

void setlog(UART_HandleTypeDef *uart)
{
    huart_ = uart;
}
