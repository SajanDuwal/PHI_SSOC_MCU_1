/*
 * com_debug.h
 *
 *  Created on: Nov 24, 2024
 *      Author: sajanduwal
 */

#ifndef INC_COM_DEBUG_H_
#define INC_COM_DEBUG_H_

#include "main.h"
#include "stdarg.h"
#include "stdio.h"

extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim1;

void myDebug(const char *fmt, ...);

int bufferSize(char *buffer);

void delay_us(uint16_t ms);

#endif /* INC_COM_DEBUG_H_ */
