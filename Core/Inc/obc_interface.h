/*
 * obc_interface.h
 *
 *  Created on: Nov 24, 2024
 *      Author: sajanduwal
 */

#ifndef INC_OBC_INTERFACE_H_
#define INC_OBC_INTERFACE_H_

#include "main.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef hlpuart1;

extern uint8_t MAIN_CMD[15];
extern uint8_t MCU_ID;

extern uint32_t MSN_CMD;
extern uint32_t MAIN_ADDR;


void WAIT_FOR_HANDSHAKE();
void Receive_MAIN_CMD();
void Execute_MAIN_CMD();

#endif /* INC_OBC_INTERFACE_H_ */
