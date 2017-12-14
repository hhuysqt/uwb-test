/*
 * Wrapper of usart transmiting and receiving, using DMA.
 * First, sprintf() to a buffer, and then send them via DMA.
 */

#ifndef MY_USART_WRAPPER_H
#define MY_USART_WRAPPER_H

#include <stm32f0xx_hal.h>
#include <string.h>
#include <stdio.h>

/*
 * Init the send-buffer engine
 */
void SendBuffInit (UART_HandleTypeDef *, DMA_HandleTypeDef *);

/*
 * register a send buffer, whitch will be sent later via DMA
 */
enum {
	SEND_OKAY,	// data will be sent later
	SEND_RETRY	// buffer overrun, and retry later
} send_status;
int SendBuffStartDMA (void *buf, int size);

/*
 * Triggers a receive
 */
void DoReceive(void);

#endif

