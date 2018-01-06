/*
 * Wrapper of usart transmiting and receiving, using DMA.
 * First, sprintf() to a buffer, and then send them via DMA.
 */
#include <usart.h>
#include <uwb.h>

/*
 * DMA buffer
 *  Every DMA transfer is default to start at the top of dma_buffer,
 *  except in one case that SendBuffDMA() is called while DMA is still sending.
 */
# define DMA_BUFFER_SIZE 4096
static unsigned char dma_buffer[DMA_BUFFER_SIZE];
static int nr_get = 0, nr_sent = 0;

/*
 * receive buffer
 */
static unsigned char recv_buff[10];

static UART_HandleTypeDef *huart;
static DMA_HandleTypeDef *hdma;

/*
 * DMA transmit-complete callback:
 *  Reset dma_buffer[]
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *the_uart)
{
	(void)the_uart;
	if (nr_sent < nr_get){
		// There's pending data to send.
		HAL_UART_Transmit_DMA(huart, &dma_buffer[nr_sent], nr_get-nr_sent);
		nr_sent = nr_get;
	} else {
		// Transmit done. Reset buffer.
		nr_sent = nr_get = 0;
	}
}

/*
 * Init the send-buffer engine
 */
void SendBuffInit (UART_HandleTypeDef *the_uart, DMA_HandleTypeDef *the_dma)
{
	huart = the_uart;
	hdma = the_dma;
}

/*
 * register a send buffer, whitch will be sent later via DMA
 */
int SendBuffStartDMA (void *buf, int size)
{
	if (size <= DMA_BUFFER_SIZE - nr_get) {
		if (nr_get == 0) {
			// A new transmittion
			memcpy(&dma_buffer[0], buf, size);
			HAL_UART_Transmit_DMA(huart, &dma_buffer[0], size);
			nr_sent = size;
		} else {
			memcpy(&dma_buffer[nr_get], buf, size);
			// the rest will be sent in DMA_TC_Callback()
		}
		nr_get += size;
		return SEND_OKAY;
	} else {
		return SEND_RETRY;
	}
}

/*
 * receive via IT
 */
void DoReceive(void)
{
	HAL_UART_Receive_IT(huart, recv_buff, 1);
}

/*
 * Handle receive during interrupt
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	(void)huart;
	char sendbuff[100];
	switch (recv_buff[0]) {
	/*
	 * Anchors and tags share the 36 addresses from
	 * 0 to 9, a to z (lowercase)
	 */
	case '0' ... '9':
	case 'a' ... 'z': {
		uint8_t address;
		if (recv_buff[0] < '9')
			address = recv_buff[0] - '0';
		else
			address = recv_buff[0] - 'a' + 10;
		sprintf(sendbuff, "Set address %d\r\n", address);
		SendBuffStartDMA(sendbuff, strlen(sendbuff));
		SetAddress(address);
		break;
	}

	/*
	 * Mode setting
	 */
	// Anchor mode
	case 'A':
		sprintf(sendbuff, "Set anchor mode\r\n");
		SendBuffStartDMA(sendbuff, strlen(sendbuff));
		SetMode(UWB_ANCHOR);
		break;
	// Tag mode
	case 'T':
		sprintf(sendbuff, "Set tag mode\r\n");
		SendBuffStartDMA(sendbuff, strlen(sendbuff));
		SetMode(UWB_TAG);
		break;
	// TDOA anchor
	case 'B':
		sprintf(sendbuff, "Set TDOA anchor mode\r\n");
		SendBuffStartDMA(sendbuff, strlen(sendbuff));
		SetMode(UWB_TDOA_ANCHOR);
		break;
	// TDOA tag
	case 'C':
		sprintf(sendbuff, "Set TDOA tag mode\r\n");
		SendBuffStartDMA(sendbuff, strlen(sendbuff));
		SetMode(UWB_TDOA_TAG);
		break;
	// Router mode
	case 'R':
		sprintf(sendbuff, "Set router mode\r\n");
		SendBuffStartDMA(sendbuff, strlen(sendbuff));
		SetMode(UWB_ROUTER);
		break;

	// reset system
	case 'S':
		sprintf(sendbuff, "Reset...\r\n\r\n");
		SendBuffStartDMA(sendbuff, strlen(sendbuff));
		mydelay(100);
		NVIC_SystemReset();
		break;

	/*
	 * Number of anchors.
	 * Press Ctrl+A ... Ctrl+J
	 */
	case 1 ... 10: {
		uint8_t nr = recv_buff[0];
		sprintf(sendbuff, "Setting %d anchors\r\n", nr);
		SendBuffStartDMA(sendbuff, strlen(sendbuff));
		SetNrAnchors(nr);
		break;
	}

	/*
	 * Help
	 */
	default: {
		const char *helpmsg = "\r\nHelp\r\n"
			"Set address   -- 0 ... 9, a ... z\r\n"
			"TWR-TOA Anchor-- A\r\n"
			"TWR-TOA Tag   -- T\r\n"
			"TDOA Anchor   -- B\r\n"
			"TDOA Tag      -- C\r\n"
			"Router mode   -- R\r\n"
			"Nr of anchors -- Ctrl+A ... Ctrl+J\r\n"
			"---\r\n"
			"Reset         -- S\r\n";
		SendBuffStartDMA((void*)helpmsg, strlen(helpmsg));
		break;
	}
	}
}
