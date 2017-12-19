/*
 * A router is only used to receive data
 */
#include <uwb.h>
#include <string.h>
#include <stdio.h>
#include <usart.h>
#include <math.h>

/*
 * Private data
 */
static uint8_t router_address = DEFAULT_ROUTER_ADDR;
#define ANTENNA_OFFSET 154.6   // In meter
#define ANTENNA_DELAY  (ANTENNA_OFFSET*499.2e6*128)/299792458.0 // In radio tick
/*
 * timer
 *  Maintain a counter to dianose send/receive timeout
 *  The cnt is increased only in on_period(), and is resetted in on_tx()/on_rx()
 *  and some other places...
 */
#  define ROUTER_LOOP_TIMEOUT 100
static int router_timeout_cnt = 0;

/*
 * Global temporary packet
 */
static packet_t txPacket;
static packet_t rxPacket;

static void router_init(uint8_t addr)
{
	(void)addr;
	// Initialize the packet in the TX buffer
	MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);
	txPacket.pan = 0xbccf;
}

/*
 * Callbacks
 */
static void router_on_tx(dwDevice_t *dev)
{
	router_timeout_cnt = 0;

	dwTime_t departure;
	dwGetTransmitTimestamp(dev, &departure);
	departure.full += (ANTENNA_DELAY / 2);
}

static void router_on_rx(dwDevice_t *dev) {
	router_timeout_cnt = 0;

	dwTime_t arival = { .full=0 };
	int dataLength = dwGetDataLength(dev);

	if (dataLength == 0)
		return;

	memset(&rxPacket, 0, MAC802154_HEADER_LENGTH);
	dwGetData(dev, (uint8_t*)&rxPacket, dataLength);

	if (rxPacket.destAddress[0] != router_address) {
		dwNewReceive(dev);
		dwSetDefaults(dev);
		dwStartReceive(dev);
		return;
	}

	switch(rxPacket.payload[PAYLOAD_TYPE]) {
	case MSG_TO_ROUTER: {
		// received a coordinate and print it through USART
		char buff[100];
		struct coordinate_mm the_coo;
		memcpy(&the_coo, rxPacket.payload + 2, sizeof(the_coo));
		sprintf(buff, "s:%5d,%5d,%5d\r\n", the_coo.x,the_coo.y,the_coo.z);
		while (SendBuffStartDMA(buff, strlen(buff)) == SEND_RETRY);
		break;
	}
	default:
		break;
	}
}

static void router_on_failed(dwDevice_t *dev)
{
	LED_ON(1);
	dwIdle(dev);
	HAL_Delay(10);
	dwNewReceive(dev);
	dwSetDefaults(dev);
	dwStartReceive(dev);
}

static void router_on_timeout(dwDevice_t *dev)
{
	LED_ON(3);
	dwIdle(dev);
	HAL_Delay(10);
	dwNewReceive(dev);
	dwSetDefaults(dev);
	dwStartReceive(dev);
}

/*
 * The outer loop calls this periodically
 * A state-machine
 */
static void router_on_period(dwDevice_t *dev)
{
	router_timeout_cnt++;

	if (router_timeout_cnt > ROUTER_LOOP_TIMEOUT) {
		router_timeout_cnt = 0;
		LED_TUGGLE(2);
		dwNewReceive(dev);
		dwSetDefaults(dev);
		dwStartReceive(dev);
	}
}

struct uwb_operation uwb_router_ops = {
	.init = router_init,
	.on_tx = router_on_tx,
	.on_rx = router_on_rx,
	.on_timeout = router_on_timeout,
	.on_failed = router_on_failed,
	.on_period = router_on_period
};
