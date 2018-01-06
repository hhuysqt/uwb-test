/*
 * TDOA anchor algorithm
 *  This TDOA algorithm is a hyperbolic method, while there're other elliptic methods.
 *  Anchors broadcast their timestamps one by one.
 *  Based on anchors' timestamps and TOA of the msg, tags calculate the distances between
 *  anchors and anchors, the TDOA between tag and anchors, and finally the coordinate of
 *  the tag, whitch is the intersection of at least 3 hyperboles.
 *  Anchors don't care about their location, while tags do the calculation.
 */

#include <uwb.h>
#include <stdio.h>
#include <string.h>

#define ANTENNA_OFFSET 154.6   // In meter
#define ANTENNA_DELAY  (ANTENNA_OFFSET*499.2e6*128)/299792458.0 // In radio tick
/*
 * Timestamps to send
 * Suppose n is the ID to this anchor;
 *  index n: the time it broadcast the last message
 *  others : the time it receive message from other corresponding anchors
 **/
static uint32_t timestamps[MAX_NR_ANCHORS];
static int anchors_in_used = NR_ANCHORS;
static int my_id = 0;
/*
 *  Maintain a counter to dianose send/receive timeout
 *  The cnt is increased only in on_period(), and is resetted in on_tx()/on_rx()
 *  and some other places...
 */
#  define TDOA_ANCHOR_LOOP_TIMEOUT 10
static int timeout_cnt = 0;
static int error_cnt = 0;

/*
 * Global temporary packet
 */
static packet_t txPacket;
static packet_t rxPacket;

/*
 * Broadcast all the timestamps
 */
static void broadcast_my_timestamps(dwDevice_t *dev)
{
	mydelay(1);
	txPacket.sourceAddress[0] = my_id;
	txPacket.payload[PAYLOAD_TYPE] = MSG_TDOA;
	// only the low 32-bits in-used in dwTime_t
	memcpy(&txPacket.payload[2], timestamps, 4*anchors_in_used);

	dwNewTransmit(dev);
	dwSetDefaults(dev);
	dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH + 2 +
			4*anchors_in_used);

	dwWaitForResponse(dev, true);
	dwStartTransmit(dev);
	LED_TUGGLE(3);
}

static void tdoa_anchor_init(dwDevice_t *dev)
{
	my_id = config.address;
	MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);
	anchors_in_used = config.nr_anchor;
	txPacket.pan = 0xbccf;
}

/*
 * Callbacks
 */
static void tdoa_anchor_on_tx(dwDevice_t *dev)
{
	timeout_cnt = 0;
	error_cnt = 0;

	dwTime_t departure;
	dwGetTransmitTimestamp(dev, &departure);
	departure.full += (ANTENNA_DELAY / 2);
	timestamps[my_id] = departure.low32;
}

static void tdoa_anchor_on_rx(dwDevice_t *dev)
{
	timeout_cnt = 0;

	int dataLength = dwGetDataLength(dev);
	if (dataLength == 0)
		return;
	dwGetData(dev, (uint8_t*)&rxPacket, dataLength);

	dwTime_t arival = { .full=0 };
	dwGetReceiveTimestamp(dev, &arival);
	arival.full -= (ANTENNA_DELAY / 2);

	uint8_t its_id = rxPacket.sourceAddress[0];
	timestamps[its_id] = arival.low32;

	if (my_id == ((its_id+1) % anchors_in_used))
		broadcast_my_timestamps(dev);
	else {
		dwNewReceive(dev);
		dwSetDefaults(dev);
		dwStartReceive(dev);
	}
}

static void tdoa_anchor_on_period(dwDevice_t *dev)
{
	timeout_cnt++;

	if (timeout_cnt > TDOA_ANCHOR_LOOP_TIMEOUT) {
		// A long time since last tx/rx, and a packet may be lost
		timeout_cnt = 0;
		error_cnt++;
		LED_TUGGLE(2);
		if (config.address == 0)
			broadcast_my_timestamps(dev);
		else if (error_cnt < 100) {
			dwNewReceive(dev);
			dwSetDefaults(dev);
			dwStartReceive(dev);
		} else {
			/*
			 * The DWM1000 cannot send anything after some recv failure.
			 * So if this happened, we should reset the system.
			 */
			NVIC_SystemReset();
		}
	}
}

struct uwb_operation uwb_tdoa_anchor_ops = {
	.init = tdoa_anchor_init,
	.on_tx = tdoa_anchor_on_tx,
	.on_rx = tdoa_anchor_on_rx,
	.on_period = tdoa_anchor_on_period
};
