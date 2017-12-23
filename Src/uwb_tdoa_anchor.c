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

/*
 * Timestamps to send
 * Suppose n is the ID to this anchor;
 *  index n: the time it broadcast the last message
 *  others : the time it receive message from other corresponding anchors
 **/
static dwTime_t timestamps[MAX_NR_ANCHORS];
static int anchors_in_used = NR_ANCHORS;
static int my_id = 0;
/*
 * anchor timer
 *  Maintain a counter to dianose send/receive timeout
 *  The cnt is increased only in on_period(), and is resetted in on_tx()/on_rx()
 *  and some other places...
 */
#  define TDOA_ANCHOR_LOOP_TIMEOUT 2
static int timeout_cnt = 0;

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
	txPacket.sourceAddress[0] = my_id;
	txPacket.payload[PAYLOAD_TYPE] = MSG_TDOA;
	// only 40-bits in-used in dwTime_t
	for (int i = 0; i < anchors_in_used; i++)
		memcpy(&txPacket.payload[2 + i*5], &timestamps[i], 5);

	dwNewTransmit(dev);
	dwSetDefaults(dev);
	dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH + 2 +
			5*anchors_in_used);

	dwWaitForResponse(dev, true);
	dwStartTransmit(dev);
}

static void tdoa_anchor_init(uint8_t addr)
{
	my_id = addr;
	MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);
	txPacket.pan = 0xbccf;
}

/*
 * Callbacks
 */
static void tdoa_anchor_on_tx(dwDevice_t *dev)
{
	timeout_cnt = 0;

	dwTime_t departure;
	dwGetTransmitTimestamp(dev, &departure);
	departure.full += (ANTENNA_DELAY / 2);
	timestamps[my_id] = departure;
}

static void tdoa_anchor_on_rx(dwDevice_t *dev)
{
	timeout_cnt = 0;

	dwTime_t arival = { .full=0 };
	int dataLength = dwGetDataLength(dev);
	if (dataLength == 0)
		return;

	dwGetData(dev, (uint8_t*)&rxPacket, dataLength);
	dwGetReceiveTimestamp(dev, &arival);
	arival.full -= (ANTENNA_DELAY / 2);

	uint8_t its_id = rxPacket.sourceAddress[0];
	timestamps[its_id] = arival;

	if (my_id == ((its_id+1) % anchors_in_used))
		broadcast_my_timestamps(dev);
}

static void tdoa_anchor_on_period(dwDevice_t *dev)
{
	timeout_cnt++;

	if (timeout_cnt > TDOA_ANCHOR_LOOP_TIMEOUT) {
		// A long time since last tx/rx, and a packet may be lost
		timeout_cnt = 0;
		LED_TUGGLE(2);
		broadcast_my_timestamps(dev);
	}
}

struct uwb_operation uwb_tdoa_anchor_ops = {
	.init = tdoa_anchor_init,
	.on_tx = tdoa_anchor_on_tx,
	.on_rx = tdoa_anchor_on_rx,
	.on_period = tdoa_anchor_on_period
};
