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
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

static const double C = 299792458.0;       // Speed of light
static const double tsfreq = 499.2e6 * 128;  // Timestamp counter frequency
#define ANTENNA_OFFSET 154.6   // In meter
#define ANTENNA_DELAY  (ANTENNA_OFFSET*499.2e6*128)/299792458.0 // In radio tick
// gradient decent
static const double iter_step = 0.3;

/*
 * Timestamps to send
 * Suppose n is the ID to this anchor;
 *  index n: the time it broadcast the last message
 *  others : the time it receive message from other corresponding anchors
 **/
// Timestamps of all anchors. We need 2 timestamps for anchor mesurement
uint32_t last_timestamp[MAX_NR_ANCHORS][MAX_NR_ANCHORS];
uint32_t this_timestamp[MAX_NR_ANCHORS][MAX_NR_ANCHORS];

// Coordinate calculation
static double distances[MAX_NR_ANCHORS];
struct coordinate anchor_coo[MAX_NR_ANCHORS] = { 0 };

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
 * Message format: timestamps following by a struct coordinate
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
	// Timestamps: the low 32-bits
	memcpy(&txPacket.payload[2], this_timestamp[my_id], 4*anchors_in_used);
	memcpy(last_timestamp[my_id], this_timestamp[my_id], 4*anchors_in_used);
	// Coordinate: in mm
	struct coordinate_mm coomm;
	coomm.x = anchor_coo[my_id].x*1000;
	coomm.y = anchor_coo[my_id].y*1000;
	coomm.z = anchor_coo[my_id].z*1000;
	memcpy(&txPacket.payload[2+4*anchors_in_used], &coomm, sizeof(struct coordinate_mm));

	dwNewTransmit(dev);
	dwSetDefaults(dev);
	dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH + 2 +
			4*anchors_in_used + sizeof(struct coordinate_mm));

	dwWaitForResponse(dev, true);
	dwStartTransmit(dev);
	LED_TUGGLE(3);
}

static void tdoa_anchor_init(dwDevice_t *dev)
{
	(void)dev;

	my_id = config.address;
	MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);
	anchors_in_used = config.nr_anchor;
	txPacket.pan = 0xbccf;

	// Init anchor coordinate
	if (my_id > 2)
		anchor_coo[my_id].x = anchor_coo[my_id].y = anchor_coo[my_id].z = 0.1;
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
	this_timestamp[my_id][my_id] = departure.low32;
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
	// Its timestamps
	memcpy(&last_timestamp[its_id], &this_timestamp[its_id], 4*anchors_in_used);
	memcpy(&this_timestamp[its_id], &rxPacket.payload[2], 4*anchors_in_used);
	this_timestamp[my_id][its_id] = arival.low32;
	// Its coordinate
	struct coordinate_mm *coomm = (struct coordinate_mm*)&rxPacket.payload[2+4*anchors_in_used];
	if (coomm->x == 0) coomm->x = 1;
	if (coomm->y == 0) coomm->y = 1;
	if (coomm->z == 0) coomm->z = 1;
	double tmpx = 0.001*coomm->x, tmpy = 0.001*coomm->y, tmpz = 0.001*coomm->z;
	anchor_coo[its_id].x = LowPassFilter(tmpx, anchor_coo[its_id].x);
	anchor_coo[its_id].y = LowPassFilter(tmpy, anchor_coo[its_id].y);
	anchor_coo[its_id].z = LowPassFilter(tmpz, anchor_coo[its_id].z);

	// Calculate the distance between me and that anchor
	double tround1 = last_timestamp[my_id][its_id] - last_timestamp[my_id][my_id],
	       treply1 = this_timestamp[its_id][its_id]- last_timestamp[its_id][my_id],
	       treply2 = this_timestamp[my_id][my_id]  - last_timestamp[my_id][its_id],
	       tround2 = this_timestamp[its_id][my_id] - this_timestamp[its_id][its_id];
	double tprop_ctn = ((tround1*tround2) - (treply1*treply2)) /
	                   (tround1 + tround2 + treply1 + treply2);
	double tprop = tprop_ctn/tsfreq;
	double cur_distance = C * tprop;
	if (cur_distance > 0.0 && cur_distance < 20.0)
		distances[its_id] = LowPassFilter(cur_distance, distances[its_id]);
	// Calculate my coordinate
	switch (my_id) {
	case 0:
		anchor_coo[0].x = anchor_coo[0].y = anchor_coo[0].z = 0.0;
		break;
	case 1:
		anchor_coo[1].x = distances[0];
		anchor_coo[1].y = anchor_coo[1].z = 0.0;
		break;
	case 2: {
		double a,b,c,x, tmp;
		a = distances[1], b = distances[0], c = anchor_coo[1].x;
		x = (b*b + c*c - a*a) / 2 / c;
		tmp = b*b - x*x;
		if (tmp < 0)
			break;
		anchor_coo[2].x = x;
		anchor_coo[2].y = sqrt(tmp);
		anchor_coo[2].z = 0.0;
		break;
	}
	default: {
		// Other anchors iterates their coordinate
		struct coordinate adj = { 0 };
		// for each edge
		for (int i = 0; i < anchors_in_used; i++) {
			if(i != my_id) {
				double dx = anchor_coo[i].x - anchor_coo[my_id].x,
				       dy = anchor_coo[i].y - anchor_coo[my_id].y,
				       dz = anchor_coo[i].z - anchor_coo[my_id].z;
				double curr_dist = sqrt(dx*dx + dy*dy + dz*dz);
				if (curr_dist < 0.2) curr_dist = 0.2;
				double real_dist = distances[i];
				double difference = curr_dist - real_dist;
				double adj_factor = iter_step * difference / curr_dist;
				dx *= adj_factor, dy *= adj_factor, dz *= adj_factor;
				adj.x -= dx, adj.y -= dy, adj.z -= dz;
			}
		}
		anchor_coo[my_id].x -= adj.x;
		anchor_coo[my_id].y -= adj.y;
		anchor_coo[my_id].z -= adj.z;
		if (my_id == 3 && anchor_coo[3].z < -0.15)
			anchor_coo[3].z = -anchor_coo[3].z;
		break;
	}
	}
#  define TRIM_VAL(val) \
	if (isnan((val)) || (val) > 20.0 || (val) < -20.0) (val) = 0.1;
	TRIM_VAL(anchor_coo[my_id].x)
	TRIM_VAL(anchor_coo[my_id].y)
	TRIM_VAL(anchor_coo[my_id].z)
	// abs(Z) could not be too small
	if (anchor_coo[my_id].z < 0.0005 && anchor_coo[my_id].z > -0.0005)
		anchor_coo[my_id].z = 0.0005;

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
	static int cnt = 0;
	cnt++;
	if (cnt > 20) {
		cnt = 0;
		char buff[100];
		int x=anchor_coo[my_id].x*1000, y=anchor_coo[my_id].y*1000, z=anchor_coo[my_id].z*1000;
		int d0 = distances[0]*1000, d1 = distances[1]*1000, d2 = distances[2]*1000;
		sprintf(buff, "%d,%d,%d  %d,%d,%d\r\n", x, y, z, d0, d1, d2);
		SendBuffStartDMA(buff, strlen(buff));
	}

	timeout_cnt++;
	if (timeout_cnt > TDOA_ANCHOR_LOOP_TIMEOUT) {
		// A long time since last tx/rx, and a packet may be lost
		timeout_cnt = 0;
		error_cnt++;
		LED_TUGGLE(2);
		if (error_cnt > 100) {
			/*
			 * The DWM1000 cannot send anything after some recv failure.
			 * So if this happened, we should reset the system.
			 */
			NVIC_SystemReset();
		} else if (config.address == 0) {
			broadcast_my_timestamps(dev);
		} else {
			dwNewReceive(dev);
			dwSetDefaults(dev);
			dwStartReceive(dev);
		}
	}
}

struct uwb_operation uwb_tdoa_anchor_ops = {
	.init = tdoa_anchor_init,
	.on_tx = tdoa_anchor_on_tx,
	.on_rx = tdoa_anchor_on_rx,
	.on_period = tdoa_anchor_on_period
};
