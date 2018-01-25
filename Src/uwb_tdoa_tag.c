/*
 * TDOA tag algorithm
 *  Anchors broadcast their timestamps one by one.
 *  Based on anchors' timestamps and TOA of the msg, the tag calculate the distances among
 *  anchors and anchors, the TDOA between tag and anchors, and finally the coordinate of
 *  the tag, whitch is the intersection of at least 3 hyperboles.
 *  Tags don't send messages, so that it occupies no bandwidth, and the number of tags is
 *  not limitted.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "uwb.h"
#include "usart.h"

static int my_id = 0;
static int anchors_in_used = NR_ANCHORS;

/*
 * constance
 */
static const double C = 299792458.0;       // Speed of light
static const double tsfreq = 499.2e6 * 128;  // Timestamp counter frequency
#define ANTENNA_OFFSET 154.6   // In meter
#define ANTENNA_DELAY  (ANTENNA_OFFSET*499.2e6*128)/299792458.0 // In radio tick
// gradient decent
static const double iter_step = 0.45;

/*
 * Timestamps of the anchors
 * Mark down the TOA of the messages, and get the timestamps from anchors.
 * First we calculate the TOF between anchors, thus the distance between anchors.
 * Then we calculate TDOA one by one between tag and anchor, thus the difference of 
 * distance between tag and anchor, and finally produce the tag's location.
 */

// the tag's timestamp
static uint32_t tag_timestamp[MAX_NR_ANCHORS];
// Used to calibrate anchors' and tag's clock
static double tag_round[MAX_NR_ANCHORS];
static double timestamp_factor[MAX_NR_ANCHORS];
// results
static struct coordinate my_coo;
static double tdoa_diff_distance[MAX_NR_ANCHORS][MAX_NR_ANCHORS];

#  define TDOA_TAG_LOOP_TIMEOUT 5
static int timeout_cnt = 0;
static int recv_cnt = 0;
static int valid_anc_mes = 0;
static bool is_init_position = false;

/*
 * Global temporary packet
 */
static packet_t rxPacket;

/*
 * distance of two coordinates
 */
static double coo_distance(struct coordinate *c1, struct coordinate *c2)
{
	double dx = c1->x-c2->x, dy = c1->y-c2->y, dz = c1->z-c2->z;
	return sqrt(dx*dx + dy*dy + dz*dz);
}

static void tdoa_tag_init(dwDevice_t *dev)
{
	my_id = config.address;
	anchors_in_used = config.nr_anchor;
	//dwReceivePermanently(dev, true);
}

/*
 * Callbacks
 */
static void tdoa_tag_on_tx(dwDevice_t *dev)
{
	(void)dev;
}

static void tdoa_tag_on_rx(dwDevice_t *dev)
{
	timeout_cnt = 0;

	int dataLength = dwGetDataLength(dev);
	if (dataLength != MAC802154_HEADER_LENGTH + 2 + 4*anchors_in_used + sizeof(struct coordinate_mm))
		return;
	dwGetData(dev, (uint8_t*)&rxPacket, dataLength);

	dwTime_t arival = { .full=0 };
	dwGetReceiveTimestamp(dev, &arival);

	uint8_t its_id = rxPacket.sourceAddress[0];
	// Its timestamps
	memcpy(&last_timestamp[its_id], &this_timestamp[its_id], 4*anchors_in_used);
	memcpy(&this_timestamp[its_id], &rxPacket.payload[2], 4*anchors_in_used);
	// Its coordinate
	struct coordinate_mm *coomm = (struct coordinate_mm*)&rxPacket.payload[2+4*anchors_in_used];
	double tmpx = 0.001*coomm->x, tmpy = 0.001*coomm->y, tmpz = 0.001*coomm->z;
	anchor_coo[its_id].x = LowPassFilter(tmpx, anchor_coo[its_id].x);
	anchor_coo[its_id].y = LowPassFilter(tmpy, anchor_coo[its_id].y);
	anchor_coo[its_id].z = LowPassFilter(tmpz, anchor_coo[its_id].z);

	// calibration
	double anchor_round = this_timestamp[its_id][its_id] - last_timestamp[its_id][its_id];
	double tmpfac = tag_round[its_id] / anchor_round;
	timestamp_factor[its_id] = tmpfac;

	if (recv_cnt < 2*anchors_in_used){
		// Not enough valid data
		recv_cnt++;
	} else {
		/*
		 * Calculate the TDOA on this anchor
		 */
		for (int i = 0; i < anchors_in_used; i++) {
			if (i == its_id)
				continue;
			double tround = this_timestamp[its_id][i] - this_timestamp[its_id][its_id];
			tround *= timestamp_factor[its_id];
			double trx = tag_timestamp[i] - tag_timestamp[its_id];
			double tdoa = tround - trx;
			double tprop = tdoa/tsfreq;
			double diff_dist = C * tprop - coo_distance(&anchor_coo[its_id], &anchor_coo[i]);
			if (diff_dist > -22.0 && diff_dist < 22.0)
				tdoa_diff_distance[its_id][i] = LowPassFilter(diff_dist, tdoa_diff_distance[its_id][i]);
		}
	}
	tag_round[its_id] = arival.low32 - tag_timestamp[its_id];
	tag_timestamp[its_id] = arival.low32;
	valid_anc_mes++;

	LED_TUGGLE(3);
	dwNewReceive(dev);
	dwSetDefaults(dev);
	dwStartReceive(dev);
}

/*
 * Initiate the tag's position.
 * TODO: Calculate it as simple and precise as posible
 */
static void initiate_tag_position(void)
{
	double x = 0.0, y = 0.0, z = 0.0;
	for (int i = 0; i < anchors_in_used; i++) {
		x += anchor_coo[i].x;
		y += anchor_coo[i].y;
		z += anchor_coo[i].z;
	}
	my_coo.x = x/anchors_in_used;
	my_coo.y = y/anchors_in_used;
	my_coo.z = z/anchors_in_used;
}

/*
 * Main calculation
 */
static void calculate_tag(void)
{
	// Pre-calculate dx, dy, dz, and dist
	double dx[MAX_NR_ANCHORS], dy[MAX_NR_ANCHORS], dz[MAX_NR_ANCHORS],
	       dist[MAX_NR_ANCHORS];
	for (int i = 0; i < anchors_in_used; i++) {
		dx[i] = my_coo.x - anchor_coo[i].x;
		dy[i] = my_coo.y - anchor_coo[i].y;
		dz[i] = my_coo.z - anchor_coo[i].z;
		double tmpd = sqrt(dx[i]*dx[i] + dy[i]*dy[i] + dz[i]*dz[i]);
		if (tmpd < 0.001) dist[i] = 0.001;
		else dist[i] = tmpd;
	}
	// for each TDOA
	struct coordinate tmp = {0};
	for (int i = 0; i < anchors_in_used; i += 2) {
		for (int j = 0; j < anchors_in_used; j++) {
			if (i == j)
				continue;
			// gradient
			double gx = dx[i]/dist[i] - dx[j]/dist[j],
			       gy = dy[i]/dist[i] - dy[j]/dist[j],
			       gz = dz[i]/dist[i] - dz[j]/dist[j];
			double abs_g = sqrt(gx*gx + gy*gy + gz*gz);
			if (abs_g < 0.001) abs_g = 0.001;
			// difference of TDOA
			double curr_tdoa = dist[i] - dist[j],
			       real_tdoa = tdoa_diff_distance[i][j];
			double adj_factor = iter_step / anchors_in_used * (real_tdoa - curr_tdoa) / abs_g;

			gx *= adj_factor, gy *= adj_factor, gz *= adj_factor;
			tmp.x += gx, tmp.y += gy, tmp.z += gz;
		}
	}
	my_coo.x += tmp.x;
	my_coo.y += tmp.y;
	my_coo.z += tmp.z;
	#define MAX_TAG_COO 7
	if (my_coo.x > MAX_TAG_COO || my_coo.x < -MAX_TAG_COO ||
		my_coo.y > MAX_TAG_COO || my_coo.y < -MAX_TAG_COO ||
		my_coo.z > MAX_TAG_COO || my_coo.z < -MAX_TAG_COO)
		initiate_tag_position();
}

static void tdoa_tag_on_period(dwDevice_t *dev)
{
	static int period_cnt = 0;
	
	if (is_init_position) {
		calculate_tag();
	} else if (valid_anc_mes > 1000) {
		initiate_tag_position();
		is_init_position = true;
	}

	period_cnt++;
	if (period_cnt >= 2) {
		period_cnt = 0;

		LED_TUGGLE(1);

//#define PRINT_DIFF_DISTANCE
#define PRINT_ANCHOR_TAG
//#define PRINT_TAG
		char buff[300];
		int i, cnt = 0;

#ifdef PRINT_DIFF_DISTANCE
		for (int j = 0; j < 4; j++) {
			for (i = 0; i < anchors_in_used; i++) {
				int dist = tdoa_diff_distance[j][i]*1000;
				sprintf(&buff[cnt], "%5d,", dist);
				cnt += 6;
			}
			buff[cnt++] = '|';
		}
		int x,y,z;
		x = my_coo.x*1000, y = my_coo.y*1000, z = my_coo.z*1000;
		sprintf(&buff[cnt], "%5d,%5d,%5d", x,y,z);
		cnt += 17;
		buff[cnt++] = '\r';
		buff[cnt++] = '\n';
		while(SendBuffStartDMA(buff, cnt) == SEND_RETRY);
#endif
#ifdef PRINT_ANCHOR_TAG
		buff[0] = 's';
		buff[1] = ':';
		cnt = 2;
		int x,y,z;
		for (i = 0; i < anchors_in_used; i++) {
			x = anchor_coo[i].x*1000, y = anchor_coo[i].y*1000, z = anchor_coo[i].z*1000;
			sprintf(&buff[cnt], "%5d,%5d,%5d|", x,y,z);
			cnt += 18;
		}
		// And finally my coordinate
		x = my_coo.x*1000, y = my_coo.y*1000, z = my_coo.z*1000;
		sprintf(&buff[cnt], "%5d,%5d,%5d|", x,y,z);
		cnt += 18;
		buff[cnt++] = '\r';
		buff[cnt++] = '\n';
		SendBuffStartDMA(buff, cnt);
#endif
#ifdef PRINT_TAG
		if (is_init_position) {
			int x = my_coo.x*1000, y = my_coo.y*1000, z = my_coo.z*1000;
			sprintf(buff, "s:%5d,%5d,%5d\r\n", x,y,z);
			while(SendBuffStartDMA(buff, 21) == SEND_RETRY);
		}
#endif
	}

	timeout_cnt++;
	if (timeout_cnt > TDOA_TAG_LOOP_TIMEOUT) {
		// A long time since last tx/rx, and a packet may be lost
		timeout_cnt = 0;
		recv_cnt = 0;
		dwNewReceive(dev);
		dwSetDefaults(dev);
		dwStartReceive(dev);
	}
}

struct uwb_operation uwb_tdoa_tag_ops = {
	.init = tdoa_tag_init,
	.on_tx = tdoa_tag_on_tx,
	.on_rx = tdoa_tag_on_rx,
	.on_period = tdoa_tag_on_period
};
