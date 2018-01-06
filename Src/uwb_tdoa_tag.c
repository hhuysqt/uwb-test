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
static const double iter_step = 0.1;

/*
 * Timestamps of the anchors
 * Mark down the TOA of the messages, and get the timestamps from anchors.
 * First we calculate the TOF between anchors, thus the distance between anchors.
 * Then we calculate TDOA one by one between tag and anchor, thus the difference of 
 * distance between tag and anchor, and finally produce the tag's location.
 */
// Timestamps from anchors. We need 2 timestamps for anchor mesurement
static uint32_t last_timestamp[MAX_NR_ANCHORS][MAX_NR_ANCHORS];
static uint32_t this_timestamp[MAX_NR_ANCHORS][MAX_NR_ANCHORS];
// the tag's timestamp
static uint32_t tag_timestamp[MAX_NR_ANCHORS];
// Used to calibrate anchors' and tag's clock
static double tag_round[MAX_NR_ANCHORS];
static double timestamp_factor[MAX_NR_ANCHORS];
// results
static double anchor_distances[MAX_NR_ANCHORS][MAX_NR_ANCHORS];
static struct coordinate anchor_coo[MAX_NR_ANCHORS], my_coo;
static double tdoa_diff_distance[MAX_NR_ANCHORS];

#  define TDOA_TAG_LOOP_TIMEOUT 5
static int timeout_cnt = 0;
static int recv_cnt = 0;
static int valid_anc_mes = 0;
static bool is_init_position = false;

/*
 * Global temporary packet
 */
static packet_t rxPacket;

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
	if (dataLength != MAC802154_HEADER_LENGTH + 2 + 4*anchors_in_used)
		return;
	dwGetData(dev, (uint8_t*)&rxPacket, dataLength);

	dwTime_t arival = { .full=0 };
	dwGetReceiveTimestamp(dev, &arival);

	uint8_t its_id = rxPacket.sourceAddress[0];

	memcpy(&last_timestamp[its_id], &this_timestamp[its_id], 4*anchors_in_used);
	memcpy(&this_timestamp[its_id], &rxPacket.payload[2], 4*anchors_in_used);
	// calibration
	double anchor_round = this_timestamp[its_id][its_id] - last_timestamp[its_id][its_id];
	timestamp_factor[its_id] = tag_round[its_id] / anchor_round;

	if (recv_cnt < 2*anchors_in_used){
		// Not enough valid data
		recv_cnt++;
	} else {
		/*
		 * Calculate the tof between anchors.
		 * this_timestamp[its_id] for T-reply's,
		 * last_timestamp[] for T-round's
		 */
		for (int i = 0; i < anchors_in_used; i++)
		if (i != its_id) {
			double tround1 = (last_timestamp[its_id][i] - last_timestamp[its_id][its_id])*timestamp_factor[its_id],
			       treply1 = (this_timestamp[i][i]      - last_timestamp[i][its_id])     *timestamp_factor[i],
			       treply2 = (this_timestamp[its_id][its_id] - last_timestamp[its_id][i])*timestamp_factor[its_id],
			       tround2 = (this_timestamp[i][its_id]      - this_timestamp[i][i])     *timestamp_factor[i];
			// Symmetric formula
			//double tprop_ctn = (tround1 + tround2 - treply1 - treply2) / 4;
			// Asymmetric formula
			double tprop_ctn = ((tround1*tround2) - (treply1*treply2)) /
			                   (tround1 + tround2 + treply1 + treply2);
			double tprop = tprop_ctn/tsfreq;
			double cur_distance = C * tprop;
			if (cur_distance > 20 || cur_distance < 0) {
				// Invalid distance
				//recv_cnt = 0;
				LED_TUGGLE(2);
			} else if (!isnan(cur_distance)) {
				if (its_id < i)
					anchor_distances[its_id][i] = LowPassFilter(cur_distance, anchor_distances[its_id][i]);
				else
					anchor_distances[i][its_id] = LowPassFilter(cur_distance, anchor_distances[i][its_id]);
				valid_anc_mes++;
			}
		}

		/*
		 * Calculate the TDOA of tag and two anchors: 0 and its_id
		 */
		if (its_id != 0) {
			double tround = this_timestamp[its_id][0] - this_timestamp[its_id][its_id];
			tround *= timestamp_factor[its_id];
			double trx = tag_timestamp[0] - tag_timestamp[its_id];
			double tdoa = tround - trx;
			double tprop = tdoa/tsfreq;
			double diff_dist = C * tprop - anchor_distances[0][its_id];
			diff_dist = -diff_dist;
			if (diff_dist > -22.0 && diff_dist < 22.0)
				tdoa_diff_distance[its_id] = LowPassFilter(diff_dist, tdoa_diff_distance[its_id]);
		} else {
			for (int i = 1; i < anchors_in_used; i++) {
				double tround = this_timestamp[0][i] - this_timestamp[0][0];
				tround *= timestamp_factor[0];
				double trx = tag_timestamp[i] - tag_timestamp[0];
				double tdoa = tround - trx;
				double tprop = tdoa/tsfreq;
				double diff_dist = C * tprop - anchor_distances[0][i];
				if (diff_dist > -22.0 && diff_dist < 22.0)
					tdoa_diff_distance[i] = LowPassFilter(diff_dist, tdoa_diff_distance[i]);
			}
		}
	}
	tag_round[its_id] = arival.low32 - tag_timestamp[its_id];
	tag_timestamp[its_id] = arival.low32;

	LED_TUGGLE(3);
	dwNewReceive(dev);
	dwSetDefaults(dev);
	dwStartReceive(dev);
}

static double coo_distance(struct coordinate *c1, struct coordinate *c2)
{
	double dx = c1->x-c2->x, dy = c1->y-c2->y, dz = c1->z-c2->z;
	return sqrt(dx*dx + dy*dy + dz*dz);
}

/*
 * The first 3 anchors are pre-defined
 */
static int initiate_first_3_anchors(void)
{
	double a,b,c,x, tmp;
	// 0
	anchor_coo[0].x = anchor_coo[0].y = anchor_coo[0].z = 0.0;
	// 1
	anchor_coo[1].x = anchor_distances[0][1], anchor_coo[1].y = anchor_coo[1].z = 0.0;
	// 2
	a = anchor_distances[1][2], b = anchor_distances[0][2], c = anchor_distances[0][1];
	anchor_coo[2].x = x = (b*b + c*c - a*a) / 2 / c;
	tmp = b*b - x*x;
	if (tmp < 0)
		return -1;
	anchor_coo[2].y = sqrt(tmp);
	anchor_coo[2].z = 0.0;
	return 0;
}
/*
 * Calculate the coordinate based on the first 3 anchors.
 * z is positive
 */
static int initiate_anchor_positive_coo(int n)
{
	double a,b,c,x,y,z, tmp;
	a = anchor_distances[0][n], b = anchor_distances[1][n], c = anchor_distances[2][n];
	double x1 = anchor_coo[1].x, x2 = anchor_coo[2].x, y2 = anchor_coo[2].y;
	x = (x1*x1 + a*a - b*b) / 2 / x1;
	y = (a*a + x2*x2 + y2*y2 - 2*x2*x - c*c) / 2 / y2;
	if (x*x > 600 || y*y > 600)
		return -1;
	anchor_coo[n].x = x, anchor_coo[n].y = y;
	tmp = a*a - x*x - y*y;
	if(tmp < 0) 
		return -1;
	z = sqrt(tmp);
	anchor_coo[n].z = z;
	return 0;	
}

/*
 * Calculate the first 4 anchors' position, and
 * the rest based on these 4 anchors.
 */
static void initiate_anchor_position(void)
{
	is_init_position = true;
	if (initiate_first_3_anchors() < 0)
		return;
	if (anchors_in_used < 4)
		return;
	// 3
	if(initiate_anchor_positive_coo(3) < 0) 
		return;
	// The rest anchors
	for (int i = 4; i < anchors_in_used; i++) {
		if (initiate_anchor_positive_coo(i) < 0) {
			is_init_position = false;
			continue;
		}
		double diff = coo_distance(&anchor_coo[3], &anchor_coo[i]) - anchor_distances[3][i];
		const double max_diff = 0.6;
		if (diff > max_diff || diff < -max_diff)
			anchor_coo[i].z = -anchor_coo[i].z;
	}
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
 * Gradient decent iterations
 */
static void calculate_anchor(void)
{
	initiate_first_3_anchors();
	if (anchors_in_used < 4)
		return;
	initiate_anchor_positive_coo(3);

#if 1
	struct coordinate adj[MAX_NR_ANCHORS] = { 0 };
	// for each edge
	for (int i = 0; i < anchors_in_used-1; i++) {
		for (int j = i+1; j < anchors_in_used; j++) {
			double dx = anchor_coo[i].x - anchor_coo[j].x,
			       dy = anchor_coo[i].y - anchor_coo[j].y,
			       dz = anchor_coo[i].z - anchor_coo[j].z;
			double curr_dist = sqrt(dx*dx + dy*dy + dz*dz);
			double real_dist = anchor_distances[i][j];
			double difference = curr_dist - real_dist;
			double adj_factor = iter_step * difference / curr_dist;
			dx *= adj_factor, dy *= adj_factor, dz *= adj_factor;
			if (i > 2)
				adj[i].x -= dx, adj[i].y -= dy, adj[i].z -= dz;
			if (j > 2)
				adj[j].x += dx, adj[j].y += dy, adj[j].z += dz;
		}
	}
	// for each node
	for (int i = 0; i < anchors_in_used; i++) {
		anchor_coo[i].x += adj[i].x;
		anchor_coo[i].y += adj[i].y;
		anchor_coo[i].z += adj[i].z;
	}
#endif
}
static void calculate_tag(void)
{
}

static void tdoa_tag_on_period(dwDevice_t *dev)
{
	static int period_cnt = 0;
	period_cnt++;
	if (period_cnt >= 10) {
		period_cnt = 0;
		if (is_init_position) {
			calculate_anchor();
//			initiate_anchor_position();
			calculate_tag();
		} else if (valid_anc_mes > 100) {
			initiate_anchor_position();
			initiate_tag_position();
		}

		LED_TUGGLE(1);

//#define PRINT_ANCHOR_MEASUREMENT
#define PRINT_ANCHOR_COORDINATE
		char buff[300];
		int i, cnt = 0;

#ifdef PRINT_DIFF_DISTANCE
		for (i = 0; i < anchors_in_used-1; i++) {
			int dist = tdoa_diff_distance[i+1]*1000;
			sprintf(&buff[i*6], "%5d,", dist);
		}
		buff[i*6] = '\r';
		buff[i*6+1] = '\n';
		while(SendBuffStartDMA(buff, i*6+2) == SEND_RETRY);
#endif
#ifdef PRINT_ANCHOR_COORDINATE
		buff[0] = 's';
		buff[1] = ':';
		cnt = 2;
		for (i = 0; i < anchors_in_used; i++) {
			int x = anchor_coo[i].x*1000, y = anchor_coo[i].y*1000, z = anchor_coo[i].z*1000;
			sprintf(&buff[cnt], "%5d,%5d,%5d|", x,y,z);
			cnt += 18;
		}
		buff[cnt++] = '\r';
		buff[cnt++] = '\n';
		while(SendBuffStartDMA(buff, cnt) == SEND_RETRY);
#endif
#ifdef PRINT_ANCHOR_MEASUREMENT
		for (i = 0; i < anchors_in_used-1; i++)
			for (int j = i+1; j < anchors_in_used; j++) {
				int dist = anchor_distances[i][j]*1000;
				sprintf(&buff[cnt], "%5d ", dist);
				cnt += 6;
			}
		buff[cnt++] = '\r';
		buff[cnt++] = '\n';
		while(SendBuffStartDMA(buff, cnt) == SEND_RETRY);
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
