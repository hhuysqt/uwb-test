/*
 * tag algorithm
 */
#include <uwb.h>
#include <string.h>
#include <stdio.h>
#include <usart.h>
#include <math.h>

/*
 * Private data
 */
static enum tag_state {
	TAG_INIT,
	TAG_READY
} tag_current_state;
static uint8_t tag_address = 0;
static const double C = 299792458.0;       // Speed of light
static const double tsfreq = 499.2e6 * 128;  // Timestamp counter frequency
#define ANTENNA_OFFSET 154.6   // In meter
#define ANTENNA_DELAY  (ANTENNA_OFFSET*499.2e6*128)/299792458.0 // In radio tick
/*
 * tag timer
 *  Maintain a counter to dianose send/receive timeout
 *  The cnt is increased only in on_period(), and is resetted in on_tx()/on_rx()
 *  and some other places...
 */
#  define TAG_LOOP_TIMEOUT 2
static int tag_timeout_cnt = 0;

/*
 * Anchors automatically measure their distance on power-up.
 * Tags uses them to do the calculations
 */
static struct coordinate anchor_coordinate[MAX_NR_ANCHORS] = { 0 };
static bool is_get_reply = true;
static int anchor_focus = -1;
static int nr_anchors = 3;
static double distance_measured[MAX_NR_ANCHORS] = { 0 };
static struct coordinate my_coordinate = { 0 };
static struct twr_full_data tag_ranging;

/*
 * Global temporary packet
 */
static packet_t txPacket;
static packet_t rxPacket;

/*
 * First order RC low pass filter
 * Cutoff frequency is very low...
 */
static double LowPassFilter(double input, double last_output)
{
#  define RCFILTER_PARAM 0.1
	return RCFILTER_PARAM*input + (1 - RCFILTER_PARAM)*last_output;
}

/*
 * Tag send an initiate packet, to trigger a twr measurement, or 
 * ask for information, depending on @type.
 */
static void TagSendInitiatePacket(dwDevice_t *dev, uint8_t my_addr, uint8_t type)
{
	uint8_t address_tmp[8] = {0,0,0,0,0,0,0xcf,0xbc};

	dwIdle(dev);
	txPacket.payload[PAYLOAD_TYPE] = type;
	address_tmp[0] = my_addr;
	memcpy(txPacket.sourceAddress, address_tmp, 8);
	address_tmp[0] = anchor_focus;
	memcpy(txPacket.destAddress, address_tmp, 8);

	dwNewTransmit(dev);
	dwSetDefaults(dev);
	dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH + 2);

	dwWaitForResponse(dev, true);
	dwStartTransmit(dev);

	tag_timeout_cnt = 0;
}

static void TagSendToRouter(dwDevice_t *dev, uint8_t my_addr, uint8_t router_addr,
	void *data, int size)
{
	uint8_t address_tmp[8] = {0,0,0,0,0,0,0xcf,0xbc};

	dwIdle(dev);
	txPacket.payload[PAYLOAD_TYPE] = MSG_TO_ROUTER;
	address_tmp[0] = my_addr;
	memcpy(txPacket.sourceAddress, address_tmp, 8);
	address_tmp[0] = router_addr;
	memcpy(txPacket.destAddress, address_tmp, 8);
	if (size > 64)
		size = 64;
	memcpy(txPacket.payload + 2, data, size);

	dwNewTransmit(dev);
	dwSetDefaults(dev);
	dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH + 2 + size);

	dwWaitForResponse(dev, false);
	dwStartTransmit(dev);

	tag_timeout_cnt = 0;	
}

static void tag_init(uint8_t addr)
{
	tag_address = addr;
	tag_current_state = TAG_INIT;
	anchor_focus = -1;
	// Initialize the packet in the TX buffer
	MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);
	txPacket.pan = 0xbccf;
}

/*
 * Callbacks
 */
static void tag_on_tx(dwDevice_t *dev)
{
	tag_timeout_cnt = 0;

	dwTime_t departure;
	dwGetTransmitTimestamp(dev, &departure);
	departure.full += (ANTENNA_DELAY / 2);

	switch (txPacket.payload[PAYLOAD_TYPE]) {
	case MSG_TWR_POLL:
		tag_ranging.poll_tx = departure;
		break;
	case MSG_TWR_FINAL:
		tag_ranging.final_tx = departure;
		break;
	}
}

static void tag_on_rx(dwDevice_t *dev) {
	tag_timeout_cnt = 0;

	dwTime_t arival = { .full=0 };
	int dataLength = dwGetDataLength(dev);

	if (dataLength == 0)
		return;

	memset(&rxPacket, 0, MAC802154_HEADER_LENGTH);
	dwGetData(dev, (uint8_t*)&rxPacket, dataLength);
	dwGetReceiveTimestamp(dev, &arival);
	arival.full -= (ANTENNA_DELAY / 2);

	if (rxPacket.destAddress[0] != tag_address) {
		dwNewReceive(dev);
		dwSetDefaults(dev);
		dwStartReceive(dev);
		return;
	}

	memcpy(txPacket.destAddress, rxPacket.sourceAddress, 8);
	memcpy(txPacket.sourceAddress, rxPacket.destAddress, 8);
	switch(rxPacket.payload[PAYLOAD_TYPE]) {
	case MSG_TWR_ANSWER:
		txPacket.payload[0] = MSG_TWR_FINAL;

		dwNewTransmit(dev);
		dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);

		dwWaitForResponse(dev, true);
		dwStartTransmit(dev);

		tag_ranging.answer_rx = arival;
		break;
	case MSG_TWR_REPORT: {
		/*
		 * Report comes, and it's time to calculate the distance
		 */
		struct twr_report_turncated *report =
			(struct twr_report_turncated*)(rxPacket.payload+2);
		double tround1, treply1, treply2, tround2, tprop_ctn, tprop,
		       cur_distance;

		memcpy(&tag_ranging.poll_rx,   &report->poll_rx,   5);
		memcpy(&tag_ranging.answer_tx, &report->answer_tx, 5);
		memcpy(&tag_ranging.final_rx,  &report->final_rx,  5);
		tround1 = tag_ranging.answer_rx.low32 - tag_ranging.poll_tx.low32;
		treply1 = tag_ranging.answer_tx.low32 - tag_ranging.poll_rx.low32;
		tround2 = tag_ranging.final_rx.low32 - tag_ranging.answer_tx.low32;
		treply2 = tag_ranging.final_tx.low32 - tag_ranging.answer_rx.low32;

		tprop_ctn = ((tround1*tround2) - (treply1*treply2)) /
			(tround1 + tround2 + treply1 + treply2);

		tprop = tprop_ctn/tsfreq;
		cur_distance = C * tprop;
		distance_measured[anchor_focus] = LowPassFilter(cur_distance, distance_measured[anchor_focus]);
		is_get_reply = true;
		break;
	}
	case MSG_INIT_ANCHOR_POS: {
		/*
		 * Anchor replys its measurement.
		 * Time to calculate anchors' coordinates.
		 */
		double anchor_dist[2];
		memcpy(&anchor_dist, &rxPacket.payload[2], sizeof(anchor_dist));
		switch (anchor_focus) {
		case 1:
			anchor_coordinate[1].x = anchor_dist[0];
			break;
		case 2: {
			double c = anchor_coordinate[1].x, b = anchor_dist[0], a = anchor_dist[1], x;
			anchor_coordinate[2].x = x = (b*b + c*c - a*a) / 2 / c;
			double tmp = b*b - x*x;
			if (tmp > 0)
				anchor_coordinate[2].y = sqrt(tmp);
			break;
		}
		default:
			break;
		}
		is_get_reply = true;
		break;
	}
	default:
		break;
	}
}

static void tag_on_failed(dwDevice_t *dev)
{
	LED_ON(1);
	dwIdle(dev);
	HAL_Delay(10);
	dwNewReceive(dev);
	dwSetDefaults(dev);
	dwStartReceive(dev);
}

static void tag_on_timeout(dwDevice_t *dev)
{
	LED_ON(3);
	dwIdle(dev);
	HAL_Delay(10);
	dwNewReceive(dev);
	dwSetDefaults(dev);
	dwStartReceive(dev);
}

static void PrintResult(void)
{
	/*
	 * Calculate the coordinate in a simple way
	 */
	static double x, y, z;
	double a = distance_measured[0], b = distance_measured[1], c = distance_measured[2],
		x1 = anchor_coordinate[1].x, 
		x2 = anchor_coordinate[2].x, y2 = anchor_coordinate[2].y;
	x = (x1*x1 + a*a - b*b) / 2 / x1;
	y = (a*a + x2*x2 + y2*y2 - 2*x2*x - c*c) / 2 / y2;
	double tmp = a*a - x*x - y*y;
	if(tmp > 0)
		z = sqrt(tmp);
	my_coordinate.x = x;
	my_coordinate.y = y;
	my_coordinate.z = z;

	char buff[100];
	int xmm = 1000*x, ymm = 1000*y, zmm = 1000*z;
	//    amm = 1000*a, bmm = 1000*b, cmm = 1000*c;
	sprintf(buff, "s:%5d,%5d,%5d\r\n", xmm,ymm,zmm);
	while (SendBuffStartDMA(buff, strlen(buff)) == SEND_RETRY);
}
/*
 * The outer loop calls this periodically
 * A state-machine
 */
static void tag_on_period(dwDevice_t *dev)
{
	tag_timeout_cnt++;

	if (tag_current_state == TAG_INIT) {
		if (anchor_focus < nr_anchors) {
			/*
			 * Ask the anchors one by one
			 */
			if (is_get_reply) {
				is_get_reply = false;
				anchor_focus++;
				TagSendInitiatePacket(dev, tag_address, MSG_INIT_ANCHOR_POS);
			} else if (tag_timeout_cnt > TAG_LOOP_TIMEOUT){
				// Timeout
				LED_TUGGLE(2);
				TagSendInitiatePacket(dev, tag_address, MSG_INIT_ANCHOR_POS);
			}
		} else {
			/*
			 * Got all anchors' self-measurement.
			 */
			char buff[100];
			sprintf(buff, "\r\nAnchor coordinate:\r\n");
			while (SendBuffStartDMA(buff, strlen(buff)) == SEND_RETRY);
			for(int i = 0; i < nr_anchors; i++){
				int xmm = 1000*anchor_coordinate[i].x, 
				    ymm = 1000*anchor_coordinate[i].y,
				    zmm = 1000*anchor_coordinate[i].z;
				sprintf(buff, "(%6d,%6d,%6d)\r\n", xmm, ymm, zmm);
				while (SendBuffStartDMA(buff, strlen(buff)) == SEND_RETRY);
			}

			tag_current_state = TAG_READY;
			is_get_reply = true;
			anchor_focus = -1;			
		}
	} else {
		/*
		 * Measurement routine
		 */
		LED_ON(1);
		if (is_get_reply) {
			// Turn to the next anchor
			is_get_reply = false;
			anchor_focus++;
			if (anchor_focus >= nr_anchors) {
				static int cnt = 0;
				PrintResult();
				cnt++;
				if (cnt > 5) {
					struct coordinate_mm coomm;
					cnt = 0;
					coomm.x = my_coordinate.x*1000;
					coomm.y = my_coordinate.y*1000;
					coomm.z = my_coordinate.z*1000;
					TagSendToRouter(dev, tag_address, DEFAULT_ROUTER_ADDR, &coomm, sizeof(coomm));
					HAL_Delay(10);
				}
				// The next round
				anchor_focus = 0;
			}
			TagSendInitiatePacket(dev, tag_address, MSG_TWR_POLL);
		} else if (tag_timeout_cnt > TAG_LOOP_TIMEOUT) {
			// Timeout and try another one
			LED_TUGGLE(2);
			TagSendInitiatePacket(dev, tag_address, MSG_TWR_POLL);
		}
		LED_OFF(1);
	}
}

struct uwb_operation uwb_tag_ops = {
	.init = tag_init,
	.on_tx = tag_on_tx,
	.on_rx = tag_on_rx,
	.on_timeout = tag_on_timeout,
	.on_failed = tag_on_failed,
	.on_period = tag_on_period
};
