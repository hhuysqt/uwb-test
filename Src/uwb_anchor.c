/*
 * anchor algorithm
 */
#include <uwb.h>
#include <string.h>
#include <stdio.h>
#include <usart.h>
#include <math.h>

/*
 * Private data
 */
static enum anchor_state {
	ANCHOR_INIT,
	ANCHOR_READY
} anchor_current_state;
static uint8_t anchor_address = 0;
const double C = 299792458.0;       // Speed of light
const double tsfreq = 499.2e6 * 128;  // Timestamp counter frequency

#define ANTENNA_OFFSET 154.6   // In meter
#define ANTENNA_DELAY  (ANTENNA_OFFSET*499.2e6*128)/299792458.0 // In radio tick

/*
 * Auto position anchor:
 * Assume that anchor 0 is at the origin, anchor 1 is on x-axis, anchor 2 is 
 * above x-axis. On initialization, anchor 1 would ask for distance from 0, 
 * and anchor 2 would ask from 0 and 1. Tags would firstly ask the anchors  
 * for their positions, and then calculate their coordinate by thenselves.
 **/
static double anchor_distance[NR_ANCHORS] __attribute__((aligned(8)));
static bool is_plot = true;
static bool is_measure_once = true;
static int ploting_anchor = -1;
static struct twr_full_data anchor_ranging;
/*
 * anchor timer
 *  Maintain a counter to dianose send/receive timeout
 *  The cnt is increased only in on_period(), and is resetted in on_tx()/on_rx()
 *  and some other places...
 */
#  define ANCHOR_LOOP_TIMEOUT 100
static int anchor_timeout_cnt = 0;

/*
 * One data struct per tag...
 */
#define NR_MAX_TAGS 100
static struct twr_report active_tags[NR_MAX_TAGS];

/*
 * Global temporary packet
 */
static packet_t txPacket;
static packet_t rxPacket;

/*
 * Triggers a twr ranging.
 * Only used in ANCHOR_INIT, for anchor auto positioning
 */
static void AnchorSendInitiatePacket(dwDevice_t *dev, uint8_t my_addr)
{
	uint8_t address_tmp[8] = {0,0,0,0,0,0,0xcf,0xbc};

	dwIdle(dev);

	txPacket.payload[PAYLOAD_TYPE] = MSG_TWR_POLL; 
	address_tmp[0] = my_addr;
	memcpy(txPacket.sourceAddress, address_tmp, 8);
	address_tmp[0] = ploting_anchor;
	memcpy(txPacket.destAddress, address_tmp, 8);

	dwNewTransmit(dev);
	dwSetDefaults(dev);
	dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);

	dwWaitForResponse(dev, true);
	dwStartTransmit(dev);

	anchor_timeout_cnt = 0;
}

static void anchor_init(uint8_t addr)
{
	anchor_address = addr;
	anchor_current_state = ANCHOR_INIT;
	// Initialize the packet in the TX buffer
	MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);
	txPacket.pan = 0xbccf;
}

/*
 * Callbacks
 */
static void anchor_on_tx(dwDevice_t *dev)
{
	anchor_timeout_cnt = 0;

	dwTime_t departure;
	dwGetTransmitTimestamp(dev, &departure);
	departure.full += (ANTENNA_DELAY / 2);

	switch (anchor_current_state) {
	case ANCHOR_INIT: {
		// anchor auto ranging
		switch (txPacket.payload[PAYLOAD_TYPE]) {
		case MSG_TWR_POLL:
			anchor_ranging.poll_tx = departure;
			break;
		case MSG_TWR_FINAL:
			anchor_ranging.final_tx = departure;
			break;
		}
		break;
	}
	case ANCHOR_READY: {
		// anchor response to tags
		switch (txPacket.payload[PAYLOAD_TYPE]) {
		case MSG_TWR_ANSWER:
			active_tags[txPacket.destAddress[0]].answer_tx = departure;
			break;
		case MSG_TWR_REPORT:
			break;
		}
		break;
	}
	default: 
		// bug here
		break;
	}
}

static void anchor_on_rx(dwDevice_t *dev) {
	anchor_timeout_cnt = 0;

	dwTime_t arival = { .full=0 };
	int dataLength = dwGetDataLength(dev);

	if (dataLength == 0)
		return;

	memset(&rxPacket, 0, MAC802154_HEADER_LENGTH);
	dwGetData(dev, (uint8_t*)&rxPacket, dataLength);
	dwGetReceiveTimestamp(dev, &arival);
	arival.full -= (ANTENNA_DELAY / 2);

	if (rxPacket.destAddress[0] != anchor_address) {
		dwNewReceive(dev);
		dwSetDefaults(dev);
		dwStartReceive(dev);
		return;
	}

	memcpy(txPacket.destAddress, rxPacket.sourceAddress, 8);
	memcpy(txPacket.sourceAddress, rxPacket.destAddress, 8);
	switch(rxPacket.payload[PAYLOAD_TYPE]) {
	// Anchor auto positioning
	case MSG_TWR_ANSWER:
		txPacket.payload[0] = MSG_TWR_FINAL;

		dwNewTransmit(dev);
		dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);

		dwWaitForResponse(dev, true);
		dwStartTransmit(dev);

		anchor_ranging.answer_rx = arival;
		break;
	case MSG_TWR_REPORT: {
		/*
		 * Report comes, and it's time to calculate the distance
		 */
		struct twr_report_turncated *report =
			(struct twr_report_turncated*)(rxPacket.payload+2);
		double tround1, treply1, treply2, tround2, tprop_ctn, tprop,
		       cur_distance;
		static double distance_sum = 0;
		static int anchor_plot_cnt = 0;

		memcpy(&anchor_ranging.poll_rx,   &report->poll_rx,   5);
		memcpy(&anchor_ranging.answer_tx, &report->answer_tx, 5);
		memcpy(&anchor_ranging.final_rx,  &report->final_rx,  5);
		tround1 = anchor_ranging.answer_rx.low32 - anchor_ranging.poll_tx.low32;
		treply1 = anchor_ranging.answer_tx.low32 - anchor_ranging.poll_rx.low32;
		tround2 = anchor_ranging.final_rx.low32 - anchor_ranging.answer_tx.low32;
		treply2 = anchor_ranging.final_tx.low32 - anchor_ranging.answer_rx.low32;

		tprop_ctn = ((tround1*tround2) - (treply1*treply2)) /
			(tround1 + tround2 + treply1 + treply2);

		tprop = tprop_ctn/tsfreq;
		cur_distance = C * tprop;
		is_measure_once = true;
#  define ANCHOR_PLOT_ROUND 100
		if (anchor_plot_cnt > ANCHOR_PLOT_ROUND) {
			char buff[100];
			double calc_dist = distance_sum / ANCHOR_PLOT_ROUND;
			int distmm = calc_dist * 1000;
			anchor_distance[ploting_anchor] = calc_dist;
			sprintf(buff, "\r\nPloted anchor %d, %dmm\r\n", 
					ploting_anchor, distmm);
			SendBuffStartDMA(buff, strlen(buff));
			is_plot = true;
			distance_sum = 0;
			anchor_plot_cnt = 0;
		} else {
			distance_sum += cur_distance;
			anchor_plot_cnt++;
		}

		break;
	}

	// Anchor received messages
	case MSG_TWR_POLL: {
		uint8_t tag_id = rxPacket.sourceAddress[0];

		active_tags[tag_id].poll_rx = arival;

		txPacket.payload[PAYLOAD_TYPE] = MSG_TWR_ANSWER;
		dwNewTransmit(dev);
		dwSetDefaults(dev);
		dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH + 2);

		dwWaitForResponse(dev, true);
		dwStartTransmit(dev);

		break;
	}
	case MSG_TWR_FINAL: {
		/*
		 * Send REPORT packet to destination tag
		 */
		uint8_t tag_id = rxPacket.sourceAddress[0];

		active_tags[tag_id].final_rx = arival;

		txPacket.payload[PAYLOAD_TYPE] = MSG_TWR_REPORT;
		struct twr_report_turncated *report =
			(struct twr_report_turncated*)(txPacket.payload+2);
		memcpy(&report->poll_rx,   &active_tags[tag_id].poll_rx,   5);
		memcpy(&report->answer_tx, &active_tags[tag_id].answer_tx, 5);
		memcpy(&report->final_rx,  &active_tags[tag_id].final_rx,  5);

		dwNewTransmit(dev);
		dwSetDefaults(dev);
		dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH + 2 +
				sizeof(struct twr_report_turncated));

		dwWaitForResponse(dev, true);
		dwStartTransmit(dev);

		break;
	}
	case MSG_INIT_ANCHOR_POS: {
		// anchor responses its positions
		txPacket.payload[PAYLOAD_TYPE] = MSG_INIT_ANCHOR_POS;
		memcpy(&txPacket.payload[2], anchor_distance, sizeof(anchor_distance));

		dwNewTransmit(dev);
		dwSetDefaults(dev);
		dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH + 2 +
				sizeof(anchor_distance));

		dwWaitForResponse(dev, true);
		dwStartTransmit(dev);

		break;
	}
	default:
		break;
	}
}

static void anchor_on_failed(dwDevice_t *dev)
{
	LED_ON(1);
	dwIdle(dev);
	HAL_Delay(100);
	dwNewReceive(dev);
	dwSetDefaults(dev);
	dwStartReceive(dev);
}

static void anchor_on_timeout(dwDevice_t *dev)
{
	LED_ON(3);
	dwIdle(dev);
	HAL_Delay(100);
	dwNewReceive(dev);
	dwSetDefaults(dev);
	dwStartReceive(dev);
}

/*
 * The outer loop calls this periodically.
 * a state-machine
 */
static void anchor_on_period(dwDevice_t *dev)
{
	anchor_timeout_cnt++;

	if (anchor_current_state == ANCHOR_INIT) {
		if (anchor_address > ploting_anchor) {
			/*
			 * Plot the anchor with lower ID, one by one.
			 */
			if (is_measure_once) {
				// Initiate another measurement
				is_measure_once = false;
				AnchorSendInitiatePacket(dev, anchor_address);
			} else if (is_plot) {
				// plot next anchor
				is_plot = false;
				ploting_anchor ++;
				AnchorSendInitiatePacket(dev, anchor_address);
			} else if (anchor_timeout_cnt > ANCHOR_LOOP_TIMEOUT){
				// Timeout and retry
				LED_TUGGLE(2);
				AnchorSendInitiatePacket(dev, anchor_address);
			}
		} else {
			/*
			 * Self-measurement done
			 */
			anchor_current_state = ANCHOR_READY;
		}
	} else if (anchor_timeout_cnt > ANCHOR_LOOP_TIMEOUT) {
		anchor_timeout_cnt = 0;
		LED_TUGGLE(2);
		dwNewReceive(dev);
		dwSetDefaults(dev);
		dwStartReceive(dev);
	}
}

struct uwb_operation uwb_anchor_ops = {
	.init = anchor_init,
	.on_tx = anchor_on_tx,
	.on_rx = anchor_on_rx,
	.on_timeout = anchor_on_timeout,
	.on_failed = anchor_on_failed,
	.on_period = anchor_on_period
};
