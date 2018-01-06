/*
 * uwb algorithm interface
 */
#ifndef MY_UWB_H_
#define MY_UWB_H_

#include <dw_ops.h>
#include <led.h>
#include <mac.h>

/*
 * Configurations
 */
struct uwb_config {
	unsigned short magic;
	unsigned char mode;
	unsigned char address;
	unsigned char nr_anchor;
	unsigned char checksum;
} __attribute__((packed));
extern struct uwb_config config;

/*
 * Define the number of anchors *here*
 */
#define NR_ANCHORS 3
#define MAX_NR_ANCHORS 8

struct coordinate {
	double x, y, z;
};
struct coordinate_mm {
	int16_t x, y, z;
};

/*
 * Different uwb mode
 */
enum uwb_mode {
	UWB_ANCHOR = 0,	// TOA anchor
	UWB_TAG,	// TOA tag
	UWB_TDOA_ANCHOR,// TDOA anchor
	UWB_TDOA_TAG,	// TDOA tag

	UWB_ROUTER,	// info tx/rx
	UWB_ENDMODE	// sentinel
};

#define DEFAULT_ROUTER_ADDR 50

/*
 * uwb callback functions
 */
struct uwb_operation {
	void (*init)(dwDevice_t *dev);
	void (*on_tx)(dwDevice_t *dev);
	void (*on_rx)(dwDevice_t *dev);
	void (*on_timeout)(dwDevice_t *dev);
	void (*on_failed)(dwDevice_t *dev);
	void (*on_period)(dwDevice_t *dev);	// the upper loop periodically calls it
};
extern struct uwb_operation uwb_anchor_ops;
extern struct uwb_operation uwb_tag_ops;
extern struct uwb_operation uwb_tdoa_anchor_ops;
extern struct uwb_operation uwb_tdoa_tag_ops;
extern struct uwb_operation uwb_router_ops;

/*
 * Two-Way-Ranging data struct
 */
enum twr_msg_type {
	/* The normal TWR-TOA messages */
	MSG_TWR_POLL, 
	MSG_TWR_ANSWER,
	MSG_TWR_FINAL,
	MSG_TWR_REPORT,
	/* TDOA messages */
	MSG_TDOA,
	/* User specific messages */
	MSG_INIT_ANCHOR_POS,	// Tag asks for anchor's position
	MSG_TO_ROUTER,		// send info to the router
};

enum payload_index {
	/*
	 * to index the payload uint8_t array...
	 */
	PAYLOAD_TYPE = 0,
	PAYLOAD_SEQUENCE,
	PAYLOAD_INFO_START,
};
/*
 * The report data struct, from anchor to tag
 * The anchor needs to keep one twr_report for every tag.
 * To save uwb bandwidth, every dwTime_t should be turncated to 5 bytes.
 */
struct twr_report {
	dwTime_t poll_rx;
	dwTime_t answer_tx;
	dwTime_t final_rx;
} __attribute__((packed));
struct twr_report_turncated {
	uint8_t poll_rx[5];
	uint8_t answer_tx[5];
	uint8_t final_rx[5];
} __attribute__((packed));

/*
 * the TWR data for distance calculation
 */
struct twr_full_data {
	dwTime_t poll_tx;
	dwTime_t poll_rx;
	dwTime_t answer_tx;
	dwTime_t answer_rx;
	dwTime_t final_tx;
	dwTime_t final_rx;
};

/*
 * TDOA: hyperbolic localization
 * Tags calculate their coordinate only by listening to anchors' messages
 * Anchors broadcast their timestamps one by one.
 */

void mydelay(int n);
double LowPassFilter(double input, double last_output);

/*
 * Initializing the low level radio handling
 * Return 0 on success.
 */
int uwbInit(void);

/*
 * Set default config
 */
void SetDefaultConfig(void);

/*
 * Config varibles
 */
void SetAddress(uint8_t addr);
void SetMode(uint8_t mode);
void SetNrAnchors(uint8_t n);

/*
 * Main loop
 */
void UWBMainLoop(void);

#endif

