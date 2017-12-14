/*
 * uwb algorithm interface
 */
#ifndef MY_UWB_H_
#define MY_UWB_H_

#include <dw_ops.h>
#include <uwb_tag.h>
#include <uwb_anchor.h>
#include <led.h>
#include <mac.h>

/*
 * Define the number of anchors *here*
 */
#define NR_ANCHORS 3
#define MAX_NR_ANCHORS 10

/*
 * Different uwb mode
 */
enum uwb_mode {
	UWB_ANCHOR = 0,
	UWB_TAG,
	UWB_ENDMODE	// sentinel
};

/*
 * uwb callback functions
 */
struct uwb_operation {
	void (*init)(uint8_t anchor_address);
	void (*on_tx)(dwDevice_t *dev);
	void (*on_rx)(dwDevice_t *dev);
	void (*on_timeout)(dwDevice_t *dev);
	void (*on_failed)(dwDevice_t *dev);
	void (*on_period)(dwDevice_t *dev);	// the upper loop periodically calls it
};

/*
 * Two-Way-Ranging data struct
 */
enum twr_msg_type {
	/* The normal twr messages */
	MSG_TWR_POLL, 
	MSG_TWR_ANSWER,
	MSG_TWR_FINAL,
	MSG_TWR_REPORT,
	/* User specific messages */
	MSG_INIT_ANCHOR_POS,	// Tag asks for anchor's position
};

enum twr_payload_index {
	/*
	 * to index the payload uint8_t array...
	 */
	PAYLOAD_TYPE = 0,
	PAYLOAD_SEQUENCE,
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

