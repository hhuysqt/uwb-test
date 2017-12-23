/*
 * dwm1000 algorithm init
 */
#include <eeprom.h>
#include <string.h>
#include <uwb.h>
#include <usart.h>

/*
 * dwm1000 instance
 */
static dwDevice_t dwm_dev;

/*
 * System configuration
 *  magic number: 0xabcd
 */
#define MAGIC_NUMBER 0xabcd
static struct uwb_config {
	unsigned short magic;
	unsigned char mode;
	unsigned char address;
	unsigned char nr_anchor;
	unsigned char checksum;
} __attribute__((packed)) config;

/*
 * default: anchor
 */
static struct uwb_operation *curr_uwb_ops = &uwb_anchor_ops;

/*
 * Check config
 */
static unsigned char CalcSum(struct uwb_config *the_config)
{
	unsigned char *calcbuf = (unsigned char *)the_config;
	unsigned char calcsum = 0;
	for (uint32_t i = 0; i < sizeof(struct uwb_config) - 1; i++)
		calcsum += calcbuf[i];
	return calcsum;
}

static int CheckConfig(struct uwb_config *the_config)
{
	if (the_config->magic != MAGIC_NUMBER)
		return -1;

	if (the_config->checksum != CalcSum(the_config))
		return -1;

	return 0;
}

/*
 * Load system config.
 * Set the default value if config is not right.
 * Return 0 on success.
 */
static int LoadConfig(void)
{
	eepromRead(0, &config, sizeof(config));
	if (CheckConfig(&config) != 0) {
		const char *msg = "Setting default config\r\n";
		SendBuffStartDMA((void*)msg, strlen(msg));
		SetDefaultConfig();
		eepromRead(0, &config, sizeof(config));
		if (CheckConfig(&config) != 0)
			return -1;
	}

	char buff[200];
	sprintf(buff, "\r\nCurrent config:\r\n"
	              " Address: %d\r\n"
	              " %s mode\r\n"
	              " %d anchors out there\r\n",
	              config.address, 
	              config.mode == UWB_ANCHOR ? "anchor" : 
	              config.mode == UWB_TAG ? "tag" : "router",
	              config.nr_anchor);
	SendBuffStartDMA(buff, strlen(buff));

	return 0;
}

/*
 * default callbacks
 */
void default_on_tx(dwDevice_t *dev)
{
	(void)dev;
}
void default_on_rx(dwDevice_t *dev)
{
	(void)dev;
}
void default_on_failed(dwDevice_t *dev)
{
	LED_ON(1);
	dwIdle(dev);
	HAL_Delay(100);
	dwNewReceive(dev);
	dwSetDefaults(dev);
	dwStartReceive(dev);
}
void default_on_timeout(dwDevice_t *dev)
{
	LED_ON(3);
	dwIdle(dev);
	HAL_Delay(100);
	dwNewReceive(dev);
	dwSetDefaults(dev);
	dwStartReceive(dev);
}
/*
 * Set uwb callback according to config
 */
static int SetCallbacks(dwDevice_t *the_uwb)
{
	switch (config.mode) {
	case UWB_ANCHOR:
		curr_uwb_ops = &uwb_anchor_ops;
		break;
	case UWB_TAG:
		curr_uwb_ops = &uwb_tag_ops;
		break;
	case UWB_ROUTER:
		curr_uwb_ops = &uwb_router_ops;
		break;
	default:
		return -1;
	}
	curr_uwb_ops->init(config.address);
	dwAttachSentHandler          (the_uwb, curr_uwb_ops->on_tx ?
		curr_uwb_ops->on_tx : default_on_tx);
	dwAttachReceivedHandler      (the_uwb, curr_uwb_ops->on_rx ?
		curr_uwb_ops->on_rx : default_on_rx);
	dwAttachReceiveTimeoutHandler(the_uwb, curr_uwb_ops->on_timeout ?
		curr_uwb_ops->on_timeout : default_on_timeout);
	dwAttachReceiveFailedHandler (the_uwb, curr_uwb_ops->on_failed ?
		curr_uwb_ops->on_failed : default_on_failed);

	return 0;
}


/*
 * Initializing the low level radio handling
 * Return 0 on success.
 */
int uwbInit()
{
	// Init libdw
	dwInit(&dwm_dev, &dwOps);
	if (dwConfigure(&dwm_dev) != 0) {
		return -1;
	}
	dwEnableAllLeds(&dwm_dev);
	dwTime_t delay = {.full = 0};
	dwSetAntenaDelay(&dwm_dev, delay);

	// Reading and setting node configuration
	if (LoadConfig() != 0)
		return -1;

	if (SetCallbacks(&dwm_dev) != 0)
		return -1;

	dwNewConfiguration(&dwm_dev);
	dwSetDefaults(&dwm_dev);
	dwEnableMode(&dwm_dev, MODE_SHORTDATA_FAST_ACCURACY);
	dwSetChannel(&dwm_dev, CHANNEL_2);
	dwUseSmartPower(&dwm_dev, true);
	dwSetPreambleCode(&dwm_dev, PREAMBLE_CODE_64MHZ_9);

	dwCommitConfiguration(&dwm_dev);

	return 0;
}

/*
 * Set default config
 */
void SetDefaultConfig(void)
{
	memset(&config, 0, sizeof(config));
	config.magic = MAGIC_NUMBER;
	config.mode = UWB_ANCHOR;
	config.address = 0;
	config.nr_anchor = NR_ANCHORS;
	config.checksum = CalcSum(&config);

	eepromWrite(0, &config, sizeof(config));
}
/*
 * Config varibles
 */
void SetAddress(uint8_t addr)
{
	config.address = addr;
	config.checksum = CalcSum(&config);
	eepromWrite(0, &config, sizeof(config));
}
void SetMode(uint8_t mode)
{
	config.mode = mode;
	if (mode == UWB_ROUTER)
		config.address = DEFAULT_ROUTER_ADDR;
	config.checksum = CalcSum(&config);
	eepromWrite(0, &config, sizeof(config));
}
void SetNrAnchors(uint8_t n)
{
	config.nr_anchor = n;
	config.checksum = CalcSum(&config);
	eepromWrite(0, &config, sizeof(config));
}

/*
 * Main loop
 */
void UWBMainLoop(void)
{
	while (1) {
		curr_uwb_ops->on_period(&dwm_dev);
		/* handle usrt input */
		DoReceive();
		HAL_Delay(1);
	}
}

/*
 * DWM1000 IRQ
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == DWM1000_IRQ_Pin) {
		HAL_NVIC_ClearPendingIRQ(DWM1000_IRQn);
		dwHandleInterrupt(&dwm_dev);
	}
}

