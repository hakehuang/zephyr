/*
 * CAN driver for Geschwister Schneider USB/CAN devices
 *
 * Copyright (c) 2024 Alexander Kozhinov
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/can.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/gs_usb.h>
#include <usb_descriptor.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(gs_usb, CONFIG_GS_USB_LOG_LEVEL);


#define GS_USB_DEFAULT_IN_EP_ADDR	0x81
#define GS_USB_DEFAULT_OUT_EP_ADDR	0x01

#define GS_USB_IN_EP_IDX		0
#define GS_USB_OUT_EP_IDX		1

#define GS_USB_NUM_USB_ENDPOINTS	2

/**
 * Host format sent by the driver:
 * refer to linux-kernel host driver:
 * https://github.com/torvalds/linux/blob/master/drivers/net/can/usb/gs_usb.c#L1416
 */
#define GS_USB_HOST_FORMAT_EXPECTED		0x0000beef

/* ToDo: Features must be selectable over KConfig */
#define GS_CAN_FEATURE_LISTEN_ONLY				BIT(0)
#define GS_CAN_FEATURE_LOOP_BACK				BIT(1)
#define GS_CAN_FEATURE_TRIPLE_SAMPLE				BIT(2)
#define GS_CAN_FEATURE_ONE_SHOT					BIT(3)
#define GS_CAN_FEATURE_HW_TIMESTAMP				BIT(4)
#define GS_CAN_FEATURE_IDENTIFY					BIT(5)
#define GS_CAN_FEATURE_USER_ID					BIT(6)
#define GS_CAN_FEATURE_PAD_PKTS_TO_MAX_PKT_SIZE	BIT(7)
#define GS_CAN_FEATURE_FD					BIT(8)
#define GS_CAN_FEATURE_REQ_USB_QUIRK_LPC546XX	BIT(9)
#define GS_CAN_FEATURE_BT_CONST_EXT				BIT(10)
#define GS_CAN_FEATURE_TERMINATION				BIT(11)
#define GS_CAN_FEATURE_GET_STATE				BIT(13)

#define GS_CAN_MODE_NORMAL						0
#define GS_CAN_MODE_LISTEN_ONLY					BIT(0)
#define GS_CAN_MODE_LOOP_BACK					BIT(1)
#define GS_CAN_MODE_TRIPLE_SAMPLE				BIT(2)
#define GS_CAN_MODE_ONE_SHOT					BIT(3)
#define GS_CAN_MODE_HW_TIMESTAMP				BIT(4)
/* GS_CAN_FEATURE_IDENTIFY					BIT(5) */
/* GS_CAN_FEATURE_USER_ID					BIT(6) */
#define GS_CAN_MODE_PAD_PKTS_TO_MAX_PKT_SIZE			BIT(7)
#define GS_CAN_MODE_FD						BIT(8)
/* GS_CAN_FEATURE_REQ_USB_QUIRK_LPC546XX			BIT(9) */
/* GS_CAN_FEATURE_BT_CONST_EXT					BIT(10) */
/* GS_CAN_FEATURE_TERMINATION BIT(11) */
#define GS_CAN_MODE_BERR_REPORTING				BIT(12)
/* GS_CAN_FEATURE_GET_STATE					BIT(13) */


USBD_CLASS_DESCR_DEFINE(primary, 0) struct gs_usb_config {
	struct usb_if_descriptor if0;
	struct usb_ep_descriptor if0_in_ep;
	struct usb_ep_descriptor if0_out_ep;
} __packed gs_usb_desc = {
	/* Interface descriptor 0 */
	.if0 = {
		.bLength = sizeof(struct usb_if_descriptor),
		.bDescriptorType = USB_DESC_INTERFACE,
		.bInterfaceNumber = 0,
		.bAlternateSetting = 0,
		.bNumEndpoints = GS_USB_NUM_USB_ENDPOINTS,
		.bInterfaceClass = USB_BCC_VENDOR,
		.bInterfaceSubClass = USB_BCC_VENDOR,
		.bInterfaceProtocol = USB_BCC_VENDOR,
		.iInterface = 0,
	},

	/* Data Endpoint IN */
	.if0_in_ep = {
		.bLength = sizeof(struct usb_ep_descriptor),
		.bDescriptorType = USB_DESC_ENDPOINT,
		.bEndpointAddress = GS_USB_DEFAULT_IN_EP_ADDR,
		.bmAttributes = USB_DC_EP_BULK,
		.wMaxPacketSize = sys_cpu_to_le16(USB_MAX_FS_BULK_MPS),
		.bInterval = 0x01,
	},

	/* Data Endpoint OUT */
	.if0_out_ep = {
		.bLength = sizeof(struct usb_ep_descriptor),
		.bDescriptorType = USB_DESC_ENDPOINT,
		.bEndpointAddress = GS_USB_DEFAULT_OUT_EP_ADDR,
		.bmAttributes = USB_DC_EP_BULK,
		.wMaxPacketSize = sys_cpu_to_le16(USB_MAX_FS_BULK_MPS),
		.bInterval = 0x01,
	},
};

static void gs_usb_ep_out_cb(uint8_t ep, enum usb_dc_ep_cb_status_code ep_status);
static void gs_usb_ep_in_cb(uint8_t ep, enum usb_dc_ep_cb_status_code ep_status);

static struct usb_ep_cfg_data ep_data[] = {
	{
		.ep_cb = gs_usb_ep_in_cb,
		.ep_addr = GS_USB_DEFAULT_IN_EP_ADDR,
	},
	{
		.ep_cb = gs_usb_ep_out_cb,
		.ep_addr = GS_USB_DEFAULT_OUT_EP_ADDR,
	}
};

enum gs_usb_breq {
	GS_USB_BREQ_HOST_FORMAT = 0,
	GS_USB_BREQ_BITTIMING,
	GS_USB_BREQ_MODE,
	GS_USB_BREQ_BERR,
	GS_USB_BREQ_BT_CONST,
	GS_USB_BREQ_DEVICE_CONFIG,
	GS_USB_BREQ_TIMESTAMP,
	GS_USB_BREQ_IDENTIFY,
	GS_USB_BREQ_GET_USER_ID,
	GS_USB_BREQ_SET_USER_ID,
	GS_USB_BREQ_DATA_BITTIMING,
	GS_USB_BREQ_BT_CONST_EXT,
	GS_USB_BREQ_SET_TERMINATION,
	GS_USB_BREQ_GET_TERMINATION,
	GS_USB_BREQ_GET_STATE,
};

enum gs_can_mode {
	GS_CAN_MODE_RESET = 0,  /* reset a channel. turns it off */
	GS_CAN_MODE_START  /* starts a channel */
};

enum gs_can_identify_mode {
	GS_CAN_IDENTIFY_OFF = 0,
	GS_CAN_IDENTIFY_ON
};

enum gs_can_state {
	GS_CAN_STATE_ERROR_ACTIVE = 0,
	GS_CAN_STATE_ERROR_WARNING,
	GS_CAN_STATE_ERROR_PASSIVE,
	GS_CAN_STATE_BUS_OFF,
	GS_CAN_STATE_STOPPED,
	GS_CAN_STATE_SLEEPING
};

enum gs_can_termination_state {
	GS_CAN_TERMINATION_UNSUPPORTED = -1,
	GS_CAN_TERMINATION_STATE_OFF = 0,
	GS_CAN_TERMINATION_STATE_ON,
};

struct gs_host_config {
	uint32_t byte_order;
} __packed __aligned(4);

struct gs_device_mode {
	uint32_t mode;
	uint32_t flags;
} __packed __aligned(4);

struct gs_usb_can_handle {
	const struct device *dev;
	const struct gpio_dt_spec *rx_led;
	const struct gpio_dt_spec *tx_led;
	const struct gpio_dt_spec *term_sw;
	struct gs_device_mode dm;
};

/*
* CAN bit-timing parameters
*
* For further information, please read chapter "8 BIT TIMING
* REQUIREMENTS" of the "Bosch CAN Specification version 2.0"
* at http://esd.cs.ucr.edu/webres/can20.pdf
*/
struct gs_device_bittiming {
	uint32_t prop_seg;  /* Propagation segment in TQs */
	uint32_t phase_seg1;  /* Phase buffer segment 1 in TQs */
	uint32_t phase_seg2;  /* Phase buffer segment 2 in TQs */
	uint32_t sjw;  /* Synchronisation jump width in TQs */
	uint32_t brp;  /* Bit-rate prescaler */
} __packed __aligned(4);

/*
* CAN harware-dependent bit-timing constant
*
* Used for calculating and checking bit-timing parameters
*/
struct gs_device_bt_const_can {
	uint32_t feature;
	uint32_t fclk_can;  /* CAN system clock frequency in Hz */
	uint32_t tseg1_min;  /* Time segment 1 = prop_seg + phase_seg1 */
	uint32_t tseg1_max;
	uint32_t tseg2_min;  /* Time segment 2 = phase_seg2 */
	uint32_t tseg2_max;
	uint32_t sjw_max;  /* Synchronisation jump width */
	uint32_t brp_min;  /* Bit-rate prescaler */
	uint32_t brp_max;
	uint32_t brp_inc;  /* Bit-rate prescaler increment */
} __packed __aligned(4);

struct gs_device_bt_const_can_fd {
	/* CAN-FD data field segment timings */
	uint32_t dtseg1_min;
	uint32_t dtseg1_max;
	uint32_t dtseg2_min;
	uint32_t dtseg2_max;
	uint32_t dsjw_max;
	uint32_t dbrp_min;
	uint32_t dbrp_max;
	uint32_t dbrp_inc;
} __packed __aligned(4);

struct gs_can_frame {
	uint8_t data[CAN_MAX_DLEN];
#if defined(CONFIG_CAN_RX_TIMESTAMP)
	uint32_t timestamp_us;
#endif  /*  */
} __packed __aligned(4);

/* Frame sent by host to device */
struct gs_host_frame {
	uint32_t echo_id;
	uint32_t can_id;

	uint8_t can_dlc;
	uint8_t channel;
	uint8_t flags;
	uint8_t reserved;

	struct gs_can_frame can_frame;
} __packed __aligned(4);

struct gs_device_bt_const {
	struct gs_device_bt_const_can can;
#ifdef CONFIG_CAN_FD_MODE
	struct gs_device_bt_const_can_fd can_fd;
#endif  /* CONFIG_CAN_FD_MODE */
} __packed __aligned(4);

/* Bit timing constants */
static struct gs_device_bt_const bt_const = {
	.can = {
		.feature =
			GS_CAN_FEATURE_LISTEN_ONLY |
			GS_CAN_FEATURE_LOOP_BACK |
			GS_CAN_FEATURE_TRIPLE_SAMPLE |
			GS_CAN_FEATURE_ONE_SHOT |
#ifdef CONFIG_CAN_RX_TIMESTAMP
			GS_CAN_FEATURE_HW_TIMESTAMP |
#endif  /* CONFIG_CAN_RX_TIMESTAMP */
			GS_CAN_FEATURE_IDENTIFY |
			GS_CAN_FEATURE_PAD_PKTS_TO_MAX_PKT_SIZE
#ifdef CONFIG_GS_USB_TERM_PIN
			| GS_CAN_FEATURE_TERMINATION
#endif /* CONFIG_GS_USB_TERM_PIN */
#ifdef CONFIG_CAN_FD_MODE
			| GS_CAN_FEATURE_FD
			| GS_CAN_FEATURE_BT_CONST_EXT
#endif  /* CONFIG_CAN_FD_MODE */
		,
		.fclk_can = 0,

		.tseg1_min = 0,
		.tseg1_max = 0,
		.tseg2_min = 0,
		.tseg2_max = 0,
		.sjw_max = 0,
		.brp_min = 0,
		.brp_max = 0,
		.brp_inc = 0,
	},

#ifdef CONFIG_CAN_FD_MODE
	.can_fd = {
		.dtseg1_min = 0,
		.dtseg1_max = 0,
		.dtseg2_min = 0,
		.dtseg2_max = 0,
		.dsjw_max = 0,
		.dbrp_min = 0,
		.dbrp_max = 0,
		.dbrp_inc = 0,
	}
#endif  /* CONFIG_CAN_FD_MODE */
};

struct gs_identify_mode {
	uint32_t mode;
} __packed __aligned(4);

struct gs_device_config {
	uint8_t reserved1;
	uint8_t reserved2;
	uint8_t reserved3;
	uint8_t icount;
	uint32_t sw_version;
	uint32_t hw_version;
} __packed __aligned(4);

const struct gs_device_config dconf = {
	.icount = CONFIG_GS_USB_NUM_CAN_CHANNEL - 1,
	.sw_version = CONFIG_GS_USB_SW_VERSION,
	.hw_version = CONFIG_GS_USB_HW_VERSION,
};

struct gs_usb_can_channel_hnd {
	struct gs_usb_can_handle can_handle;
	uint32_t sw_timestamp_us;  /* software based timestamp in us */
	int32_t term_state;  /* termination pin state */
} __attribute__((aligned(4)));

static struct gs_usb_can_channel_hnd can_ch_hnd_arr[CONFIG_GS_USB_NUM_CAN_CHANNEL];

K_MSGQ_DEFINE(gs_usb_tx_can_msg_q, sizeof(struct gs_host_frame),
			  CONFIG_GS_USB_CAN_TX_MSG_QUEUE_SIZE, __alignof__(struct gs_host_frame));
K_MSGQ_DEFINE(gs_usb_rx_can_msg_q, sizeof(struct gs_host_frame),
			  CONFIG_GS_USB_CAN_RX_MSG_QUEUE_SIZE, __alignof__(struct gs_host_frame));

static void gs_usb_can_tx_thread();
static void gs_usb_can_rx_thread();

#define STACK_SIZE				2048  /* ToDo: put to KConfig */
/* ToDo: Add KConfig switch for usage of both priority modes */
#define TX_THREAD_PRIORITY		K_PRIO_COOP(8)  /* ToDo: put to KConfig */
#define RX_THREAD_PRIORITY		K_PRIO_COOP(8)  /* ToDo: put to KConfig */

K_THREAD_DEFINE(gc_usb_can_tx_tid, STACK_SIZE, gs_usb_can_tx_thread,
				NULL, NULL, NULL, TX_THREAD_PRIORITY, 0, 0);
K_THREAD_DEFINE(gc_usb_can_rx_tid, STACK_SIZE, gs_usb_can_rx_thread,
				NULL, NULL, NULL, RX_THREAD_PRIORITY, 0, 0);

static struct gs_usb_can_channel_hnd *gs_usb_get_can_channel_hnd(const uint32_t ch)
{
	return ch < ARRAY_SIZE(can_ch_hnd_arr) ? &can_ch_hnd_arr[ch] : NULL;
}

/**
 * Looks up for channel number of corresponding gs_usb channel handler
 *
 * \param can_handle - pointer to gs_usb can device handle
 * \return ch - channel number of corresponding can handler (-1 if not found)
 */
static int gs_usb_get_can_channel_from_can_dev(const struct device *dev)
{
	int ch = -1;
	for(size_t i = 0; i < ARRAY_SIZE(can_ch_hnd_arr); i++) {
		if(can_ch_hnd_arr[i].can_handle.dev == dev) {
			ch = i;
			break;
		}
	}
	return ch;
}


static enum gs_can_termination_state gs_usb_get_can_term(const uint32_t ch)
{
	const struct gs_usb_can_channel_hnd* can_ch_hnd = gs_usb_get_can_channel_hnd(ch);
	if(can_ch_hnd == NULL) {
		return GS_CAN_TERMINATION_UNSUPPORTED;
	}
	return can_ch_hnd->term_state;
}

int gs_usb_set_can_term(const uint32_t ch, const int32_t state)
{
	struct gs_usb_can_channel_hnd* can_ch_hnd = gs_usb_get_can_channel_hnd(ch);
	int ret = -ENOTSUP;
	if(can_ch_hnd == NULL) {
		return ret;
	}

	if (state == (int32_t)GS_CAN_TERMINATION_UNSUPPORTED) {
		return ret;
	}

	if (state == (int32_t)GS_CAN_TERMINATION_STATE_ON) {
		if (can_ch_hnd->can_handle.term_sw) {
			gpio_pin_set_dt(can_ch_hnd->can_handle.term_sw, 1);
		}
		can_ch_hnd->term_state = (uint32_t)GS_CAN_TERMINATION_STATE_ON;
	} else if (state == (int32_t)GS_CAN_TERMINATION_STATE_OFF) {
		if (can_ch_hnd->can_handle.term_sw) {
			gpio_pin_set_dt(can_ch_hnd->can_handle.term_sw, 0 );
		}
		can_ch_hnd->term_state = (uint32_t)GS_CAN_TERMINATION_STATE_OFF;
	} else {
		return ret;
	}
	ret = 0;
	return ret;
}

static void gs_usb_can_rx_cb(const struct device *dev, struct can_frame *frame,
							 void *user_data)
{
	/* ToDo: using k_fifo may be faster here since whis function will be called in ISR context? */
	int ret = 0;
	struct gs_host_frame hf = { 0 };

	LOG_HEXDUMP_DBG(frame, sizeof(struct can_frame), "got frame from CAN bus:");

	const int ch = gs_usb_get_can_channel_from_can_dev(dev);
	if (ch < 0) {
		LOG_ERR("Invalid CAN channel number: %d", ch);
		return;
	}

	hf.channel = ch;
	hf.echo_id = 0xFFFFFFFF;  /* not an echo frame */

	// hf.can_frame.timestamp_us = 0;  /* ToDo: set timestamp */

	hf.can_id = frame->id;
	hf.can_dlc = frame->dlc;
	// hf.flags = 0;  // ToDo: transform to hf.flags
#if defined(CONFIG_CAN_RX_TIMESTAMP)
		hf.can_frame.timestamp_us = frame->timestamp;  /* ToDo: Does can_frame has timestemp in us? */
#endif  /* CONFIG_CAN_RX_TIMESTAMP */
	memcpy(hf.can_frame.data, frame->data, CAN_MAX_DLEN);

	ret = k_msgq_put(&gs_usb_rx_can_msg_q, &hf, K_NO_WAIT);
	if (ret != 0) {
		LOG_ERR("CAN message RQ queue is full: purge old data. ret code: [%d]", ret);
		k_msgq_purge(&gs_usb_rx_can_msg_q);  /* ToDo: Here we begin to loose the messages */
		return;
	}
}

static void gs_usb_can_tx_thread()
{
	int ret = 0;
	struct gs_host_frame hf = { 0 };
	struct can_frame fr = { 0 };

	while (true) {
		memset(&hf, 0, sizeof(struct gs_host_frame));
		memset(&fr, 0, sizeof(struct can_frame));
		k_msgq_get(&gs_usb_tx_can_msg_q, &hf, K_FOREVER);

		const struct gs_usb_can_channel_hnd* can_ch_hnd = gs_usb_get_can_channel_hnd(hf.channel);
		if (can_ch_hnd == NULL) {
			LOG_ERR("Invalid can channel: %d", hf.channel);
			continue;
		}

		fr.id = hf.can_id;
		fr.dlc = hf.can_dlc;
		/* ToDo: timestamp readout from the CAN-Bus stack */
		// fr.flags = 0;  // ToDo: transoform to hf.flags
		// fr.timestamp = ???;  // ToDo: transoform hf.can_frame.timestamp
		memcpy(fr.data, hf.can_frame.data, CAN_MAX_DLEN);

		LOG_HEXDUMP_DBG(&fr, sizeof(fr), "can_send:");
		ret = can_send(can_ch_hnd->can_handle.dev, &fr, K_MSEC(100), NULL, NULL);
		if (ret != 0) {
			LOG_ERR("can_send failed [%d]", ret);
			continue;
		}

		// Echo sends frame back to host:
		ret = usb_write(ep_data[GS_USB_IN_EP_IDX].ep_addr, (const uint8_t*)&hf,
						sizeof(hf), NULL);
		if(ret < 0) {
			LOG_ERR("usb_write failed on ep: 0x%x ret: %d",
					ep_data[GS_USB_IN_EP_IDX].ep_addr, ret);
			continue;
		}

		if (can_ch_hnd->can_handle.tx_led) {
			gpio_pin_toggle_dt(can_ch_hnd->can_handle.tx_led);
		}
	}
}

static void gs_usb_ep_out_cb(uint8_t ep, enum usb_dc_ep_cb_status_code ep_status)
{
	if(ep != ep_data[GS_USB_OUT_EP_IDX].ep_addr) {
		LOG_DBG("ep != ep_data[GS_USB_OUT_EP_IDX].ep_addr --> 0x%x != 0x%x",
				ep, ep_data[GS_USB_OUT_EP_IDX].ep_addr);
		return;
	}

	int ret = 0;
	struct gs_host_frame hf = { 0 };
	uint32_t bytes_to_read = 0;

	ret = usb_read(ep, NULL, 0, &bytes_to_read);
	if(ret != 0) {
		LOG_DBG("Can not read number of bytes [%d]", ret);
		return;
	}

	ret = usb_read(ep, (uint8_t*)&hf, bytes_to_read, NULL);
	if(ret != 0) {
		LOG_DBG("Can not read host frame [%d]", ret);
		return;
	}

	LOG_HEXDUMP_DBG(&hf, sizeof(struct gs_host_frame), "got gs_host_frame:");

	ret = k_msgq_put(&gs_usb_tx_can_msg_q, &hf, K_FOREVER);
	if (ret != 0) {
		LOG_ERR("CAN message TX queue is full: purge old data. ret code: [%d]", ret);
		k_msgq_purge(&gs_usb_tx_can_msg_q);  /* ToDo: Here we begin to loose the messages */
		return;
	}
}

static void gs_usb_can_rx_thread()
{
	int ret = 0;
	struct gs_host_frame hf = {0};

	while (true) {
		memset(&hf, 0, sizeof(struct gs_host_frame));
		k_msgq_get(&gs_usb_rx_can_msg_q, &hf, K_FOREVER);
		LOG_DBG("got frame from CAN bus: 0x%x", hf.can_id);

		const struct gs_usb_can_channel_hnd* can_ch_hnd = gs_usb_get_can_channel_hnd(hf.channel);
		if (can_ch_hnd == NULL) {
			LOG_ERR("Invalid can channel: %d", hf.channel);
			continue;
		}

		ret = usb_write(ep_data[GS_USB_IN_EP_IDX].ep_addr, (const uint8_t*)&hf,
						sizeof(hf), NULL);
		if(ret < 0) {
			LOG_ERR("usb_write failed on ep: 0x%x ret: %d",
					ep_data[GS_USB_IN_EP_IDX].ep_addr, ret);
			continue;
		}

		if (can_ch_hnd->can_handle.rx_led) {
			gpio_pin_toggle_dt(can_ch_hnd->can_handle.rx_led);
		}
	}
}

static void gs_usb_ep_in_cb(uint8_t ep, enum usb_dc_ep_cb_status_code ep_status)
{
	LOG_DBG("start: ep: 0x%x", ep);
	// #warning "Value 64 must be replaced by CONFIG_GS_USB_CAN_DATA_MAX_PACKET_SIZE"
	// if (usb_write(ep, gs_usb_buf, 64, NULL)) {
	// 	LOG_DBG("ep 0x%x", ep);
	// }
}

/**
 * @brief Callback used to know the USB connection status
 *
 * @param status USB device status code.
 *
 * @return  N/A.
 */
static void gs_usb_status_cb(struct usb_cfg_data *cfg,
							 enum usb_dc_status_code status,
							 const uint8_t *param)
{
	LOG_DBG("start");
	ARG_UNUSED(param);
	ARG_UNUSED(cfg);

	/* Check the USB status and do needed action if required */
	switch (status) {
	case USB_DC_ERROR:
		LOG_DBG("USB device error");
		break;
	case USB_DC_RESET:
		LOG_DBG("USB device reset detected");
		break;
	case USB_DC_CONNECTED:
		LOG_DBG("USB device connected");
		break;
	case USB_DC_CONFIGURED:
		LOG_DBG("USB device configured");
		break;
	case USB_DC_DISCONNECTED:
		LOG_DBG("USB device disconnected");
		break;
	case USB_DC_SUSPEND:
		LOG_DBG("USB device suspended");
		break;
	case USB_DC_RESUME:
		LOG_DBG("USB device resumed");
		break;
	case USB_DC_INTERFACE:
		LOG_DBG("USB unhandled state: %d", status);
		break;
	case USB_DC_SOF:
		break;
	case USB_DC_UNKNOWN:
	default:
		LOG_DBG("USB unknown state");
		break;
	}
}

static int gs_usb_vendor_handler_fail()
{
	/* ToDo: This function need to be revised and may be not necessary. */
	usb_cancel_transfers();
	LOG_DBG("out_fail: executed...");
	return -ENOTSUP;
}

static can_mode_t gs_usb_breq_mode_flags_hnd(const uint32_t flags)
{
	can_mode_t mode = CAN_MODE_NORMAL;

	if (flags & GS_CAN_MODE_LOOP_BACK) {
		mode |= CAN_MODE_LOOPBACK;
	}

	if (flags & GS_CAN_MODE_LISTEN_ONLY) {
		mode |= CAN_MODE_LISTENONLY;
	}

	if (flags & GS_CAN_MODE_TRIPLE_SAMPLE) {
		mode |= CAN_MODE_3_SAMPLES;
	}

	if (flags & GS_CAN_MODE_ONE_SHOT) {
		mode |= CAN_MODE_ONE_SHOT;
	}

	if (flags & GS_CAN_MODE_BERR_REPORTING) {
	}

	if (flags & GS_CAN_MODE_FD) {
		mode |= CAN_MODE_FD;
	}

	// if (flags & GS_CAN_MODE_HW_TIMESTAMP) {
	// }

	return mode;
}

/**
 * Convert bytes reciverd over usb to zephyr's can_timing.
 *
 * \param data - bytes array pointer
 * \param len - length of bytes
 */
static void gs_usb_bytes_to_can_timing(uint8_t **data, int32_t *len,
										struct can_timing *can_timing)
{
	struct gs_device_bittiming gs_timing;

	memcpy(&gs_timing, *data, *len);

	can_timing->sjw = gs_timing.sjw;
	can_timing->prop_seg =  0;  /* gs_timing.prop_seg - used in can_timing.phase_seg1 */
	can_timing->phase_seg1 = gs_timing.prop_seg + gs_timing.phase_seg1;
	can_timing->phase_seg2 = gs_timing.phase_seg2;
	can_timing->prescaler = gs_timing.brp;
}

static int gs_usb_dev2host_hnd(struct gs_usb_can_channel_hnd* can_ch_hnd,
								struct usb_setup_packet *setup,
								int32_t *len, uint8_t **data)
{
	int ret = 0;
	switch (setup->bRequest) {
		case GS_USB_BREQ_HOST_FORMAT:
			LOG_DBG("GS_USB_BREQ_HOST_FORMAT");
			/* Not supported */
			break;
		case GS_USB_BREQ_BITTIMING:
			LOG_DBG("GS_USB_BREQ_BITTIMING");
			*len = MIN(sizeof(struct gs_device_bittiming), setup->wLength);
			break;
		case GS_USB_BREQ_BT_CONST:
			*data = (uint8_t*)&bt_const;
			*len = MIN(sizeof(struct gs_device_bt_const_can), setup->wLength);
			LOG_HEXDUMP_DBG(*data, *len, "GS_USB_BREQ_BT_CONST:");
			break;
		case GS_USB_BREQ_BT_CONST_EXT:
#ifdef CONFIG_CAN_FD_MODE
			*data = (uint8_t*)&bt_const;
			*len = MIN(sizeof(struct gs_device_bt_const_can) + sizeof(struct gs_device_bt_const_can_fd),
					   setup->wLength);
			LOG_HEXDUMP_DBG(*data, *len, "GS_USB_BREQ_BT_CONST_EXT:");
#endif  /* CONFIG_CAN_FD_MODE */
			break;
		case GS_USB_BREQ_DEVICE_CONFIG:
			*data = (uint8_t*)&dconf;
			*len = MIN(sizeof(struct gs_device_config), setup->wLength);
			LOG_HEXDUMP_DBG(*data, *len, "GS_USB_BREQ_DEVICE_CONFIG:");
			break;
		case GS_USB_BREQ_TIMESTAMP:
			*data = (uint8_t*)&can_ch_hnd->sw_timestamp_us;
			*len = MIN(sizeof(can_ch_hnd->sw_timestamp_us), setup->wLength);
			LOG_HEXDUMP_DBG(*data, *len, "GS_USB_BREQ_TIMESTAMP:");
			break;
		case GS_USB_BREQ_GET_TERMINATION:
			int32_t state = 0;
			state = gs_usb_get_can_term(setup->wValue);
			*len =  MIN(sizeof(state), setup->wLength);
			memcpy(*data, &state, *len);
			LOG_HEXDUMP_DBG(*data, *len, "GS_USB_BREQ_GET_TERMINATION:");
			break;
		case GS_USB_BREQ_GET_STATE:
			LOG_HEXDUMP_DBG(*data, *len, "GS_USB_BREQ_GET_STATE:");
			break;
		default:
			LOG_HEXDUMP_DBG(*data, *len, "unknown command:");
			break;
	}
	return ret;
}

static int gs_usb_host2dev_hnd(struct gs_usb_can_channel_hnd* can_ch_hnd,
								struct usb_setup_packet *setup,
								int32_t *len, uint8_t **data)
{
	int ret = 0;
	switch (setup->bRequest) {
		case GS_USB_BREQ_HOST_FORMAT:
			LOG_DBG("GS_USB_BREQ_HOST_FORMAT");
			uint32_t host_format = 0;
			memcpy(&host_format, *data, *len);
			if(GS_USB_HOST_FORMAT_EXPECTED != host_format) {
				ret = -ENOTSUP;
			}
			break;
		case GS_USB_BREQ_BITTIMING:
			LOG_DBG("GS_USB_BREQ_BITTIMING");
			struct can_timing can_timing;
			gs_usb_bytes_to_can_timing(data, len, &can_timing);
			ret = can_set_timing(can_ch_hnd->can_handle.dev, &can_timing);
			break;
		case GS_USB_BREQ_DATA_BITTIMING:
			LOG_DBG("GS_USB_BREQ_DATA_BITTIMING");
#ifdef CONFIG_CAN_FD_MODE
			struct can_timing can_timing;
			gs_usb_bytes_to_can_timing(data, len, &can_timing);
			ret = can_set_timing_data(can_ch_hnd->can_handle.dev, &can_timing);
#endif  /* CONFIG_CAN_FD_MODE */
			break;
		case GS_USB_BREQ_MODE:
			// will be called on: sudo ip link set can0 up
			LOG_HEXDUMP_DBG(*data, *len, "GS_USB_BREQ_MODE:");

			struct gs_device_mode dm = {0};
			memcpy(&dm, *data, sizeof(struct gs_device_mode));

			const can_mode_t can_mode = gs_usb_breq_mode_flags_hnd(dm.flags);
			if (can_ch_hnd->can_handle.dm.mode == GS_CAN_MODE_RESET) {
				LOG_DBG("can_set_mode: mode: 0x%x", can_mode);
				ret = can_set_mode(can_ch_hnd->can_handle.dev, can_mode);
				if (ret < 0) {
					LOG_ERR("Failed to handle gs_usb mode flags [0x%x]: [%d]",
							dm.flags, ret);
					return ret;
				}
				can_ch_hnd->can_handle.dm.flags = dm.flags;
			}

			if (dm.mode == GS_CAN_MODE_RESET) {
				ret = can_stop(can_ch_hnd->can_handle.dev);
				if (ret < 0) {
					LOG_ERR("Failed to stop CAN controller: [%d]", ret);
					return ret;
				}
			} else if(dm.mode == GS_CAN_MODE_START) {
				ret = can_start(can_ch_hnd->can_handle.dev);
				if (ret == -EIO) {
					LOG_ERR("Failed to start CAN controller: [%d]", ret);
					return ret;
				}
			}
			can_ch_hnd->can_handle.dm.mode = dm.mode;
			break;
		case GS_USB_BREQ_BT_CONST:
			LOG_DBG("GS_USB_BREQ_BT_CONST");
			break;
		case GS_USB_BREQ_BT_CONST_EXT:
			LOG_DBG("GS_USB_BREQ_BT_CONST_EXT");
			break;
		case GS_USB_BREQ_DEVICE_CONFIG:
			LOG_DBG("GS_USB_BREQ_DEVICE_CONFIG");
			break;
		case GS_USB_BREQ_SET_TERMINATION:
			LOG_HEXDUMP_DBG(*data, *len, "GS_USB_BREQ_SET_TERMINATION:");
			int32_t new_state;
			memcpy(&new_state, *data, *len);
			if (gs_usb_set_can_term(setup->wValue, new_state)) {
				return gs_usb_vendor_handler_fail();
			}
			break;
		case GS_USB_BREQ_IDENTIFY:
			/* ToDo: Implement one of LED's blinking (which one?) */
			LOG_HEXDUMP_DBG(*data, *len, "GS_USB_BREQ_IDENTIFY:");
			break;
		case GS_USB_BREQ_GET_STATE:
			LOG_HEXDUMP_DBG(*data, *len, "GS_USB_BREQ_GET_STATE:");
		default:
			LOG_HEXDUMP_DBG(*data, *len, "unknown command:");
			break;
	}

	return ret;
}

static int gs_usb_vendor_handler(struct usb_setup_packet *setup,
								 int32_t *len, uint8_t **data)
{
	int ret = 0;
	struct gs_usb_can_channel_hnd *can_ch_hnd = NULL;

	/*
	 * For all "per device" USB control messages
	 * (GS_USB_BREQ_HOST_FORMAT and GS_USB_BREQ_DEVICE_CONFIG) the
	 * Linux gs_usb driver uses a setup->wValue = 1.
	 *
	 * All other control messages are "per channel" and specify the
	 * channel number in setup->wValue. So check setup->wValue for valid
	 * CAN channel.
	 */
	if (!(setup->bRequest == GS_USB_BREQ_HOST_FORMAT ||
		  setup->bRequest == GS_USB_BREQ_DEVICE_CONFIG)) {
		can_ch_hnd = gs_usb_get_can_channel_hnd(setup->wValue);
		if (!can_ch_hnd) {
			return gs_usb_vendor_handler_fail();
		}
	}

	// Host - is a PC
	// Dev - is this MCU
	if (usb_reqtype_is_to_device(setup)) {
		ret = gs_usb_host2dev_hnd(can_ch_hnd, setup, len, data);
	} else {
		ret = gs_usb_dev2host_hnd(can_ch_hnd, setup, len, data);
	}

	return ret;
}

static int gs_usb_class_handler(struct usb_setup_packet *setup,
								int32_t *len, uint8_t **data)
{
	LOG_DBG("start");
	return 0;
}

static void gs_interface_config(struct usb_desc_header *head,
									uint8_t bInterfaceNumber)
{
	ARG_UNUSED(head);
	gs_usb_desc.if0.bInterfaceNumber = bInterfaceNumber;
}

static int gs_usb_init_internal(void)
{
	int ret = 0;
	return ret;
}

int gs_usb_init(const struct device *can_dev,
				const struct gpio_dt_spec *rx_led,
				const struct gpio_dt_spec *tx_led,
				const struct gpio_dt_spec *term_res_gpio)
{
	int ret = 0;

	ARG_UNUSED(tx_led);
	ARG_UNUSED(term_res_gpio);

	const uint32_t ch = 0;  // ToDo: It must be able to handle CONFIG_GS_USB_NUM_CAN_CHANNEL
	can_ch_hnd_arr[ch].can_handle.dev = can_dev;
#ifdef CONFIG_GS_USB_TERM_PIN
	can_ch_hnd_arr[ch].term_state = GS_CAN_TERMINATION_STATE_OFF;
#else
	can_ch_hnd_arr[ch].term_state = GS_CAN_TERMINATION_UNSUPPORTED;
#endif  /* CONFIG_GS_USB_TERM_PIN */

	ret = can_get_core_clock(can_ch_hnd_arr[ch].can_handle.dev, &bt_const.can.fclk_can);
	if (ret != 0) {
		return ret;
	}

	/* ToDo: supported capabilities shall define features here */
	can_mode_t cap;
	ret = can_get_capabilities(can_ch_hnd_arr[ch].can_handle.dev, &cap);
	if (ret != 0) {
		return ret;
	}

	const struct can_timing *max = can_get_timing_max(can_ch_hnd_arr[ch].can_handle.dev);
	const struct can_timing *min = can_get_timing_min(can_ch_hnd_arr[ch].can_handle.dev);

	bt_const.can.tseg1_min = min->phase_seg1;
	bt_const.can.tseg1_max = max->phase_seg1;
	bt_const.can.tseg2_min = min->phase_seg2;
	bt_const.can.tseg2_max = max->phase_seg2;
	bt_const.can.sjw_max = max->sjw;
	bt_const.can.brp_min = min->prescaler;
	bt_const.can.brp_max = max->prescaler;
	/*
	* This value can not be extracted over CAN-API in zephyr at the moment.
	* can-utils uses value of 1:
	* https://github.com/linux-can/can-utils/blob/master/calc-bit-timing/can-calc-bit-timing.c
	*/
	bt_const.can.brp_inc = 1;

#ifdef CONFIG_CAN_FD_MODE
	/* Data segment timings of CAN-FD */
	max = can_get_timing_data_max(can_ch_hnd_arr[ch].can_handle.dev);
	min = can_get_timing_data_min(can_ch_hnd_arr[ch].can_handle.dev);

	bt_const.can_fd.dtseg1_min = min->phase_seg1;
	bt_const.can_fd.dtseg1_max = max->phase_seg1;
	bt_const.can_fd.dtseg2_min = min->phase_seg2;
	bt_const.can_fd.dtseg2_max = max->phase_seg2;
	bt_const.can_fd.dsjw_max = max->sjw;
	bt_const.can_fd.dbrp_min = min->prescaler;
	bt_const.can_fd.dbrp_max = max->prescaler;
	bt_const.can_fd.dbrp_inc = 1;  /* see brp_inc description above */
#endif  /* CONFIG_CAN_FD_MODE */

	const struct can_filter gs_usb_rx_filter = {
		.flags = 0U,
		.id = 0U,
		.mask = 0U
	};

	for (size_t i = 0; i < ARRAY_SIZE(can_ch_hnd_arr); i++) {
		ret = can_add_rx_filter(can_ch_hnd_arr[i].can_handle.dev, gs_usb_can_rx_cb,
								NULL, &gs_usb_rx_filter);
		if (ret < 0) {
			LOG_ERR("Unable to add can rx filter [%d]", ret);
			return ret;
		}
		LOG_ERR("Initialized can rx filter [%d]", ret);
	}
	/* Reset it. In this loop context it is used as returned filter_id of
	* can_add_rx_filter function above.
	*/
	ret = 0;

	// ToDo: configure can channel device mode and flags on init
	// can_ch_hnd_arr[ch].can_handle.dm.mode =

	can_ch_hnd_arr[ch].can_handle.rx_led = rx_led;
	if (can_ch_hnd_arr[ch].can_handle.rx_led) {
		gpio_pin_configure_dt(can_ch_hnd_arr[ch].can_handle.rx_led,
							  GPIO_OUTPUT_INACTIVE);
	}

	can_ch_hnd_arr[ch].can_handle.tx_led = tx_led;
	if (can_ch_hnd_arr[ch].can_handle.tx_led) {
		gpio_pin_configure_dt(can_ch_hnd_arr[ch].can_handle.tx_led,
							  GPIO_OUTPUT_INACTIVE);
	}

	can_ch_hnd_arr[ch].can_handle.term_sw = term_res_gpio;
	if (can_ch_hnd_arr[ch].can_handle.term_sw) {
		gpio_pin_configure_dt(can_ch_hnd_arr[ch].can_handle.term_sw,
							  GPIO_OUTPUT_INACTIVE);
	}

	return ret;
}


int gs_usb_start(const uint32_t ch)
{
	int ret = 0;

	// ToDo: configure can channel device mode and flags on start
	// can_ch_hnd_arr[ch].can_handle.dm.mode =

	ret = usb_enable(NULL);
	if (ret != 0) {
		LOG_ERR("Failed to enable USB");
		return ret;
	}

	LOG_DBG("ep_data[GS_USB_IN_EP_IDX].ep_addr: 0x%x", ep_data[GS_USB_IN_EP_IDX].ep_addr);
	LOG_DBG("ep_data[GS_USB_OUT_EP_IDX].ep_addr: 0x%x", ep_data[GS_USB_OUT_EP_IDX].ep_addr);

	LOG_DBG("gs_usb_desc.if0_in_ep.bEndpointAddress: 0x%x", gs_usb_desc.if0_in_ep.bEndpointAddress);
	LOG_DBG("gs_usb_desc.if0_out_ep.bEndpointAddress: 0x%x", gs_usb_desc.if0_out_ep.bEndpointAddress);

	return ret;
}

USBD_DEFINE_CFG_DATA(gs_usb_class_config) = {
	.usb_device_description = NULL,
	.interface_descriptor = &gs_usb_desc.if0,
	.interface_config = gs_interface_config,
	.cb_usb_status = gs_usb_status_cb,
	.interface = {
		.class_handler = gs_usb_class_handler,
		.custom_handler = NULL,
		.vendor_handler = gs_usb_vendor_handler,
	},
	.num_endpoints = ARRAY_SIZE(ep_data),
	.endpoint = ep_data,
};

/* Initialize this before can device init */
SYS_INIT(gs_usb_init_internal, POST_KERNEL, 0);
