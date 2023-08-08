/*
 * Copyright 2023 NXP
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * based on dmic_nrfx_pdm.c
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/dma.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/drivers/pinctrl.h>
#include <soc.h>

#include <fsl_dmic.h>

#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
LOG_MODULE_REGISTER(dmic_mcux, CONFIG_AUDIO_DMIC_LOG_LEVEL);

#define DT_DRV_COMPAT nxp_mcux_dmic

struct mcux_dmic_pdm_chan {
     bool enabled;
     bool active;
     bool hwvad_enabled;
     bool use2fs;
     struct device *dev_dma;
     dmic_channel_config_t dmic_channel_cfg;
};

struct mcux_dmic_drv_data {
	//struct onoff_manager *clk_mgr;
	//struct onoff_client clk_cli;
	struct k_mem_slab *mem_slab;
	uint32_t block_size;
	struct k_msgq rx_queue;
	bool request_clock : 1;
	bool configured    : 1;
	volatile bool active;
	volatile bool stopping;
	DMIC_Type *base_address;
	struct mcux_dmic_pdm_chan *pdm_channels;
};

struct mcux_dmic_cfg {
	const struct pinctrl_dev_config *pcfg;
};

static void free_buffer(struct mcux_dmic_drv_data *drv_data, void *buffer)
{
	k_mem_slab_free(drv_data->mem_slab, &buffer);
	LOG_DBG("Freed buffer %p", buffer);
}

//static void event_handler(const struct device *dev, const mcux_dmic_evt_t *evt)
//{
//	struct mcux_dmic_drv_data *drv_data = dev->data;
//	int ret;
//	bool stop = false;
//
//	if (evt->buffer_requested) {
//		void *buffer;
//		nrfx_err_t err;
//
//		ret = k_mem_slab_alloc(drv_data->mem_slab, &buffer, K_NO_WAIT);
//		if (ret < 0) {
//			LOG_ERR("Failed to allocate buffer: %d", ret);
//			stop = true;
//		} else {
//			/*err = nrfx_pdm_buffer_set(buffer,
//						  drv_data->block_size / 2);
//			if (err != NRFX_SUCCESS) {
//				LOG_ERR("Failed to set buffer: 0x%08x", err);
//				stop = true;
//			}*/
//		}
//	}
//
//	if (drv_data->stopping) {
//		if (evt->buffer_released) {
//			free_buffer(drv_data, evt->buffer_released);
//		}
//
//		if (drv_data->active) {
//			drv_data->active = false;
//			if (drv_data->request_clock) {
//				(void)onoff_release(drv_data->clk_mgr);
//			}
//		}
//	} else if (evt->buffer_released) {
//		ret = k_msgq_put(&drv_data->rx_queue,
//				 &evt->buffer_released,
//				 K_NO_WAIT);
//		if (ret < 0) {
//			LOG_ERR("No room in RX queue");
//			stop = true;
//
//			free_buffer(drv_data, evt->buffer_released);
//		} else {
//			LOG_DBG("Queued buffer %p", evt->buffer_released);
//		}
//	}
//
//	if (stop) {
//		drv_data->stopping = true;
//		//nrfx_pdm_stop();
//	}
//}

//static bool is_better(uint32_t freq,
//		      uint8_t ratio,
//		      uint32_t req_rate,
//		      uint32_t *best_diff,
//		      uint32_t *best_rate,
//		      uint32_t *best_freq)
//{
//	uint32_t act_rate = freq / ratio;
//	uint32_t diff = act_rate >= req_rate ? (act_rate - req_rate)
//					     : (req_rate - act_rate);
//
//	LOG_DBG("Freq %u, ratio %u, act_rate %u", freq, ratio, act_rate);
//
//	if (diff < *best_diff) {
//		*best_diff = diff;
//		*best_rate = act_rate;
//		*best_freq = freq;
//		return true;
//	}
//
//	return false;
//}

/** TODO: change this with device tree configurations **/
static void _init_channels_with_default_cfg(uint8_t num_chan, struct mcux_dmic_pdm_chan *pdm_channels) {

	for(uint8_t i=0;i<num_chan;i++) {

		pdm_channels[i].dmic_channel_cfg.divhfclk            = kDMIC_PdmDiv1;
		pdm_channels[i].dmic_channel_cfg.osr                 = 25U;
		pdm_channels[i].dmic_channel_cfg.gainshft            = 2U;
		pdm_channels[i].dmic_channel_cfg.preac2coef          = kDMIC_CompValueZero;
		pdm_channels[i].dmic_channel_cfg.preac4coef          = kDMIC_CompValueZero;
		pdm_channels[i].dmic_channel_cfg.dc_cut_level        = kDMIC_DcCut155;
		pdm_channels[i].dmic_channel_cfg.post_dc_gain_reduce = 1;
		pdm_channels[i].dmic_channel_cfg.saturate16bit       = 1U;
		pdm_channels[i].dmic_channel_cfg.sample_rate         = kDMIC_PhyFullSpeed;
		#if defined(FSL_FEATURE_DMIC_CHANNEL_HAS_SIGNEXTEND) && (FSL_FEATURE_DMIC_CHANNEL_HAS_SIGNEXTEND)
			pdm_channels[i].dmic_channel_cfg.enableSignExtend = true;
		#endif
	}

	return;
}
/*
 * For now, we will support only one stream, max two channels. Other use cases
 * are unclear at this point
 * */
static int dmic_mcux_configure(const struct device *dev,
				   struct dmic_cfg *config)
{

	struct mcux_dmic_drv_data *drv_data = dev->data;
	struct mcux_dmic_cfg *dmic_cfg = dev->config;
	struct pdm_chan_cfg *channel = &config->channel;
        struct pcm_stream_cfg *stream = &config->streams[0];

	uint32_t map;

	if (drv_data->active) {
                LOG_ERR("Cannot configure device while it is active");
                return -EBUSY;
        }

	/* for now we support only channels #0 and #1 */
	switch(channel->req_num_chan) {
		case 1:
			map = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);
			channel->act_num_chan = 1;
			break;
		case 2:
			map = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT)
				| dmic_build_channel_map(1, 0, PDM_CHAN_RIGHT);
			channel->act_num_chan = 2;
			break;
		default:
			LOG_ERR("Requested number of channels is invalid");
			return -EINVAL;
	}

	channel->act_num_streams = 1;
	channel->act_chan_map_hi = 0;
	channel->act_chan_map_lo = map;

	if (channel->req_num_streams != 1 ||
	    channel->req_chan_map_lo != map) {
		LOG_ERR("Requested number of channels is invalid");
		return -EINVAL;
	}
	
	if (stream->pcm_rate == 0 || stream->pcm_width == 0) {
		if (drv_data->configured) {
			DMIC_DeInit(drv_data->base_address);
			drv_data->configured = false;
		}

		return 0;
	}

	if (stream->pcm_width != 16 || stream->pcm_width != 24) {
		LOG_ERR("Only 16-bit and 24-bit samples are supported");
		return -EINVAL;
	}

	_init_channels_with_default_cfg(channel->act_num_chan, drv_data->pdm_channels);
	return 0;
}
//static int start_transfer(struct dmic_nrfx_pdm_drv_data *drv_data)
//{
//	return 0;
//}

//static void clock_started_callback(struct onoff_manager *mgr,
//				   struct onoff_client *cli,
//				   uint32_t state,
//				   int res)
//{
//	return;
//}

//static int trigger_start(const struct device *dev)
//{
//	return 0;
//}

static int dmic_mcux_trigger(const struct device *dev,
				 enum dmic_trigger cmd)
{
	return 0;
}

static int dmic_mcux_read(const struct device *dev,
			      uint8_t stream,
			      void **buffer, size_t *size, int32_t timeout)
{
	return 0;
}

static void init_clock_manager(const struct device *dev)
{
	return;
}

static void dmic_mcux_isr(const struct device *dev) {

	return;
}

static const struct _dmic_ops dmic_ops = {
	.configure = dmic_mcux_configure,
	.trigger = dmic_mcux_trigger,
	.read = dmic_mcux_read,
};

#define DMIC(idx) DT_NODELABEL(dmic##idx)
#define DMIC_CLK_SRC(idx) DT_STRING_TOKEN(DMIC(idx), clock_source)
#define PDM_DMIC_CHAN_SET_STATUS(pdm_chan_node_id, idx) 						\
	(pdm_channels##idx[DT_NODE_CHILD_IDX(pdm_chan_node_id)]).enabled=true;				\
	(pdm_channels##idx[DT_NODE_CHILD_IDX(pdm_chan_node_id)]).active=false;				\
	(pdm_channels##idx[DT_NODE_CHILD_IDX(pdm_chan_node_id)]).use2fs=				\
		(bool)DT_PROP(pdm_chan_node_id, use2fs);						\
	(pdm_channels##idx[DT_NODE_CHILD_IDX(pdm_chan_node_id)]).hwvad_enabled=				\
		(bool)DT_PROP(pdm_chan_node_id, hwvad);
	//BUILD_ASSERT((DT_NODE_CHILD_IDX(pdm_chan_node_id) != 0) & !(bool)DT_PROP(pdm_chan_node_id, hwvad), "error: hwvad can only be enabled on channel 0");

#define MCUX_DMIC_DEVICE(idx)						     				\
	struct mcux_dmic_pdm_chan pdm_channels##idx[FSL_FEATURE_DMIC_CHANNEL_NUM] ; 			\
	static struct mcux_dmic_drv_data mcux_dmic_data##idx = { 					\
		.pdm_channels = pdm_channels##idx,							\
		.base_address = (DMIC_Type *) DT_REG_ADDR(DMIC(idx)),					\
		.active = false,									\
	};												\
	static struct mcux_dmic_cfg mcux_dmic_cfg##idx;							\
	static int mcux_dmic_init##idx(const struct device *dev)	     				\
	{												\
		DT_FOREACH_CHILD_STATUS_OKAY_VARGS(DT_DRV_INST(idx), PDM_DMIC_CHAN_SET_STATUS, idx)     \
		IRQ_CONNECT(DT_IRQN(DMIC(idx)), DT_IRQ(DMIC(idx), priority),   				\
			    dmic_mcux_isr, DEVICE_DT_INST_GET(idx), 0);					\
		DMIC_Init((mcux_dmic_data##idx).base_address);						\
		return 0;						     				\
	}								     				\
        PINCTRL_DT_DEFINE(DMIC(idx));									\
	DEVICE_DT_DEFINE(DMIC(idx), mcux_dmic_init##idx, NULL,		     				\
			 &mcux_dmic_data##idx, &mcux_dmic_cfg##idx,  					\
			 POST_KERNEL, CONFIG_AUDIO_DMIC_INIT_PRIORITY,	     				\
			 &dmic_ops);



/* Existing SoCs only have one PDM instance. */
DT_INST_FOREACH_STATUS_OKAY(MCUX_DMIC_DEVICE)
