/*
 * Copyright 2023 NXP
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * based on dmic_nrfx_pdm.c
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Assumptions
 *   - the Zephyr API provides a "map" that currently limits a stream to 2 channels, right and left
 *   - the notion of "map" assumes the LEFT and RIGHT channels of EACH PDM controller needs 
 *     to be adjacent to each other
 *   - The current implementation of DMIC for MCUX is limited to two channels and one stream
 *   - Some of the code assumes sequential channel numbers from 0, this will be improved over time
 */

#include <zephyr/drivers/dma.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <soc.h>

#include <fsl_dmic.h>

#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
LOG_MODULE_REGISTER(dmic_mcux, CONFIG_AUDIO_DMIC_LOG_LEVEL);

#define DT_DRV_COMPAT nxp_mcux_dmic

#define CONFIG_DMIC_MCUX_QUEUE_SIZE 8		// Move to KCONFIG

enum e_dmic_state {
     run_ping,
     run_pong,
     stop,
     fail
} ;

struct mcux_dmic_pdm_chan {
     bool enabled;
     bool active;
     bool hwvad_enabled;
     bool use2fs;
     struct device *dev_dma;
     dmic_channel_config_t dmic_channel_cfg;
     struct device *dma;
     uint8_t dma_chan;
     struct dma_config dma_cfg;
     struct dma_block_config dma_block;
};

struct mcux_dmic_drv_data {
	struct k_mem_slab *mem_slab;
	void *ping_block;
	void *pong_block;
	uint32_t block_size;
	uint8_t fifo_size;
	bool request_clock : 1;
	bool configured    : 1;
	volatile bool active;
	volatile bool stopping;
	DMIC_Type *base_address;
	struct mcux_dmic_pdm_chan *pdm_channels;
	int8_t max_chan_num;
	uint8_t act_num_chan;
	enum e_dmic_state dmic_state;
	uint8_t 	pcm_width     ; 
	struct k_msgq rx_queue; 
};

struct mcux_dmic_cfg {
	const struct pinctrl_dev_config *pcfg;
	struct k_msgq rx_buffer;

	bool use2fs;
};

static uint32_t _get_dmic_OSR_divider(uint32_t pcm_rate, bool use2fs) {

	uint32_t use2fs_div = use2fs ? 2 : 1;
	uint32_t osr;
	const uint32_t dmic_clk = 3072000;
	
        osr = (uint32_t)(dmic_clk/(pcm_rate * use2fs_div));

	return osr;
}

static void _init_channels(struct mcux_dmic_drv_data *drv_data, uint8_t num_chan, 
			   struct mcux_dmic_pdm_chan *pdm_channels, uint32_t pcm_rate) {

        dmic_channel_config_t dmic_channel_cfg;
        
	LOG_INF("_init_channels: in");
	for(uint8_t i=0;i<num_chan;i++) {

		LOG_INF("configure: chan %u", i);
	
               pdm_channels[i].dmic_channel_cfg.divhfclk            = kDMIC_PdmDiv1;
               pdm_channels[i].dmic_channel_cfg.osr                 = 16U ; //_get_dmic_OSR_divider(pcm_rate, true);
               pdm_channels[i].dmic_channel_cfg.gainshft            = 6U;                  /* default */
               pdm_channels[i].dmic_channel_cfg.preac2coef          = kDMIC_CompValueZero; /* default */
               pdm_channels[i].dmic_channel_cfg.preac4coef          = kDMIC_CompValueZero; /* default */
               pdm_channels[i].dmic_channel_cfg.dc_cut_level        = kDMIC_DcCut155;      /* default */
               pdm_channels[i].dmic_channel_cfg.post_dc_gain_reduce = 1;                   /* default */
               pdm_channels[i].dmic_channel_cfg.saturate16bit       = 1U;                  /* default */
               pdm_channels[i].dmic_channel_cfg.enableSignExtend    = false;               /* default */
               pdm_channels[i].dmic_channel_cfg.sample_rate         = kDMIC_PhyFullSpeed;  /* default */
		#if defined(FSL_FEATURE_DMIC_CHANNEL_HAS_SIGNEXTEND) && (FSL_FEATURE_DMIC_CHANNEL_HAS_SIGNEXTEND)
					pdm_channels[i].dmic_channel_cfg.enableSignExtend = false;
		#endif
		
		DMIC_ConfigChannel(drv_data->base_address,
		                  (dmic_channel_t)i, 
		                  (stereo_side_t)(i%2), 
		                  &(pdm_channels[i].dmic_channel_cfg));
		                  
		DMIC_FifoChannel(drv_data->base_address, (dmic_channel_t)i, 15, 1, 1);
	}

	return;
}

static int _reload_dmas(struct mcux_dmic_drv_data *drv_data, void* sample_buffer) {

	struct mcux_dmic_pdm_chan *pdm_channels = drv_data->pdm_channels;
	uint8_t num_chan = drv_data->act_num_chan;
	uint32_t dma_buf_size = drv_data->block_size / num_chan; 
	int ret = 0;

	for(uint8_t i=0;i<num_chan;i++) {

		ret = dma_reload(pdm_channels[0].dma, pdm_channels[0].dma_chan,
			         DMIC_FifoGetAddress(drv_data->base_address, (uint32_t)0),
			         sample_buffer, dma_buf_size);
		if(ret < 0) {
			return ret;
		}
	}

	return 0;
}

static void free_block(struct mcux_dmic_drv_data *drv_data, void **buffer)
{
	k_mem_slab_free(drv_data->mem_slab, buffer);
}

static void dmic_mcux_activate_channels(struct mcux_dmic_drv_data *drv_data, bool enable) {

	uint32_t mask = 0x0;
        
	for(uint8_t i=0;i<drv_data->act_num_chan;i++) {
	    mask |= (enable << i);   
	}

	DMIC_EnableChannnel(drv_data->base_address, mask);
}

static void dmic_mcux_dma_cb(const struct device *dev, void *user_data, uint32_t channel, int status){

	struct mcux_dmic_drv_data *drv_data = ((struct device*)user_data)->data;
	struct mcux_dmic_pdm_chan *pdm_channels = drv_data->pdm_channels;
	struct mcux_dmic_cfg *dmic_cfg = ((struct device*)user_data)->config;
	uint8_t num_chan = drv_data->act_num_chan;
	uint32_t dma_buf_size = drv_data->block_size / num_chan;
	int ret;
	
	if(status < 0) {

	     drv_data->dmic_state = fail;
	     free_block(drv_data, drv_data->ping_block);
	     free_block(drv_data, drv_data->pong_block);
	     return;
	}
	
	switch(drv_data->dmic_state) {
	  case run_ping: 
		ret = k_msgq_put(&drv_data->rx_queue,
		                 &drv_data->ping_block,
		                 K_NO_WAIT);
		if(ret < 0) {
			drv_data->dmic_state = fail;
			free_block(drv_data, drv_data->ping_block);
	     		free_block(drv_data, drv_data->pong_block);
	     		return;
		}

		ret = k_mem_slab_alloc(drv_data->mem_slab, &drv_data->ping_block, K_NO_WAIT);
		if(ret < 0) {
			drv_data->dmic_state = fail;
			return;
		}

		ret = _reload_dmas(drv_data, drv_data->pong_block);
		if(ret < 0) {
			drv_data->dmic_state = fail;
			free_block(drv_data, drv_data->ping_block);
	     		free_block(drv_data, drv_data->pong_block);
	     		return;
		}
		drv_data->dmic_state = run_pong;

		break;

	 case run_pong:
		ret = k_msgq_put(&drv_data->rx_queue,
		                 &drv_data->pong_block,
		                 K_NO_WAIT);
		if(ret < 0) {
			drv_data->dmic_state = fail;
			free_block(drv_data, drv_data->ping_block);
	     		free_block(drv_data, drv_data->pong_block);
	     		return;
		}

		ret = k_mem_slab_alloc(drv_data->mem_slab, &drv_data->pong_block, K_NO_WAIT);
		if(ret < 0) {
			drv_data->dmic_state = fail;
			return;
		}

		ret = _reload_dmas(drv_data, drv_data->ping_block);
		if(ret < 0) {
			drv_data->dmic_state = fail;
			free_block(drv_data, drv_data->ping_block);
	     		free_block(drv_data, drv_data->pong_block);
	     		return;
		}
		drv_data->dmic_state = run_ping;

		break;

	  default:
		break;
	}		

	return;
}

static int dmic_mcux_setup_dma(struct device *dev) {

        struct mcux_dmic_drv_data *drv_data = dev->data;
        struct dmic_cfg *dmic_dev_cfg = dev->config;
	struct mcux_dmic_pdm_chan *pdm_channels = drv_data->pdm_channels;
	uint8_t num_chan = drv_data->act_num_chan;
	uint32_t dma_buf_size = drv_data->block_size / num_chan;
	int ret = 0;

	
	for(uint8_t i=0;i<num_chan;i++) {
	    /* DMA configuration , block config follows */
	    
	    pdm_channels[i].dma_cfg.user_data = dev;
	    pdm_channels[i].dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
	    pdm_channels[i].dma_cfg.source_data_size = (drv_data->pcm_width == 16) ?  2U : 4U;
	    pdm_channels[i].dma_cfg.dest_data_size = (drv_data->pcm_width == 16) ?  2U : 4U;
	    //pdm_channels[i].dma_cfg.source_burst_length = 4U;
	    //pdm_channels[i].dma_cfg.dest_burst_length = (drv_data->pcm_width == 16) ?  2U :4U;
	    pdm_channels[i].dma_cfg.dma_callback = dmic_mcux_dma_cb;
	    pdm_channels[i].dma_cfg.error_callback_en = 0;
	    pdm_channels[i].dma_cfg.block_count = 1;
	    pdm_channels[i].dma_cfg.head_block = &(pdm_channels[i].dma_block);
	    
	    if(i == num_chan - 1) {
	        pdm_channels[i].dma_cfg.dma_callback = dmic_mcux_dma_cb;
	        pdm_channels[i].dma_cfg.complete_callback_en = 0;
	        
	    } else {
	        pdm_channels[i].dma_cfg.dma_callback = NULL;
	    }
	    
	    /* DMA block config */
	    pdm_channels[i].dma_block.source_gather_en = false;
	    pdm_channels[i].dma_block.block_size = dma_buf_size;
	    pdm_channels[i].dma_block.source_address = 
		     DMIC_FifoGetAddress(drv_data->base_address, (uint32_t)i);
	    pdm_channels[i].dma_block.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	   
	    // samples will be "desposited" interleaved in the dest buffer. Address increment is 4 bytes for 16 bits, 
	    // 8 for 24/32 bits. Same applies for dest_scatter_interval.
	    pdm_channels[i].dma_block.dest_address = drv_data->ping_block;
            pdm_channels[i].dma_block.dest_scatter_interval = 
		     ((drv_data->pcm_width == 16) ?  (num_chan*2) : (num_chan*4));
	    // we transfer only the number of bytes per PCM sample 
	    pdm_channels[i].dma_block.dest_scatter_count = (drv_data->pcm_width == 16) ?  2U :4U;
	    pdm_channels[i].dma_block.dest_scatter_en = true;
	    pdm_channels[i].dma_block.dest_reload_en = 1;
	    pdm_channels[i].dma_block.next_block = NULL;
	    
	    ret = dma_config(pdm_channels[i].dma, pdm_channels[i].dma_chan, &(pdm_channels[i].dma_cfg));
	    if(ret < 0) {
		return ret;
	    }
	}

	return 0;
}

static int dmic_mcux_start_dma(struct mcux_dmic_drv_data *drv_data) {

	struct mcux_dmic_pdm_chan *pdm_channels = drv_data->pdm_channels;
	uint8_t num_chan = drv_data->act_num_chan;
	int ret;
	
	for(uint8_t i=0;i<num_chan;i++) {
	    ret = dma_start(pdm_channels[i].dma, pdm_channels[i].dma_chan);
	    if(ret) {
	        for(uint8_t j=0;j<i;j++) {
	            if(dma_stop(pdm_channels[j].dma, pdm_channels[j].dma_chan)) {
	                // !!!
	                return -1;
	            }
	        }
	        return ret;
	    }
 	    DMIC_EnableChannelDma(drv_data->base_address, (dmic_channel_t)i, true);
	}

	return 0;
}

static int dmic_mcux_stop_dma(struct mcux_dmic_drv_data *drv_data) {

	struct mcux_dmic_pdm_chan *pdm_channels = drv_data->pdm_channels;
	uint8_t num_chan = drv_data->act_num_chan;
	int ret = 0;

	for(uint8_t i=0;i<num_chan;i++) {
 	    DMIC_EnableChannelDma(drv_data->base_address, (dmic_channel_t)i, false);
	    if(dma_stop(pdm_channels[i].dma, pdm_channels[i].dma_chan) < 0) {
		ret = 1;
	    }
	}

	return ret;
}

/*
 * For now, we will support only one stream, max two channels. Other use cases
 * are unclear at this point
 * */
static int dmic_mcux_configure(const struct device *dev,
				   struct dmic_cfg *config)
{

	struct mcux_dmic_drv_data *drv_data = dev->data;
	struct mcux_dmic_cfg *dmic_dev_cfg = dev->config;
	struct pdm_chan_cfg *channel = &config->channel;
        struct pcm_stream_cfg *stream = &config->streams[0];

	uint32_t map;

	LOG_INF("configure: in");
	if (drv_data->active) {
                LOG_ERR("Cannot configure device while it is active");
                return -EBUSY;
        }

	/* for now we support only channels #0 and #1 */
	switch(channel->req_num_chan) {
		case 1:
			LOG_INF("configure: one channel");
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

	LOG_INF("configure: one channelPCM width = %u", stream->pcm_width);
	if (stream->pcm_width != 16 && stream->pcm_width != 24) {
		LOG_ERR("Only 16-bit and 24-bit samples are supported");
		return -EINVAL;
	}
        drv_data->pcm_width = stream->pcm_width;
	drv_data->mem_slab   = stream->mem_slab;
	drv_data->block_size   = stream->block_size;
	drv_data->act_num_chan = channel->act_num_chan;
	
	DMIC_Use2fs(drv_data->base_address, dmic_dev_cfg->use2fs);
	_init_channels(drv_data, channel->act_num_chan, drv_data->pdm_channels, stream->pcm_rate);
	
	drv_data->configured=true;
	return 0;
}

static int dmic_mcux_stop(const struct device *dev)
{
        struct mcux_dmic_drv_data *drv_data = dev->data;
        struct mcux_dmic_cfg *dmic_cfg = dev->config;

	/* disable FIFO */
	dmic_mcux_stop_dma(drv_data);
 	dmic_mcux_activate_channels(drv_data, false);
	return 0;
}

static int dmic_mcux_start(const struct device *dev)
{
        struct mcux_dmic_drv_data *drv_data = dev->data;
        struct dmic_cfg *dmic_dev_cfg = dev->config;
       	struct pdm_chan_cfg *channel = &dmic_dev_cfg->channel;
        //struct pcm_stream_cfg *stream = &dmic_dev_cfg->streams[0];
	int ret;
	
	LOG_INF("dmic_mcux_start: in");
	
        /* Enable Channel */
        ret = k_mem_slab_alloc(drv_data->mem_slab, &drv_data->ping_block, K_NO_WAIT);
        if(ret < 0) {
             LOG_ERR("start: failed to allocate buffer");
             return -1;
        }
        ret = k_mem_slab_alloc(drv_data->mem_slab, &drv_data->pong_block, K_NO_WAIT);
        if(ret < 0) {
             LOG_ERR("start: failed to allocate buffer");
             return -1;
        }
        
        //(struct mcux_dmic_drv_data *drv_data, struct pdm_chan_cfg *channel)
        ret = dmic_mcux_setup_dma(dev);
	if(ret < 0) { 
	    return ret;
	}
        drv_data->dmic_state = run_ping;
        ret = dmic_mcux_start_dma(drv_data);
	if(ret < 0) {
	    return ret;
	}
	dmic_mcux_activate_channels(drv_data, true);

	return 0;
}

static int dmic_mcux_trigger(const struct device *dev,
				 enum dmic_trigger cmd)
{
	struct mcux_dmic_drv_data *drv_data = dev->data;

	LOG_INF("trigger: in");
	switch (cmd) {
	case DMIC_TRIGGER_PAUSE:
	case DMIC_TRIGGER_STOP:
		LOG_INF("trigger: stop");
		if (drv_data->active) {
			drv_data->stopping = true;
			dmic_mcux_stop(dev);
		}
		break;

	case DMIC_TRIGGER_RELEASE:
	case DMIC_TRIGGER_START:
		LOG_INF("trigger: start");
		if (!drv_data->configured) {
			LOG_ERR("Device is not configured");
			return -EIO;
		} else if (!drv_data->active) {
			drv_data->stopping = false;
			return dmic_mcux_start(dev);
		}
		break;

	default:
		LOG_ERR("Invalid command: %d", cmd);
		return -EINVAL;
	}
	return 0;
}

static int dmic_mcux_read(const struct device *dev,
			      uint8_t stream,
			      void **buffer, size_t *size, int32_t timeout)
{
	struct mcux_dmic_drv_data *drv_data = dev->data;
        int ret;
        	
        ARG_UNUSED(stream);

	if (!drv_data->configured) {
		LOG_ERR("Device is not configured");
		return -EIO;
	}
        	
	ret = k_msgq_get(&drv_data->rx_queue, buffer, SYS_TIMEOUT_MS(timeout));
	if(ret < 0) {
	    return ret;
	} else {
            *size = drv_data->block_size;
	}
	
	return 0;
}

static const struct _dmic_ops dmic_ops = {
	.configure = dmic_mcux_configure,
	.trigger = dmic_mcux_trigger,
	.read = dmic_mcux_read,
};

#define DMIC(idx) DT_NODELABEL(dmic##idx)
#define PDMC(idx) DT_NODELABEL(pdmc##idx)
#define DMIC_CLK_SRC(idx) DT_STRING_TOKEN(DMIC(idx), clock_source)
#define PDM_DMIC_CHAN_SET_STATUS(pdm_chan_node_id, idx) 						\
	(pdm_channels##idx[DT_NODE_CHILD_IDX(pdm_chan_node_id)]).enabled=true;				\
	(pdm_channels##idx[DT_NODE_CHILD_IDX(pdm_chan_node_id)]).active=false;				\
	(pdm_channels##idx[DT_NODE_CHILD_IDX(pdm_chan_node_id)]).hwvad_enabled=				\
		(bool)DT_PROP(pdm_chan_node_id, hwvad);							\
	(pdm_channels##idx[DT_NODE_CHILD_IDX(pdm_chan_node_id)]).dma=				DEVICE_DT_GET(DT_DMAS_CTLR(pdm_chan_node_id));								\
	(pdm_channels##idx[DT_NODE_CHILD_IDX(pdm_chan_node_id)]).dma_chan=				DT_DMAS_CELL_BY_IDX(pdm_chan_node_id, 0, channel);						
	
	//BUILD_ASSERT((DT_NODE_CHILD_IDX(pdm_chan_node_id) != 0) & !(bool)DT_PROP(pdm_chan_node_id, hwvad), "error: hwvad can only be enabled on channel 0");

#if !(defined(FSL_FEATURE_DMIC_HAS_NO_IOCFG) && FSL_FEATURE_DMIC_HAS_NO_IOCFG)
	#define SET_IO DMIC_SetIOCFG((mcux_dmic_data##idx).base_address, kDMIC_PdmDual)
#else
	#define SET_IO while(0){}
#endif

#define MCUX_DMIC_DEVICE(idx)	 									\
       	struct mcux_dmic_pdm_chan pdm_channels##idx[FSL_FEATURE_DMIC_CHANNEL_NUM] ; 			\
	static uint32_t *rx_msgs##idx[CONFIG_DMIC_MCUX_QUEUE_SIZE];            			\
	static struct mcux_dmic_drv_data mcux_dmic_data##idx = { 					\
		.pdm_channels = pdm_channels##idx,							\
		.base_address = (DMIC_Type *) DT_REG_ADDR(DMIC(idx)),					\
		.active = false,									\
		.fifo_size = 	DT_PROP(DMIC(idx), fifo_size),						\
		.dmic_state = stop,									\
	};												\
	PINCTRL_DT_DEFINE(DMIC(idx));									\
	static struct mcux_dmic_cfg mcux_dmic_cfg##idx = {						\
		.pcfg = PINCTRL_DT_DEV_CONFIG_GET(DMIC(idx)),						\
	};												\
	static int mcux_dmic_init##idx(const struct device *dev)	     				\
	{												\
		DT_FOREACH_CHILD_STATUS_OKAY_VARGS(DT_DRV_INST(idx), PDM_DMIC_CHAN_SET_STATUS, idx)     \
		k_msgq_init(&mcux_dmic_data##idx.rx_queue,               				\
                            (char *)rx_msgs##idx, sizeof(uint32_t *),            			\
                            ARRAY_SIZE(rx_msgs##idx));							\
		irq_enable(DT_IRQN(DMIC(idx)));								\
	        pinctrl_apply_state(mcux_dmic_cfg##idx.pcfg, PINCTRL_STATE_DEFAULT);	    		\
		DMIC_Init((mcux_dmic_data##idx).base_address);						\
		SET_IO;  										\
													\
		return 0;						     				\
	}								     				\
        												\
	DEVICE_DT_DEFINE(DMIC(idx), mcux_dmic_init##idx, NULL,		     				\
			 &mcux_dmic_data##idx, &mcux_dmic_cfg##idx,  					\
			 POST_KERNEL, CONFIG_AUDIO_DMIC_INIT_PRIORITY,	     				\
			 &dmic_ops);



/* Existing SoCs only have one PDM instance. */
DT_INST_FOREACH_STATUS_OKAY(MCUX_DMIC_DEVICE)
