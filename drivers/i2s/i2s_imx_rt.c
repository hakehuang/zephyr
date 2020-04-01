/*
 * Copyright (c) 2020 NXP Semiconductor INC.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief I2S bus (SAI) driver for NXP i.MX RT series.
 *
 */

#include <errno.h>
#include <string.h>
#include <sys/__assert.h>
#include <kernel.h>
#include <device.h>
#include <init.h>
#include <drivers/dma.h>
#include <drivers/i2s.h>
#include <drivers/pinmux.h>
#include <drivers/clock_control.h>
#include <soc.h>
#include "i2s_imx_rt.h"

#define LOG_DOMAIN dev_i2s_rt
#define LOG_LEVEL CONFIG_I2S_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_DOMAIN);

#define DEFAULT_I2S_DMA_BURST_SIZE kSAI_WordWidth16bits

#define I2S_IRQ_CONNECT(i2s_id)                                                \
	IRQ_CONNECT(DT_INST_##i2s_id##_NXP_RT_I2S_IRQ_0,                       \
		    DT_INST_##i2s_id##_NXP_RT_I2S_IRQ_0_PRIORITY, i2s_rt_isr,  \
		    DEVICE_GET(i2s##i2s_id##_rt), 0)

#define I2S_DEVICE_NAME(i2s_id) i2s##i2s_id##_rt
#define I2S_DEVICE_DATA_NAME(i2s_id) i2s##i2s_id##_data
#define I2S_DEVICE_CONFIG_NAME(i2s_id) i2s##i2s_id##_config

#define I2S_DEVICE_CONFIG_DEFINE(i2s_id)                                        \
	static const struct i2s_config i2s##i2s_id##_config = {                 \
		.base = (I2S_Type *)DT_INST_##i2s_id##_NXP_RT_I2S_BASE_ADDRESS, \
		.edma_name = CONFIG_I2S_##i2s_id##_DMA_NAME,                    \
		.pinmux_name =                                                  \
			DT_INST_##i2s_id##_NXP_RT_I2S_PINMUXS_CONTROLLER,       \
		.i2s_id = i2s_id,                                               \
		.irq_id = DT_INST_##i2s_id##_NXP_RT_I2S_IRQ_0,                  \
	}

#define I2S_DEVICE_OBJECT_DECLARE(i2s_id)                                      \
	DEVICE_DECLARE(I2S_DEVICE_NAME(i2s_id))

#define I2S_DEVICE_OBJECT(i2s_id) DEVICE_GET(I2S_DEVICE_NAME(i2s_id))

#define I2S_DEVICE_DATA_DEFINE(i2s_id)                                         \
	static struct i2s_dev_data i2s##i2s_id##_data = {\
		.tx = {						\
			.dma_channel = DT_INST_##i2s_id##_NXP_RT_I2S_DMAS_SOURCE_0,	\
			.dma_cfg = {				\
				.source_burst_length = DEFAULT_I2S_DMA_BURST_SIZE,\
				.dest_burst_length = DEFAULT_I2S_DMA_BURST_SIZE,\
				.dma_callback = i2s_dma_tx_callback,	\
				.callback_arg = I2S_DEVICE_OBJECT(i2s_id),\
				.complete_callback_en = 1,	\
				.error_callback_en = 1,		\
				.block_count = 1,		\
				.head_block =			\
				&i2s##i2s_id##_data.tx.dma_block,\
				.channel_direction = MEMORY_TO_PERIPHERAL,\
				.dma_slot = CONFIG_I2S_##i2s_id##_TX_DMA_SLOT,\
			},					\
		},						\
		.rx = {						\
			.dma_channel = DT_INST_##i2s_id##_NXP_RT_I2S_DMAS_SOURCE_1,	\
			.dma_cfg = {				\
				.source_burst_length = DEFAULT_I2S_DMA_BURST_SIZE,\
				.dest_burst_length = DEFAULT_I2S_DMA_BURST_SIZE,\
				.dma_callback = i2s_dma_rx_callback,\
				.callback_arg = I2S_DEVICE_OBJECT(i2s_id),\
				.complete_callback_en = 1,	\
				.error_callback_en = 1,		\
				.block_count = 1,		\
				.head_block =			\
				&i2s##i2s_id##_rt_data.rx.dma_block,\
				.channel_direction = PERIPHERAL_TO_MEMORY,\
				.dma_slot = CONFIG_I2S_##i2s_id##_RX_DMA_SLOT,\
				},				\
		},						\
	}

#define I2S_DEVICE_AND_API_INIT(i2s_id)                                        \
	DEVICE_AND_API_INIT(I2S_DEVICE_NAME(i2s_id),                           \
			    DT_INST_##i2s_id##_NXP_RT_I2S_LABEL,               \
			    i2s_rt_initialize, &I2S_DEVICE_DATA_NAME(i2s_id),  \
			    &I2S_DEVICE_CONFIG_NAME(i2s_id), POST_KERNEL,      \
			    CONFIG_I2S_INIT_PRIORITY, &i2s_rt_driver_api)

/*
 * This indicates the Tx/Rx stream. Most members of the stream are
 * self-explanatory
 *
 * in_queue and out_queue are used as follows
 *   transmit stream:
 *      application provided buffer is queued to in_queue until loaded to DMA.
 *      when DMA channel is idle, buffer is retrieved from in_queue and loaded
 *      to DMA and queued to out_queue.
 *      when DMA completes, buffer is retrieved from out_queue and freed.
 *
 *   receive stream:
 *      driver allocates buffer from slab and loads DMA
 *      buffer is queued to in_queue
 *      when DMA completes, buffer is retrieved from in_queue and queued to
 *      out_queue
 *	when application reads, buffer is read (may optionally block) from
 *      out_queue and presented to application.
 */
struct stream {
	s32_t state;
	u32_t dma_channel;
	u32_t start_channel;
	sai_handle_t handle;
	void (*irq_call_back)(void);
	struct dma_config dma_cfg;
	struct dma_block_config dma_block;
	struct k_msgq in_queue;
	void *in_msgs[CONFIG_I2S_BUFFER_QUEUE_COUNT];
	struct k_msgq out_queue;
	void *out_msgs[CONFIG_I2S_BUFFER_QUEUE_COUNT];
};

struct i2s_rt_config {
	I2S_Type *base;
	u32_t irq_id;
	char *edma_name;
	char *pinmux_name;
	u32_t i2s_id;
	void (*irq_connect)(void);
};

/* Device run time data */
struct i2s_dev_data {
	struct i2s_config cfg;
	struct device *dev_dma;
	struct stream tx;
	struct stream rx;
};

#define DEV_NAME(dev) ((dev)->config->name)
#define DEV_CFG(dev)                                                           \
	((const struct i2s_rt_config *const)(dev)->config->config_info)
#define DEV_DATA(dev) ((struct i2s_dev_data *const)(dev)->driver_data)
#define DEV_BASE(dev) ((I2S_Type *)DEV_CFG(dev)->base)

struct pinmux_imx_config;

#define DEV_PINMUX_CFG(dev)                                                    \
	((const struct pinmux_imx_config *const)(dev)->config->config_info)
#define DEV_PINMUX_BASE(dev) ((void *)DEV_CFG(dev)->base)

I2S_DEVICE_OBJECT_DECLARE(1);
I2S_DEVICE_OBJECT_DECLARE(2);
I2S_DEVICE_OBJECT_DECLARE(3);

static void i2s_dma_tx_callback(void *, u32_t, int);
static void i2s_tx_stream_disable(struct device *);
static void i2s_rx_stream_disable(struct device *);

static inline void i2s_purge_stream_buffers(struct stream *strm,
					    struct k_mem_slab *mem_slab)
{
	void *buffer;

	while (k_msgq_get(&strm->in_queue, &buffer, K_NO_WAIT) == 0) {
		k_mem_slab_free(mem_slab, &buffer);
	}
	while (k_msgq_get(&strm->out_queue, &buffer, K_NO_WAIT) == 0) {
		k_mem_slab_free(mem_slab, &buffer);
	}
}

static void i2s_tx_stream_disable(struct device *dev)
{
	struct stream *strm = &DEV_DATA(dev)->tx;

	LOG_INF("Stopping DMA channel %u for TX stream", strm->dma_channel);
	dma_stop(dev_dma, strm->dma_channel);

	/* purge buffers queued in the stream */
	i2s_purge_stream_buffers(strm, dev_data->cfg.mem_slab);
}

static void i2s_rx_stream_disable(struct device *dev)
{
	struct stream *strm = &DEV_DATA(dev)->rx;

	LOG_INF("Stopping RX stream & DMA channel %u", strm->dma_channel);
	dma_stop(dev_dma, strm->dma_channel);

	/* purge buffers queued in the stream */
	i2s_purge_stream_buffers(strm, dev_data->cfg.mem_slab);
}

/* This function is executed in the interrupt context */
static void i2s_dma_tx_callback(void *arg, u32_t channel, int status)
{
	struct device *dev = (struct device *)arg;
	const struct i2s_rt_config *const dev_cfg = DEV_CFG(dev);
	struct i2s_dev_data *const dev_data = DEV_DATA(dev);

	struct stream *strm = &dev_data->tx;
	void *buffer;
	int ret;

	ret = k_msgq_get(&strm->out_queue, &buffer, K_NO_WAIT);
	if (ret == 0) {
		/* transmission complete. free the buffer */
		k_mem_slab_free(dev_data->cfg.mem_slab, &buffer);
	} else {
		LOG_ERR("no buffer in output queue for channel %u", channel);
	}

	switch (strm->state) {
	case I2S_STATE_RUNNING:
		/* get the next buffer from queue */
		ret = k_msgq_get(&strm->in_queue, &buffer, K_NO_WAIT);
		if (ret == 0) {
			/* reload the DMA */
			u32_t data_path = DEV_DATA(dev)->tx.start_channel;
			dma_reload(dev_data->dev_dma, strm->dma_channel,
				   (u32_t)buffer,
				   (u32_t)&DEV_BASE(dev)->TDR[data_path],
				   dev_data->cfg.block_size);
			dma_start(dev_data->dev_dma, strm->dma_channel);
			k_msgq_put(&strm->out_queue, &buffer, K_NO_WAIT);
		}

		if (ret || status) {
			/*
			 * DMA encountered an error (status != 0)
			 * or
			 * No bufers in input queue
			 */
			LOG_ERR("DMA status %08x channel %u k_msgq_get ret %d",
				status, channel, ret);
			strm->state = I2S_STATE_STOPPING;
			i2s_tx_stream_disable(dev);
		}

		break;

	case I2S_STATE_STOPPING:
		i2s_tx_stream_disable(dev_data);
		break;
	}
}

static void i2s_dma_rx_callback(void *arg, u32_t channel, int status)
{
	struct device *dev = (struct device *)arg;
	const struct i2s_rt_config *const dev_cfg = DEV_CFG(dev);
	struct i2s_dev_data *const dev_data = DEV_DATA(dev);
	struct stream *strm = &dev_data->rx;
	void *buffer;
	int ret;

	switch (strm->state) {
	case I2S_STATE_RUNNING:
		/* retrieve buffer from input queue */
		ret = k_msgq_get(&strm->in_queue, &buffer, K_NO_WAIT);
		if (ret != 0) {
			LOG_ERR("get buffer from in_queue %p failed (%d)",
				&strm->in_queue, ret);
		}
		/* put buffer to output queue */
		ret = k_msgq_put(&strm->out_queue, &buffer, K_NO_WAIT);
		if (ret != 0) {
			LOG_ERR("buffer %p -> out_queue %p err %d", buffer,
				&strm->out_queue, ret);
		}
		/* allocate new buffer for next audio frame */
		ret = k_mem_slab_alloc(dev_data->cfg.mem_slab, &buffer,
				       K_NO_WAIT);
		if (ret != 0) {
			LOG_ERR("buffer alloc from slab %p err %d",
				dev_data->cfg.mem_slab, ret);
			i2s_rx_stream_disable(dev_data);
			strm->state = I2S_STATE_READY;
		} else {
			u32_t data_path = dev_data->rx.start_channel;
			/* put buffer in input queue */
			ret = k_msgq_put(&strm->in_queue, &buffer, K_NO_WAIT);
			if (ret != 0) {
				LOG_ERR("buffer %p -> in_queue %p err %d",
					buffer, &strm->in_queue, ret);
			}

			/* reload the DMA */
			dma_reload(dev_data->dev_dma, strm->dma_channel,
				   (u32_t)&DEV_BASE(dev)->RDR[data_path],
				   (u32_t)buffer, dev_data->cfg.block_size);
			dma_start(dev_data->dev_dma, strm->dma_channel);
		}
		break;
	case I2S_STATE_STOPPING:
		i2s_rx_stream_disable(dev_data);
		strm->state = I2S_STATE_READY;
		break;
	}
}

static void _enable_mclk_direction(struct device *dev, bool dir)
{
	const struct device *iomuxgpr_dev =
		device_get_binding(DEV_CFG(dev)->pinmux_name);
	u32_t offset = 0;
	u32_t mask = 0;
	/* enable MCLK output */
	switch (DEV_CFG(dev)->i2s_id) {
	case 0:
		offset = DT_INST_0_NXP_RT_I2S_PINMUXS_PIN;
		mask = DT_INST_0_NXP_RT_I2S_PINMUXS_FUNCTION;
		break;
	case 1:
		offset = DT_INST_1_NXP_RT_I2S_PINMUXS_PIN;
		mask = DT_INST_1_NXP_RT_I2S_PINMUXS_FUNCTION;
		break;
	case 2:
		offset = DT_INST_2_NXP_RT_I2S_PINMUXS_PIN;
		mask = DT_INST_2_NXP_RT_I2S_PINMUXS_FUNCTION;
		break;
	default:
		LOG_ERR("i2s bus is out of scope");
		return;
	}

	if (dir) {
		u32_t value = 0;
		pinmux_pin_get(iomuxgpr_dev, offset, &value);
		value |= mask;
		pinmux_pin_set(iomuxgpr_dev, offset, &value);
	} else {
		u32_t value = 0;
		pinmux_pin_get(iomuxgpr_dev, offset, &value);
		value &= ~mask;
		pinmux_pin_set(iomuxgpr_dev, offset, &value);
	}
}

static void _get_mclk_rate(struct device *dev, u32_t *mclk)
{
	const struct i2s_rt_config *const dev_cfg = DEV_CFG(dev);
	struct device *ccm_dev;
	clock_control_subsys_t sub_sys;
	u32_t rate = 0, pre_div, src_div;
	switch (dev_cfg->i2s_id) {
	case 0:
		ccm_dev = device_get_binding(
			DT_INST_0_NXP_RT_I2S_CLOCK_CONTROLLER);
		clk_sub_sys = (clock_control_subsys_t)IMX_CCM_SAI1_CLK;
		src_div = DT_INST_0_NXP_RT_I2S_CLOCK_BITS_2;
		pre_div = DT_INST_0_NXP_RT_I2S_CLOCK_BITS_1;
		break;
	case 1:
		ccm_dev = device_get_binding(
			DT_INST_1_NXP_RT_I2S_CLOCK_CONTROLLER);
		clk_sub_sys = (clock_control_subsys_t)IMX_CCM_SAI2_CLK;
		src_div = DT_INST_1_NXP_RT_I2S_CLOCK_BITS_2;
		pre_div = DT_INST_1_NXP_RT_I2S_CLOCK_BITS_1;
		break;
	case 2:
		ccm_dev = device_get_binding(
			DT_INST_2_NXP_RT_I2S_CLOCK_CONTROLLER);
		clk_sub_sys = (clock_control_subsys_t)IMX_CCM_SAI3_CLK;
		src_div = DT_INST_2_NXP_RT_I2S_CLOCK_BITS_2;
		pre_div = DT_INST_2_NXP_RT_I2S_CLOCK_BITS_1;
		break;
	default:
		LOG_ERR("not a valide i2s instance");
		break;
	}
	clock_control_get_rate(ccm_dev, clk_sub_sys, &rate);
	*mclk = rate / (src_div + 1) / (pre_div + 1);
}

static int i2s_rt_configure(struct device *dev, enum i2s_dir dir,
			    struct i2s_config *i2s_cfg)
{
	const struct i2s_rt_config *const dev_cfg = DEV_CFG(dev);
	struct i2s_dev_data *const dev_data = DEV_DATA(dev);
	struct dma_block_config *dma_block;
	sai_transceiver_t config;
	/*num_words is frame size*/
	u8_t num_words = i2s_cfg->channels;
	u8_t channels = i2s_cfg->channels;
	u8_t word_size_bits = i2s_cfg->word_size;
	u8_t word_size_bytes;
	u32_t bit_clk_freq, mclk;
	int ret;

	if ((dev_data->tx.state != I2S_STATE_NOT_READY) &&
	    (dev_data->tx.state != I2S_STATE_READY) &&
	    (dev_data->rx.state != I2S_STATE_NOT_READY) &&
	    (dev_data->rx.state != I2S_STATE_READY)) {
		LOG_ERR("invalid state tx(%u) rx(%u)", dev_data->tx.state,
			dev_data->rx.state);
		return -EINVAL;
	}

	if (i2s_cfg->frame_clk_freq == 0U) {
		LOG_ERR("Invalid frame_clk_freq %u", i2s_cfg->frame_clk_freq);
		return -EINVAL;
	}

	if (word_size_bits < SAI_WORD_SIZE_BITS_MIN ||
	    word_size_bits > SAI_WORD_SIZE_BITS_MAX) {
		LOG_ERR("Unsupported I2S word size %u", word_size_bits);
		return -EINVAL;
	}

	if (num_words < SAI_WORD_PER_FRAME_MIN ||
	    num_words > SAI_WORD_PER_FRAME_MAX) {
		LOG_ERR("Unsupported words per frame number %u", num_words);
		return -EINVAL;
	}

	if ((i2s_cfg->options & I2S_OPT_PINGPONG) == I2S_OPT_PINGPONG) {
		LOG_ERR("Ping-pong mode not supported");
		return -ENOTSUP;
	}

	memcpy(&dev_data->cfg, i2s_cfg, sizeof(struct i2s_config));

	memset(config, 0, sizeof(config));

	if (i2s_cfg->options & I2S_OPT_FRAME_CLK_SLAVE) {
		if (i2s_cfg->options & I2S_OPT_BIT_CLK_SLAVE) {
			config.masterSlave = kSAI_Slave;
		} else {
			config.masterSlave = kSAI_Bclk_Master_FrameSync_Slave;
		}
	} else {
		if (i2s_cfg->options & I2S_OPT_BIT_CLK_SLAVE) {
			config.masterSlave = kSAI_Bclk_Slave_FrameSync_Master;
		} else {
			config.masterSlave = kSAI_Master;
		}
	}

	_enable_mclk_direction(dev, i2s_cfg->options & I2S_OPT_MASTER_CLK_OUT);
	_get_mclk_rate(dev, &mclk);

	/* sync mode configurations */
	if (I2S_OPT_SYNC_MODE == i2s_cfg->options) {
		config.syncMode = kSAI_ModeSync;
	} else {
		config.syncMode = kSAI_ModeAsync;
	}

	config.frameSync.frameSyncPolarity = kSAI_PolarityActiveLow;
	config.bitClock.bclkSrcSwap = false;
	/* clock signal polarity */
	switch (i2s_cfg->format & I2S_FMT_CLK_FORMAT_MASK) {
	case I2S_FMT_CLK_NF_NB:
		config.frameSync.frameSyncPolarity = kSAI_PolarityActiveLow;
		config.bitClock.bclkSrcSwap = false;
		break;

	case I2S_FMT_CLK_NF_IB:
		config.frameSync.frameSyncPolarity = kSAI_PolarityActiveLow;
		config.bitClock.bclkSrcSwap = true;
		break;

	case I2S_FMT_CLK_IF_NB:
		config.frameSync.frameSyncPolarity = kSAI_PolarityActiveHigh;
		config.bitClock.bclkSrcSwap = false;
		break;

	case I2S_FMT_CLK_IF_IB:
		config.frameSync.frameSyncPolarity = kSAI_PolarityActiveHigh;
		config.bitClock.bclkSrcSwap = true;
		break;
	}
	/* bit clock source is MCLK */
	config.bitClock.bclkSource = kSAI_BclkSourceMclkDiv;
	/* additional settings for bclk read the SDK header for more details*/
	config.bitClock.bclkInputDelay = false;

	/* frame sync default configurations */
#if defined(FSL_FEATURE_SAI_HAS_FRAME_SYNC_ON_DEMAND) &&                       \
	FSL_FEATURE_SAI_HAS_FRAME_SYNC_ON_DEMAND
	config.frameSync.frameSyncGenerateOnDemand = false;
#endif

	config.frameSync.frameSyncWidth = (uint8_t)word_size_bits;

	/* serial data default configurations */
#if defined(FSL_FEATURE_SAI_HAS_CHANNEL_MODE) &&                               \
	FSL_FEATURE_SAI_HAS_CHANNEL_MODE
	config.serialData.dataMode = kSAI_DataPinStateOutputZero;
#endif

	/* format */
	switch (i2s_cfg->format & I2S_FMT_DATA_FORMAT_MASK) {
	case I2S_FMT_DATA_FORMAT_I2S:
		SAI_GetClassicI2SConfig(&config, word_size_bits, kSAI_Stereo,
					kSAI_Channel0Mask);
		break;
	case I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED:
		SAI_GetLeftJustifiedConfig(&config, word_size_bits, kSAI_Stereo,
					   kSAI_Channel0Mask);
		break;
	case I2S_FMT_DATA_FORMAT_PCM_SHORT:
		SAI_GetDSPConfig(&config, kSAI_FrameSyncLenOneBitClk,
				 word_size_bits, kSAI_Stereo,
				 kSAI_Channel0Mask);
		break;
	case I2S_FMT_DATA_FORMAT_PCM_LONG:
		SAI_GetTDMConfig(&config, kSAI_FrameSyncLenPerWordWidth,
				 kSAI_FrameSyncLenOneBitClk, word_size_bits,
				 kSAI_Stereo, kSAI_Channel0Mask);
		break;
	default:
		LOG_ERR("Unsupported I2S data format");
		return -EINVAL;
	}

	if (dir == I2S_DIR_TX) {
		SAI_TransferTxSetConfig(DEV_BASE(dev),
					&(DEV_DATA(dev)->tx.handle), &config);
		/* set bit clock divider */
		SAI_TxSetBitClockRate(DEV_BASE(dev), mclk, frame_clk_freq,
				      word_size, i2s_cfg->channels);
	} else {
		SAI_TransferRxSetConfig(DEV_BASE(dev),
					&(DEV_DATA(dev)->rx.handle), &config);
		/* set bit clock divider */
		SAI_RxSetBitClockRate(DEV_BASE(dev), mclk, frame_clk_freq,
				      word_size, i2s_cfg->channels);
	}

	/* enable interrupt */
	irq_enable(dev_cfg->irq_id);

	dev_data->tx.state = I2S_STATE_READY;
	dev_data->rx.state = I2S_STATE_READY;

	return 0;
}

static int i2s_tx_stream_start(struct device *dev)
{
	int ret = 0;
	void *buffer;
	unsigned int key;
	struct stream *strm = &DEV_DATA(dev)->tx;
	u32_t data_path = strm->start_channel;
	struct device *dev_dma = DEV_DATA(dev)->dev_dma;

	/* retrieve buffer from input queue */
	ret = k_msgq_get(&strm->in_queue, &buffer, K_NO_WAIT);
	if (ret != 0) {
		LOG_ERR("No buffer in input queue to start transmission");
		return ret;
	}

	ret = dma_reload(dev_dma, strm->dma_channel, (u32_t)buffer,
			 (u32_t)&DEV_BASE(dev)->TDR[data_path],
			 dev_data->cfg.block_size);
	if (ret != 0) {
		LOG_ERR("dma_reload failed (%d)", ret);
		return ret;
	}

	/* put buffer in output queue */
	ret = k_msgq_put(&strm->out_queue, &buffer, K_NO_WAIT);
	if (ret != 0) {
		LOG_ERR("failed to put buffer in output queue");
		return ret;
	}

	ret = dma_start(dev_dma, strm->dma_channel);

	if (ret < 0) {
		LOG_ERR("dma_start failed (%d)", ret);
		return ret;
	}

	return 0;
}

static int i2s_rx_stream_start(struct device *dev)
{
	int ret = 0;
	void *buffer;
	unsigned int key;
	u32_t data_path = strm->start_channel;
	struct stream *strm = &DEV_DATA(dev)->rx;
	struct device *dev_dma = DEV_DATA(dev)->dev_dma;

	/* allocate receive buffer from SLAB */
	ret = k_mem_slab_alloc(dev_data->cfg.mem_slab, &buffer, K_NO_WAIT);
	if (ret != 0) {
		LOG_ERR("buffer alloc from mem_slab failed (%d)", ret);
		return ret;
	}

	ret = dma_reload(dev_dma, strm->dma_channel,
			 (u32_t)&DEV_BASE(dev)->RDR[data_path], (u32_t)buffer,
			 dev_data->cfg.block_size);
	if (ret != 0) {
		LOG_ERR("dma_reload failed (%d)", ret);
		return ret;
	}

	/* put buffer in input queue */
	ret = k_msgq_put(&strm->in_queue, &buffer, K_NO_WAIT);
	if (ret != 0) {
		LOG_ERR("failed to put buffer in output queue");
		return ret;
	}

	LOG_INF("Starting DMA Ch%u", strm->dma_channel);
	ret = dma_start(dev_dma, strm->dma_channel);
	if (ret < 0) {
		LOG_ERR("Failed to start DMA Ch%d (%d)", strm->dma_channel,
			ret);
		return ret;
	}

	return 0;
}

static int i2s_rt_trigger(struct device *dev, enum i2s_dir dir,
			  enum i2s_trigger_cmd cmd)
{
	const struct i2s_rt_config *const dev_cfg = DEV_CFG(dev);
	struct i2s_dev_data *const dev_data = DEV_DATA(dev);
	struct stream *strm;
	unsigned int key;
	int ret = 0;

	strm = (dir == I2S_DIR_TX) ? &dev_data->tx : &dev_data->rx;

	key = irq_lock();
	switch (cmd) {
	case I2S_TRIGGER_START:
		if (strm->state != I2S_STATE_READY) {
			LOG_ERR("START trigger: invalid state %u", strm->state);
			ret = -EIO;
			break;
		}

		__ASSERT_NO_MSG(strm->mem_block == NULL);

		if (dir == I2S_DIR_TX) {
			ret = i2s_tx_stream_start(dev);
		} else {
			ret = i2s_rx_stream_start(dev);
		}

		if (ret < 0) {
			LOG_DBG("START trigger failed %d", ret);
			break;
		}

		strm->state = I2S_STATE_RUNNING;
		break;

	case I2S_TRIGGER_STOP:
	case I2S_TRIGGER_DRAIN:
	case I2S_TRIGGER_DROP:
		if (strm->state != I2S_STATE_RUNNING) {
			LOG_DBG("STOP/DRAIN/DROP trigger: invalid state");
			ret = -EIO;
			break;
		}
		strm->state = I2S_STATE_STOPPING;
		break;

	case I2S_TRIGGER_PREPARE:
		break;

	default:
		LOG_ERR("Unsupported trigger command");
		ret = -EINVAL;
	}

	irq_unlock(key);
	return ret;
}

static int i2s_rt_read(struct device *dev, void **mem_block, size_t *size)
{
	struct i2s_dev_data *const dev_data = DEV_DATA(dev);
	struct stream *strm = &dev_data->rx;
	void *buffer;
	int ret = 0;

	if (strm->state == I2S_STATE_NOT_READY) {
		LOG_ERR("invalid state %d", strm->state);
		return -EIO;
	}

	ret = k_msgq_get(&strm->out_queue, &buffer, dev_data->cfg.timeout);
	if (ret != 0) {
		return -EAGAIN;
	}

	*mem_block = buffer;
	*size = dev_data->cfg.block_size;
	return 0;
}

static int i2s_rt_write(struct device *dev, void *mem_block, size_t size)
{
	struct i2s_dev_data *const dev_data = DEV_DATA(dev);
	struct stream *strm = &dev_data->tx;
	int ret;

	if (strm->state != I2S_STATE_RUNNING &&
	    strm->state != I2S_STATE_READY) {
		LOG_ERR("invalid state (%d)", strm->state);
		return -EIO;
	}

	ret = k_msgq_put(&strm->in_queue, &mem_block, dev_data->cfg.timeout);
	if (ret) {
		LOG_ERR("k_msgq_put failed %d", ret);
		return ret;
	}

	return ret;
}

/* clear IRQ sources atm */
static void i2s_rt_isr(void *arg)
{
	struct device *dev = (struct device *)arg;
	const struct i2s_rt_config *const dev_cfg = DEV_CFG(dev);
	switch (dev_cfg->irq_id) {
#if defined(I2S0)
	case DT_INST_0_NXP_RT_I2S_IRQ_0:
		I2S0_DriverIRQHandler();
		break;
#endif
#if defined(I2S1)
	case DT_INST_1_NXP_RT_I2S_IRQ_0:
		I2S1_DriverIRQHandler();
		break;
#endif
#if defined(I2S2)
	case DT_INST_2_NXP_RT_I2S_IRQ_0:
		I2S2_DriverIRQHandler() break;
#endif
	default:
		LOG_ERR("irq not support");
		break;
	}
}

static void _callback(I2S_Type *base, sai_handle_t *handle, status_t status,
		      void *userData)
{
	LOG_DBG("sai user call back occurred");
}

static void _audio_clock_settings(struct device *dev)
{
	u32_t lp, pd, num, den, src, clK_src, pre_div, src_div;
	clock_audio_pll_config_t audioPllConfig;

	if (DEV_CFG(dev)->i2s_id == 0) {
		src = DT_INST_0_NXP_RT_I2S_PLL_CLOCKS_VALUE_0;
		lp  = DT_INST_0_NXP_RT_I2S_PLL_CLOCKS_VALUE_1;
		pd  = DT_INST_0_NXP_RT_I2S_PLL_CLOCKS_VALUE_2;
		num = DT_INST_0_NXP_RT_I2S_PLL_CLOCKS_VALUE_3;
		den = DT_INST_0_NXP_RT_I2S_PLL_CLOCKS_VALUE_4;
		clK_src = DT_INST_0_NXP_RT_I2S_CLOCK_BITS_0;
		pre_div = DT_INST_0_NXP_RT_I2S_CLOCK_BITS_1;
		src_div = DT_INST_0_NXP_RT_I2S_CLOCK_BITS_2;
		/*Clock setting for SAI1*/
		CLOCK_SetMux(kCLOCK_Sai1Mux, clK_src);
		CLOCK_SetDiv(kCLOCK_Sai1PreDiv, pre_div);
		CLOCK_SetDiv(kCLOCK_Sai1Div, src_div);
	} else if (DEV_CFG(dev)->i2s_id == 1) {
		src = DT_INST_1_NXP_RT_I2S_PLL_CLOCKS_VALUE_0;
		lp  = DT_INST_1_NXP_RT_I2S_PLL_CLOCKS_VALUE_1;
		pd  = DT_INST_1_NXP_RT_I2S_PLL_CLOCKS_VALUE_2;
		num = DT_INST_1_NXP_RT_I2S_PLL_CLOCKS_VALUE_3;
		den = DT_INST_1_NXP_RT_I2S_PLL_CLOCKS_VALUE_4;
		clK_src = DT_INST_1_NXP_RT_I2S_CLOCK_BITS_0;
		pre_div = DT_INST_1_NXP_RT_I2S_CLOCK_BITS_1;
		src_div = DT_INST_1_NXP_RT_I2S_CLOCK_BITS_2;
		/*Clock setting for SAI2*/
		CLOCK_SetMux(kCLOCK_Sai2Mux, clK_src);
		CLOCK_SetDiv(kCLOCK_Sai2PreDiv, pre_div);
		CLOCK_SetDiv(kCLOCK_Sai2Div, src_div);
	} else if (DEV_CFG(dev)->i2s_id == 2) {
		src = DT_INST_2_NXP_RT_I2S_PLL_CLOCKS_VALUE_0;
		lp  = DT_INST_2_NXP_RT_I2S_PLL_CLOCKS_VALUE_1;
		pd  = DT_INST_2_NXP_RT_I2S_PLL_CLOCKS_VALUE_2;
		num = DT_INST_2_NXP_RT_I2S_PLL_CLOCKS_VALUE_3;
		den = DT_INST_2_NXP_RT_I2S_PLL_CLOCKS_VALUE_4;
		clK_src = DT_INST_2_NXP_RT_I2S_CLOCK_BITS_0;
		pre_div = DT_INST_2_NXP_RT_I2S_CLOCK_BITS_1;
		src_div = DT_INST_2_NXP_RT_I2S_CLOCK_BITS_2;
		/*Clock setting for SAI3*/
		CLOCK_SetMux(kCLOCK_Sai3Mux, clK_src);
		CLOCK_SetDiv(kCLOCK_Sai3PreDiv, pre_div);
		CLOCK_SetDiv(kCLOCK_Sai3Div, src_div);
	} else {
		LOF_ERR("i2s bus id does not support");
	}
	audioPllConfig.loopDivider = ld;
	audioPllConfig.postDivider = pd;
	audioPllConfig.numerator = num;
	audioPllConfig.denominator = den;
	audioPllConfig.src = src;

	LOG_DBG("loopDivider = %d", ld);
	LOG_DBG("postDivider = %d", pd);
	LOG_DBG("numerator = %d", num);
	LOG_DBG("denominator = %d", den);
	LOG_DBG("src = %d", src);
	LOG_DBG("clK_src = %d", clK_src);
	LOG_DBG("pre_div = %d", pre_div);
	LOG_DBG("src_div = %d", src_div);

	CLOCK_InitAudioPll(&audioPllConfig);

}

static int i2s_rt_initialize(struct device *dev)
{
	const struct i2s_rt_config *const dev_cfg = DEV_CFG(dev);
	struct i2s_dev_data *const dev_data = DEV_DATA(dev);
	u32_t mclk;

	dev_data->dev_dma = device_get_binding(dev_cfg->edma_name);
	if (!dev_data->dev_dma) {
		LOG_ERR("%s device not found", dev_cfg->edma_name);
		return -ENODEV;
	}

	/* Initialize the buffer queues */
	k_msgq_init(&dev_data->tx.in_queue, (char *)dev_data->tx.in_msgs,
		    sizeof(void *), CONFIG_I2S_BUFFER_QUEUE_COUNT);
	k_msgq_init(&dev_data->rx.in_queue, (char *)dev_data->rx.in_msgs,
		    sizeof(void *), CONFIG_I2S_BUFFER_QUEUE_COUNT);
	k_msgq_init(&dev_data->tx.out_queue, (char *)dev_data->tx.out_msgs,
		    sizeof(void *), CONFIG_I2S_BUFFER_QUEUE_COUNT);
	k_msgq_init(&dev_data->rx.out_queue, (char *)dev_data->rx.out_msgs,
		    sizeof(void *), CONFIG_I2S_BUFFER_QUEUE_COUNT);

	/* register ISR */
	dev_cfg->irq_connect();

	/*clock configuration*/
	_audio_clock_settings(dev);

	SAI_Init(DEV_BASE(dev));
	/*audio does not have call back to user*/
	SAI_TransferTxCreateHandle(DEV_BASE(dev), &dev_data->tx.handle,
				   _callback, NULL);
	SAI_TransferRxCreateHandle(DEV_BASE(dev), &dev_data->rx.handle,
				   _callback, NULL);

	/*Enable MCLK clock*/
	BOARD_EnableSaiMclkOutput(true);

	dev_data->tx.state = I2S_STATE_NOT_READY;
	dev_data->rx.state = I2S_STATE_NOT_READY;

#if (defined(FSL_FEATURE_SAI_HAS_MCR) && (FSL_FEATURE_SAI_HAS_MCR)) ||         \
	(defined(FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER) &&                      \
	 (FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER))
	sai_master_clock_t mclkConfig = {
#if defined(FSL_FEATURE_SAI_HAS_MCR) && (FSL_FEATURE_SAI_HAS_MCR)
		.mclkOutputEnable = true,
#if !(defined(FSL_FEATURE_SAI_HAS_NO_MCR_MICS) &&                              \
      (FSL_FEATURE_SAI_HAS_NO_MCR_MICS))
		.mclkSource = kSAI_MclkSourceSysclk,
#endif
#endif
	};
#endif

	_get_mclk_rate(dev, &mclk);
/* master clock configurations */
#if (defined(FSL_FEATURE_SAI_HAS_MCR) && (FSL_FEATURE_SAI_HAS_MCR)) ||         \
	(defined(FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER) &&                      \
	 (FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER))
#if defined(FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER) &&                           \
	(FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER)
	mclkConfig.mclkHz = mclk;
	mclkConfig.mclkSourceClkHz = mclk;
#endif
	SAI_SetMasterClockConfig(DEV_BASE(dev), &mclkConfig);
#endif

	LOG_INF("Device %s initialized", DEV_NAME(dev));

	return 0;
}

static const struct i2s_driver_api i2s_rt_driver_api = {
	.configure = i2s_rt_configure,
	.read = i2s_rt_read,
	.write = i2s_rt_write,
	.trigger = i2s_rt_trigger,
};

static void i2s0_irq_connect(void)
{
	I2S_IRQ_CONNECT(0);
}

I2S_DEVICE_CONFIG_DEFINE(0);
I2S_DEVICE_DATA_DEFINE(0);
I2S_DEVICE_AND_API_INIT(0);

static void i2s1_irq_connect(void)
{
	I2S_IRQ_CONNECT(1);
}

I2S_DEVICE_CONFIG_DEFINE(1);
I2S_DEVICE_DATA_DEFINE(1);
I2S_DEVICE_AND_API_INIT(1);

static void i2s2_irq_connect(void)
{
	I2S_IRQ_CONNECT(2);
}

I2S_DEVICE_CONFIG_DEFINE(2);
I2S_DEVICE_DATA_DEFINE(2);
I2S_DEVICE_AND_API_INIT(2);
