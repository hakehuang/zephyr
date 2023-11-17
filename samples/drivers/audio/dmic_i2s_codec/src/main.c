/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/audio/codec.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dmic_sample);

/*
  MCLK for rt595 is 12288000
  DMIC CLK default is 3.072MHZ
  24 bit:
     sample rate 32K
     bit width 24
  16 bit:
     sample rate 48K
     bit width 16
*/

#define MAX_SAMPLE_RATE  32000
#define SAMPLE_BIT_WIDTH 16
#define BYTES_PER_SAMPLE 2
/* Milliseconds to wait for a block to be read. */
#define READ_TIMEOUT     1000

/* Size of a block for 100 ms of audio data. */
#define BLOCK_SIZE(_sample_rate, _number_of_channels) \
	(BYTES_PER_SAMPLE * (_sample_rate / 10) * _number_of_channels)

/* Driver will allocate blocks from this slab to receive audio data into them.
 * Application, after getting a given block from the driver and processing its
 * data, needs to free that block.
 */
#define MAX_BLOCK_SIZE   BLOCK_SIZE(MAX_SAMPLE_RATE, 1)
#define BLOCK_COUNT      4
K_MEM_SLAB_DEFINE_STATIC(blob, MAX_BLOCK_SIZE, BLOCK_COUNT, 4);

int main(void)
{

	int ret;
	struct pcm_stream_cfg stream = {
		.pcm_width = SAMPLE_BIT_WIDTH,
		.mem_slab  = &blob,
	};
	
	struct dmic_cfg dmic_cfg = {
		.io = {
			/* These fields are not currently in use. */
			.min_pdm_clk_freq = 1000000,
			.max_pdm_clk_freq = 3500000,
			.min_pdm_clk_dc   = 40,
			.max_pdm_clk_dc   = 60,
		},
		.streams = &stream,
		.channel = {
			.req_num_streams = 1,
		},
	};
	
	
	const struct device *const dmic_dev = DEVICE_DT_GET(DT_NODELABEL(dmic_dev));
	const struct device *const codec_dev = DEVICE_DT_GET(DT_NODELABEL(audio_codec));
	const struct device *const i2s_dev = DEVICE_DT_GET(DT_NODELABEL(i2s_pdm));
	struct audio_codec_cfg audio_cfg;


	dmic_cfg.channel.req_num_chan = 2;
	dmic_cfg.channel.req_chan_map_lo =
		dmic_build_channel_map(0, 0, PDM_CHAN_LEFT) |
		dmic_build_channel_map(1, 0, PDM_CHAN_RIGHT);
	dmic_cfg.streams[0].pcm_rate = MAX_SAMPLE_RATE;
	dmic_cfg.streams[0].block_size =
		BLOCK_SIZE(dmic_cfg.streams[0].pcm_rate, 1);

	audio_cfg.dai_type = AUDIO_DAI_TYPE_I2S;
	audio_cfg.dai_cfg.i2s.word_size = SAMPLE_BIT_WIDTH;
	audio_cfg.dai_cfg.i2s.channels =  2;
	audio_cfg.dai_cfg.i2s.format = I2S_FMT_DATA_FORMAT_I2S;
	audio_cfg.dai_cfg.i2s.options = I2S_OPT_FRAME_CLK_MASTER;
	audio_cfg.dai_cfg.i2s.frame_clk_freq = MAX_SAMPLE_RATE;
	audio_cfg.dai_cfg.i2s.mem_slab = &blob;
	audio_cfg.dai_cfg.i2s.block_size = MAX_BLOCK_SIZE;

	printf("DMIC sample\n");
	if (!device_is_ready(codec_dev)) {
		LOG_ERR("%s is not ready", codec_dev->name);
		return 0;
	}
	printf("audio codec ready\n");
	/* set to master mode  */
	audio_codec_configure(codec_dev, &audio_cfg);


	if (!device_is_ready(dmic_dev)) {
		LOG_ERR("%s is not ready", dmic_dev->name);
		return 0;
	}
	dmic_configure(dmic_dev, &dmic_cfg);

	if (!device_is_ready(i2s_dev)) {
		LOG_ERR("%s is not ready", i2s_dev->name);
		return 0;
	}
	printf("i2s ready\n");
	/* set to slave mode  */
	audio_cfg.dai_cfg.i2s.options = I2S_OPT_BIT_CLK_SLAVE | I2S_OPT_FRAME_CLK_SLAVE;
	i2s_configure(i2s_dev, I2S_DIR_TX, &audio_cfg.dai_cfg.i2s);
	
	ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
	if (ret < 0) {
		LOG_ERR("START trigger failed: %d", ret);
		return ret;
	}
	printk("========start loop============\n");
	while(1) {
		k_msleep(1000);
	}

	LOG_INF("Exiting");
	return 0;
}
