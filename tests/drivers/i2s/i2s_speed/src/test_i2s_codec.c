/*
 * Copyright (c) 2021 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr.h>
#include <ztest.h>
#include <audio/codec.h>
#include <drivers/i2s.h>

#ifdef CONFIG_MASTER_CLK
#define CODEC_I2S_OPTIONS (I2S_OPT_FRAME_CLK_SLAVE | I2S_OPT_BIT_CLK_MASTER)
#else
#define CODEC_I2S_OPTIONS (I2S_OPT_FRAME_CLK_SLAVE | I2S_OPT_BIT_CLK_SLAVE)
#endif

#define TIMEOUT         2000
#define FRAME_CLK_FREQ  8000
#define CODEC_DEV_NAME DT_LABEL(DT_INST(0, wolfson_wm8960))
#define SAMPLE_NO   (32)
#define BLOCK_SIZE (2*SAMPLE_NO)

#ifdef CONFIG_AUDIO_CODEC

int set_codec_loop(void)
{
	const struct device *dev_codec = device_get_binding(CODEC_DEV_NAME);
	struct audio_codec_cfg cfg = {
		.mclk_freq = 6144000U,
		.dai_type = AUDIO_DAI_TYPE_I2S,
		.dai_cfg = {
			.i2s = {
				.word_size = 16U,
				.channels = 2U,
				.format = I2S_FMT_DATA_FORMAT_I2S,
				.options = CODEC_I2S_OPTIONS,
				.frame_clk_freq = FRAME_CLK_FREQ,
				.block_size = BLOCK_SIZE,
				.timeout = TIMEOUT,
				.options = AUDIO_ROUTE_PLAYBACK_CAPTURE,
			}
		},
		.dai_route = AUDIO_ROUTE_PLAYBACK_CAPTURE,
	};
	audio_property_value_t val;

	val.loopback = true;
	zassert_not_null(dev_codec, "device " CODEC_DEV_NAME " not found");
	audio_codec_configure(dev_codec, &cfg);
	audio_codec_start_output(dev_codec);
	return 0;
}

#endif
