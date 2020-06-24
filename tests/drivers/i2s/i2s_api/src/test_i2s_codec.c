/*
 * Copyright (c) 2020 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr.h>
#include <ztest.h>
#include <audio/codec.h>
#include <drivers/i2s.h>
#include "i2s_api_test.h"

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
                .options = I2S_OPT_FRAME_CLK_SLAVE | I2S_OPT_BIT_CLK_SLAVE,
                .frame_clk_freq = FRAME_CLK_FREQ,
                .block_size = BLOCK_SIZE,
                .timeout = TIMEOUT,
                .options = AUDIO_ROUTE_PACKBACK_CAPTURE,
            }
        },
        .dai_route = AUDIO_ROUTE_PACKBACK_CAPTURE,
    };
    audio_property_value_t val;
    val.loopback = true;
    zassert_not_null(dev_codec, "device " CODEC_DEV_NAME " not found");
    audio_codec_configure(dev_codec, &cfg);
    audio_codec_start_output(dev_codec);
    // audio_codec_set_property(dev_codec, AUDIO_PROPERTY_LOOPBACK, AUDIO_CHANNEL_ALL, val);
    return 0;
}

#endif
