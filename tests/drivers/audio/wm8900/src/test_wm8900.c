/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/audio/codec.h>

#define WM8900_NODE DT_ALIAS(wm8900)

static const struct device *get_wm8900_device(void)
{
	const struct device *dev = DEVICE_DT_GET(WM8900_NODE);

	zassert_true(device_is_ready(dev), "WM8900 device is not ready");

	return dev;
}

ZTEST(wm8900, test_wm8900_init)
{
	const struct device *dev = get_wm8900_device();

	zassert_not_null(dev, "WM8900 device not found");
}

ZTEST(wm8900, test_wm8900_volume_control)
{
	const struct device *dev = get_wm8900_device();
	int ret;

	/* Test volume setting */
	ret = audio_codec_volume_config(dev, AUDIO_CHANNEL_ALL, 50);
	zassert_ok(ret, "Failed to set volume");

	/* Test mute */
	ret = audio_codec_mute_config(dev, AUDIO_CHANNEL_ALL, true);
	zassert_ok(ret, "Failed to mute");

	/* Test unmute */
	ret = audio_codec_mute_config(dev, AUDIO_CHANNEL_ALL, false);
	zassert_ok(ret, "Failed to unmute");
}

ZTEST_SUITE(wm8900, NULL, NULL, NULL, NULL, NULL);