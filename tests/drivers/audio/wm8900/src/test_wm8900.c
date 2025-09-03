/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/audio/codec.h>
#include <zephyr/drivers/emul.h>
#include "wm8900_emul.h"

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
	const struct emul *emul = emul_get_binding(DT_LABEL(DT_ALIAS(wm8900)));
	int ret;

	/* Test volume setting */
	ret = audio_codec_volume_config(dev, AUDIO_CHANNEL_ALL, 50);
	zassert_ok(ret, "Failed to set volume");

	/* Verify volume was set in emulator */
	if (emul != NULL) {
		uint16_t left_vol = wm8900_emul_get_reg(emul, WM8900_REG_DAC_DIGITAL_VOLUME_LEFT);
		uint16_t right_vol = wm8900_emul_get_reg(emul, WM8900_REG_DAC_DIGITAL_VOLUME_RIGHT);
		
		/* Check that volume bits are set appropriately */
		zassert_true((left_vol & WM8900_REGMASK_OUT_VOL) > 0, "Left volume not set");
		zassert_true((right_vol & WM8900_REGMASK_OUT_VOL) > 0, "Right volume not set");
	}

	/* Test mute */
	ret = audio_codec_mute_config(dev, AUDIO_CHANNEL_ALL, true);
	zassert_ok(ret, "Failed to mute");

	/* Verify mute was set in emulator */
	if (emul != NULL) {
		uint16_t left_vol = wm8900_emul_get_reg(emul, WM8900_REG_DAC_DIGITAL_VOLUME_LEFT);
		uint16_t right_vol = wm8900_emul_get_reg(emul, WM8900_REG_DAC_DIGITAL_VOLUME_RIGHT);
		
		/* Check that mute bits are set */
		zassert_true((left_vol & WM8900_REGMASK_OUT_MUTE) != 0, "Left mute not set");
		zassert_true((right_vol & WM8900_REGMASK_OUT_MUTE) != 0, "Right mute not set");
	}

	/* Test unmute */
	ret = audio_codec_mute_config(dev, AUDIO_CHANNEL_ALL, false);
	zassert_ok(ret, "Failed to unmute");

	/* Verify unmute was set in emulator */
	if (emul != NULL) {
		uint16_t left_vol = wm8900_emul_get_reg(emul, WM8900_REG_DAC_DIGITAL_VOLUME_LEFT);
		uint16_t right_vol = wm8900_emul_get_reg(emul, WM8900_REG_DAC_DIGITAL_VOLUME_RIGHT);
		
		/* Check that mute bits are cleared */
		zassert_true((left_vol & WM8900_REGMASK_OUT_MUTE) == 0, "Left mute not cleared");
		zassert_true((right_vol & WM8900_REGMASK_OUT_MUTE) == 0, "Right mute not cleared");
	}
}

ZTEST_SUITE(wm8900, NULL, NULL, NULL, NULL, NULL);