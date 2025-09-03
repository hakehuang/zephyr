/*
 * Copyright 2025 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Emulator for Wolfson WM8900 Audio Codec
 */

#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c_emul.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <string.h>

#include "wm8900.h"
#include "wm8900_emul.h"

#define DT_DRV_COMPAT wolfson_wm8900_emul

LOG_MODULE_REGISTER(wm8900_emul, CONFIG_AUDIO_CODEC_LOG_LEVEL);

/* Number of registers to emulate */
#define WM8900_NUM_REGS 0xFF

struct wm8900_emul_data {
	uint16_t reg[WM8900_NUM_REGS];
};

struct wm8900_emul_cfg {
	uint16_t addr;
};

void wm8900_emul_reset(const struct emul *target)
{
	struct wm8900_emul_data *data = target->data;

	memset(data->reg, 0, sizeof(data->reg));

	/* Set default values for key registers */
	data->reg[WM8900_REG_RESET] = 0x0000;
	data->reg[WM8900_REG_POWER_MGMT_0] = 0x0000;
	data->reg[WM8900_REG_POWER_MGMT_2] = 0x0000;
	data->reg[WM8900_REG_CLK_RATES_0] = 0x0000;
	data->reg[WM8900_REG_AUDIO_IF_0] = 0x0000;
	
	/* Set default volume levels */
	data->reg[WM8900_REG_DAC_DIGITAL_VOLUME_LEFT] = 
		WM8900_REGVAL_IN_VOL(0, WM8900_OUTPUT_VOLUME_DEFAULT);
	data->reg[WM8900_REG_DAC_DIGITAL_VOLUME_RIGHT] = 
		WM8900_REGVAL_IN_VOL(0, WM8900_OUTPUT_VOLUME_DEFAULT);
	data->reg[WM8900_REG_ANALOG_LEFT_IN_0] = 
		WM8900_REGVAL_IN_VOL(0, WM8900_INPUT_VOLUME_DEFAULT);
	data->reg[WM8900_REG_ANALOG_RIGHT_IN_0] = 
		WM8900_REGVAL_IN_VOL(0, WM8900_INPUT_VOLUME_DEFAULT);
}

void wm8900_emul_set_reg(const struct emul *target, uint8_t reg_addr, uint16_t value)
{
	struct wm8900_emul_data *data = target->data;

	if (reg_addr < WM8900_NUM_REGS) {
		data->reg[reg_addr] = value;
	}
}

uint16_t wm8900_emul_get_reg(const struct emul *target, uint8_t reg_addr)
{
	struct wm8900_emul_data *data = target->data;

	if (reg_addr < WM8900_NUM_REGS) {
		return data->reg[reg_addr];
	}
	return 0;
}

int wm8900_emul_set_volume(const struct emul *target, uint8_t channel, uint8_t volume)
{
	struct wm8900_emul_data *data = target->data;
	uint8_t vol_value;

	/* Convert 0-100 scale to 0-63 scale for output volume */
	vol_value = (volume * WM8900_OUTPUT_VOLUME_MAX) / 100;
	vol_value = CLAMP(vol_value, WM8900_OUTPUT_VOLUME_MIN, WM8900_OUTPUT_VOLUME_MAX);

	if (channel == AUDIO_CHANNEL_LEFT || channel == AUDIO_CHANNEL_ALL) {
		data->reg[WM8900_REG_DAC_DIGITAL_VOLUME_LEFT] = 
			WM8900_REGVAL_IN_VOL(0, vol_value);
	}

	if (channel == AUDIO_CHANNEL_RIGHT || channel == AUDIO_CHANNEL_ALL) {
		data->reg[WM8900_REG_DAC_DIGITAL_VOLUME_RIGHT] = 
			WM8900_REGVAL_IN_VOL(0, vol_value);
	}

	return 0;
}

int wm8900_emul_set_mute(const struct emul *target, uint8_t channel, bool muted)
{
	struct wm8900_emul_data *data = target->data;
	uint16_t mute_bit = muted ? WM8900_REGMASK_OUT_MUTE : 0;

	if (channel == AUDIO_CHANNEL_LEFT || channel == AUDIO_CHANNEL_ALL) {
		data->reg[WM8900_REG_DAC_DIGITAL_VOLUME_LEFT] = 
			(data->reg[WM8900_REG_DAC_DIGITAL_VOLUME_LEFT] & ~WM8900_REGMASK_OUT_MUTE) | mute_bit;
	}

	if (channel == AUDIO_CHANNEL_RIGHT || channel == AUDIO_CHANNEL_ALL) {
		data->reg[WM8900_REG_DAC_DIGITAL_VOLUME_RIGHT] = 
			(data->reg[WM8900_REG_DAC_DIGITAL_VOLUME_RIGHT] & ~WM8900_REGMASK_OUT_MUTE) | mute_bit;
	}

	return 0;
}

static int wm8900_emul_handle_write(const struct emul *target, uint8_t reg, uint16_t value)
{
	struct wm8900_emul_data *data = target->data;

	/* Handle special register behaviors */
	switch (reg) {
	case WM8900_REG_RESET:
		/* Reset command - perform soft reset */
		if (value == 0x0000) {
			wm8900_emul_reset(target);
		}
		break;
	
	case WM8900_REG_POWER_MGMT_0:
	case WM8900_REG_POWER_MGMT_2:
	case WM8900_REG_CLK_RATES_0:
	case WM8900_REG_AUDIO_IF_0:
	case WM8900_REG_AUDIO_IF_1:
	case WM8900_REG_DAC_DIGITAL_VOLUME_LEFT:
	case WM8900_REG_DAC_DIGITAL_VOLUME_RIGHT:
	case WM8900_REG_ANALOG_LEFT_IN_0:
	case WM8900_REG_ANALOG_RIGHT_IN_0:
		/* These registers can be written directly */
		data->reg[reg] = value;
		break;
	
	default:
		/* For other registers, just store the value */
		data->reg[reg] = value;
		break;
	}

	return 0;
}

static int wm8900_emul_transfer_i2c(const struct emul *target, struct i2c_msg *msgs,
								int num_msgs, int addr)
{
	struct wm8900_emul_data *data = target->data;

	if (num_msgs < 1) {
		LOG_ERR("Invalid number of messages: %d", num_msgs);
		return -EIO;
	}

	/* First message should be the register address */
	uint8_t reg_addr = msgs->buf[0];
	bool is_read = (msgs->flags & I2C_MSG_READ) != 0;

	if (is_read) {
		/* Read operation */
		if (reg_addr >= WM8900_NUM_REGS) {
			LOG_ERR("Invalid register address: 0x%02X", reg_addr);
			return -EIO;
		}

		/* Return the register value */
		uint16_t reg_value = data->reg[reg_addr];
		msgs->buf[0] = (reg_value >> 8) & 0xFF;
		msgs->buf[1] = reg_value & 0xFF;

		LOG_DBG("Read reg 0x%02X: 0x%04X", reg_addr, reg_value);
	} else {
		/* Write operation */
		if (msgs->len < 3) {
			LOG_ERR("Invalid write message length: %d", msgs->len);
			return -EIO;
		}

		/* Extract the 16-bit value from the message */
		uint16_t value = (msgs->buf[1] << 8) | msgs->buf[2];

		if (reg_addr >= WM8900_NUM_REGS) {
			LOG_ERR("Invalid register address: 0x%02X", reg_addr);
			return -EIO;
		}

		int rc = wm8900_emul_handle_write(target, reg_addr, value);
		if (rc != 0) {
			return rc;
		}

		LOG_DBG("Write reg 0x%02X: 0x%04X", reg_addr, value);
	}

	return 0;
}

static int wm8900_emul_init(const struct emul *target, const struct device *parent)
{
	ARG_UNUSED(parent);
	wm8900_emul_reset(target);
	return 0;
}

static const struct i2c_emul_api wm8900_emul_api_i2c = {
	.transfer = wm8900_emul_transfer_i2c,
};

#define WM8900_EMUL_DEFINE(inst) \
	static struct wm8900_emul_data wm8900_emul_data_##inst; \
	static const struct wm8900_emul_cfg wm8900_emul_cfg_##inst = { \
		.addr = DT_INST_REG_ADDR(inst), \
	}; \
	EMUL_DT_INST_DEFINE(inst, wm8900_emul_init, &wm8900_emul_data_##inst, \
				  &wm8900_emul_cfg_##inst, &wm8900_emul_api_i2c, NULL)

DT_INST_FOREACH_STATUS_OKAY(WM8900_EMUL_DEFINE);