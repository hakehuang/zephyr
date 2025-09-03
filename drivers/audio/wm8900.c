/*
 * Copyright 2025 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/audio/codec.h>
#include <zephyr/devicetree/clocks.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wolfson_wm8900, CONFIG_AUDIO_CODEC_LOG_LEVEL);

#include "wm8900.h"

#define DT_DRV_COMPAT wolfson_wm8900

struct wm8900_driver_config {
	struct i2c_dt_spec i2c;
	int clock_source;
	const struct device *mclk_dev;
	clock_control_subsys_t mclk_name;
};

#define DEV_CFG(dev) ((const struct wm8900_driver_config *const)dev->config)

static void wm8900_write_reg(const struct device *dev, uint8_t reg, uint16_t val);
static void wm8900_read_reg(const struct device *dev, uint8_t reg, uint16_t *val);
static void wm8900_update_reg(const struct device *dev, uint8_t reg, uint16_t mask, uint16_t val);
static void wm8900_soft_reset(const struct device *dev);

static void wm8900_configure_output(const struct device *dev);
static void wm8900_configure_input(const struct device *dev);

static int wm8900_protocol_config(const struct device *dev, audio_dai_type_t dai_type)
{
	wm8900_protocol_t proto;

	switch (dai_type) {
	case AUDIO_DAI_TYPE_I2S:
		proto = kWM8900_ProtocolI2S;
		break;
	case AUDIO_DAI_TYPE_LEFT_JUSTIFIED:
		proto = kWM8900_ProtocolLeftJustified;
		break;
	case AUDIO_DAI_TYPE_RIGHT_JUSTIFIED:
		proto = kWM8900_ProtocolRightJustified;
		break;
	case AUDIO_DAI_TYPE_PCMA:
		proto = kWM8900_ProtocolPCMA;
		break;
	case AUDIO_DAI_TYPE_PCMB:
		proto = kWM8900_ProtocolPCMB;
		break;
	default:
		return -EINVAL;
	}

	wm8900_update_reg(dev, WM8900_REG_AUDIO_IF_1, (0x0003U | (1U << 4U)), (uint16_t)proto);

	LOG_DBG("Codec protocol: %#x", proto);
	return 0;
}

static int wm8900_audio_fmt_config(const struct device *dev, audio_dai_cfg_t *cfg, uint32_t mclk)
{
	wm8900_sample_rate_t wm_sample_rate;
	uint32_t fs;
	uint16_t wmfs_ratio;
	uint16_t mclkDiv;
	uint16_t word_size = cfg->i2s.word_size;

	switch (cfg->i2s.frame_clk_freq) {
	case 8000:
		wm_sample_rate = kWM8900_SampleRate8kHz;
		break;
	case 11025:
		wm_sample_rate = kWM8900_SampleRate11025Hz;
		break;
	case 12000:
		wm_sample_rate = kWM8900_SampleRate12kHz;
		break;
	case 16000:
		wm_sample_rate = kWM8900_SampleRate16kHz;
		break;
	case 22050:
		wm_sample_rate = kWM8900_SampleRate22050Hz;
		break;
	case 24000:
		wm_sample_rate = kWM8900_SampleRate24kHz;
		break;
	case 32000:
		wm_sample_rate = kWM8900_SampleRate32kHz;
		break;
	case 44100:
		wm_sample_rate = kWM8900_SampleRate44100Hz;
		break;
	case 48000:
		wm_sample_rate = kWM8900_SampleRate48kHz;
		break;
	default:
		LOG_WRN("Invalid codec sample rate: %d", cfg->i2s.frame_clk_freq);
		return -EINVAL;
	}

	wm8900_read_reg(dev, WM8900_REG_CLK_RATES_0, &mclkDiv);
	fs = (mclk >> (mclkDiv & 0x1U)) / cfg->i2s.frame_clk_freq;

	switch (fs) {
	case 64:
		wmfs_ratio = kWM8900_FsRatio64X;
		break;
	case 128:
		wmfs_ratio = kWM8900_FsRatio128X;
		break;
	case 192:
		wmfs_ratio = kWM8900_FsRatio192X;
		break;
	case 256:
		wmfs_ratio = kWM8900_FsRatio256X;
		break;
	case 384:
		wmfs_ratio = kWM8900_FsRatio384X;
		break;
	case 512:
		wmfs_ratio = kWM8900_FsRatio512X;
		break;
	case 768:
		wmfs_ratio = kWM8900_FsRatio768X;
		break;
	case 1024:
		wmfs_ratio = kWM8900_FsRatio1024X;
		break;
	case 1408:
		wmfs_ratio = kWM8900_FsRatio1408X;
		break;
	case 1536:
		wmfs_ratio = kWM8900_FsRatio1536X;
		break;
	default:
		LOG_WRN("Invalid Fs ratio: %d", fs);
		return -EINVAL;
	}

	/* Disable SYSCLK */
	wm8900_write_reg(dev, WM8900_REG_CLK_RATES_2, 0x00);

	/* Set Clock ratio and sample rate */
	wm8900_write_reg(dev, WM8900_REG_CLK_RATES_1,
			 ((wmfs_ratio) << 10U) | (uint16_t)(wm_sample_rate));

	switch (cfg->i2s.word_size) {
	case 16:
		word_size = 0;
		break;
	case 20:
		word_size = 1;
		break;
	case 24:
		word_size = 2;
		break;
	case 32:
		word_size = 3;
		break;
	default:
		LOG_ERR("Word size %d bits not supported; falling back to 16 bits",
			cfg->i2s.word_size);
		word_size = 0;
		break;
	}
	/* Set bit resolution */
	wm8900_update_reg(dev, WM8900_REG_AUDIO_IF_1, (0x000CU), ((uint16_t)(word_size) << 2U));

	/* Enable SYSCLK */
	wm8900_write_reg(dev, WM8900_REG_CLK_RATES_2, 0x1007);
	return 0;
}

static int wm8900_out_update(
	const struct device *dev,
	audio_channel_t channel,
	uint16_t val,
	uint16_t mask
)
{
	switch (channel) {
	case AUDIO_CHANNEL_FRONT_LEFT:
		wm8900_update_reg(dev, WM8900_REG_ANALOG_OUT2_LEFT, mask, val);
		return 0;

	case AUDIO_CHANNEL_FRONT_RIGHT:
		wm8900_update_reg(dev, WM8900_REG_ANALOG_OUT2_RIGHT, mask, val);
		return 0;

	case AUDIO_CHANNEL_HEADPHONE_LEFT:
		wm8900_update_reg(dev, WM8900_REG_ANALOG_OUT1_LEFT, mask, val);
		return 0;

	case AUDIO_CHANNEL_HEADPHONE_RIGHT:
		wm8900_update_reg(dev, WM8900_REG_ANALOG_OUT1_RIGHT, mask, val);
		return 0;

	case AUDIO_CHANNEL_ALL:
		wm8900_update_reg(dev, WM8900_REG_ANALOG_OUT1_LEFT, mask, val);
		wm8900_update_reg(dev, WM8900_REG_ANALOG_OUT1_RIGHT, mask, val);
		wm8900_update_reg(dev, WM8900_REG_ANALOG_OUT2_LEFT, mask, val);
		wm8900_update_reg(dev, WM8900_REG_ANALOG_OUT2_RIGHT, mask, val);
		return 0;

	default:
		return -EINVAL;
	}
}

static int wm8900_out_volume_config(const struct device *dev, audio_channel_t channel, int volume)
{
	/* Set volume values with VU = 0 */
	const uint16_t val = WM8900_REGVAL_OUT_VOL(1, 0, 0, volume);
	const uint16_t mask =
		WM8900_REGMASK_OUT_MUTE | WM8900_REGMASK_OUT_VU | WM8900_REGMASK_OUT_ZC | WM8900_REGMASK_OUT_VOL;

	return wm8900_out_update(dev, channel, val, mask);
}

static int wm8900_out_mute_config(const struct device *dev, audio_channel_t channel, bool mute)
{
	const uint16_t val = WM8900_REGVAL_OUT_VOL(mute, 0, 0, 0);
	const uint16_t mask = WM8900_REGMASK_OUT_MUTE;

	return wm8900_out_update(dev, channel, val, mask);
}

static int wm8900_in_update(const struct device *dev, audio_channel_t channel, uint16_t mask,
			    uint16_t val)
{
	switch (channel) {
	case AUDIO_CHANNEL_FRONT_LEFT:
		wm8900_update_reg(dev, WM8900_REG_ANALOG_LEFT_IN_0, mask, val);
		return 0;

	case AUDIO_CHANNEL_FRONT_RIGHT:
		wm8900_update_reg(dev, WM8900_REG_ANALOG_RIGHT_IN_0, mask, val);
		return 0;

	case AUDIO_CHANNEL_ALL:
		wm8900_update_reg(dev, WM8900_REG_ANALOG_LEFT_IN_0, mask, val);
		wm8900_update_reg(dev, WM8900_REG_ANALOG_RIGHT_IN_0, mask, val);
		return 0;

	default:
		return -EINVAL;
	}
}

static int wm8900_in_volume_config(const struct device *dev, audio_channel_t channel, int volume)
{
	const uint16_t val = WM8900_REGVAL_IN_VOL(0, volume);
	const uint16_t mask = WM8900_REGMASK_IN_MUTE | WM8900_REGMASK_IN_VOLUME;

	return wm8900_in_update(dev, channel, mask, val);
}

static int wm8900_in_mute_config(const struct device *dev, audio_channel_t channel, bool mute)
{
	const uint16_t val = WM8900_REGVAL_IN_VOL(mute, 0);
	const uint16_t mask = WM8900_REGMASK_IN_MUTE;

	return wm8900_in_update(dev, channel, mask, val);
}

static int wm8900_route_input(const struct device *dev, audio_channel_t channel, uint32_t input)
{
	uint8_t reg;

	switch (channel) {
	case AUDIO_CHANNEL_FRONT_LEFT:
		reg = WM8900_REG_ANALOG_LEFT_IN_1;
		break;

	case AUDIO_CHANNEL_FRONT_RIGHT:
		reg = WM8900_REG_ANALOG_RIGHT_IN_1;
		break;

	default:
		return -EINVAL;
	}

	/* Input PGA source */
	wm8900_write_reg(dev, reg, input);
	return 0;
}

static int wm8900_route_output(const struct device *dev, audio_channel_t channel, uint32_t output)
{
	/* Output MIXER */
	switch (channel) {
	case AUDIO_CHANNEL_HEADPHONE_LEFT:
		wm8900_write_reg(dev, WM8900_REG_ANALOG_OUT1_LEFT, output);
		break;
	case AUDIO_CHANNEL_HEADPHONE_RIGHT:
		wm8900_write_reg(dev, WM8900_REG_ANALOG_OUT1_RIGHT, output);
		break;
	case AUDIO_CHANNEL_FRONT_LEFT:
	case AUDIO_CHANNEL_REAR_LEFT:
	case AUDIO_CHANNEL_SIDE_LEFT:
		wm8900_write_reg(dev, WM8900_REG_ANALOG_OUT2_LEFT, output);
		break;
	case AUDIO_CHANNEL_FRONT_RIGHT:
	case AUDIO_CHANNEL_REAR_RIGHT:
	case AUDIO_CHANNEL_SIDE_RIGHT:
		wm8900_write_reg(dev, WM8900_REG_ANALOG_OUT2_RIGHT, output);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int wm8900_init(const struct device *dev)
{
	const struct wm8900_driver_config *config = DEV_CFG(dev);

	if (!device_is_ready(config->i2c.bus)) {
		LOG_ERR("I2C bus %s is not ready", config->i2c.bus->name);
		return -ENODEV;
	}

	/* Perform soft reset */
	wm8900_soft_reset(dev);

	/* Configure default input and output settings */
	wm8900_configure_input(dev);
	wm8900_configure_output(dev);

	LOG_INF("WM8900 codec initialized");
	return 0;
}

static int wm8900_dai_config(const struct device *dev, audio_dai_cfg_t *cfg)
{
	const struct wm8900_driver_config *config = DEV_CFG(dev);
	uint32_t mclk;
	int ret;

	if (cfg == NULL) {
		return -EINVAL;
	}

	/* Get MCLK frequency */
	ret = clock_control_get_rate(config->mclk_dev, config->mclk_name, &mclk);
	if (ret < 0) {
		LOG_ERR("Failed to get MCLK frequency: %d", ret);
		return ret;
	}

	/* Configure protocol */
	ret = wm8900_protocol_config(dev, cfg->type);
	if (ret < 0) {
		LOG_ERR("Failed to configure protocol: %d", ret);
		return ret;
	}

	/* Configure audio format */
	ret = wm8900_audio_fmt_config(dev, cfg, mclk);
	if (ret < 0) {
		LOG_ERR("Failed to configure audio format: %d", ret);
		return ret;
	}

	return 0;
}

static int wm8900_io_config(const struct device *dev, audio_io_cfg_t *cfg)
{
	if (cfg == NULL) {
		return -EINVAL;
	}

	switch (cfg->type) {
	case AUDIO_IO_TYPE_INPUT:
		return wm8900_route_input(dev, cfg->channel, cfg->route);
	case AUDIO_IO_TYPE_OUTPUT:
		return wm8900_route_output(dev, cfg->channel, cfg->route);
	default:
		return -EINVAL;
	}
}

static int wm8900_mute_config(const struct device *dev, audio_channel_t channel, bool mute)
{
	return wm8900_out_mute_config(dev, channel, mute);
}

static int wm8900_volume_config(const struct device *dev, audio_channel_t channel, int volume)
{
	return wm8900_out_volume_config(dev, channel, volume);
}

static int wm8900_mic_config(const struct device *dev, audio_channel_t channel, int volume, bool mute)
{
	int ret;

	ret = wm8900_in_volume_config(dev, channel, volume);
	if (ret < 0) {
		return ret;
	}

	return wm8900_in_mute_config(dev, channel, mute);
}

static int wm8900_configure(const struct device *dev, struct audio_codec_cfg *cfg)
{
	/* WM8900 configuration is handled through dai_config and io_config */
	return 0;
}

static void wm8900_start_output(const struct device *dev)
{
	/* Enable output stages */
	wm8900_update_reg(dev, WM8900_REG_POWER_MGMT_2, 0x0180, 0x0180);
}

static void wm8900_stop_output(const struct device *dev)
{
	/* Disable output stages */
	wm8900_update_reg(dev, WM8900_REG_POWER_MGMT_2, 0x0180, 0x0000);
}

static int wm8900_set_property(const struct device *dev, audio_property_t property,
			       audio_channel_t channel, audio_property_value_t val)
{
	switch (property) {
	case AUDIO_PROPERTY_OUTPUT_VOLUME:
		return wm8900_out_volume_config(dev, channel, val.vol);
	case AUDIO_PROPERTY_OUTPUT_MUTE:
		return wm8900_out_mute_config(dev, channel, val.mute);
	case AUDIO_PROPERTY_INPUT_VOLUME:
		return wm8900_in_volume_config(dev, channel, val.vol);
	case AUDIO_PROPERTY_INPUT_MUTE:
		return wm8900_in_mute_config(dev, channel, val.mute);
	default:
		return -ENOTSUP;
	}
}

static int wm8900_apply_properties(const struct device *dev)
{
	/* WM8900 properties are applied immediately, no caching needed */
	return 0;
}

static int wm8900_clear_errors(const struct device *dev)
{
	/* WM8900 doesn't have error status registers to clear */
	return -ENOSYS;
}

static int wm8900_register_error_callback(const struct device *dev,
				      audio_codec_error_callback_t cb)
{
	/* WM8900 doesn't support error callbacks */
	return -ENOSYS;
}

static const struct audio_codec_api wm8900_api = {
	.configure = wm8900_configure,
	.start_output = wm8900_start_output,
	.stop_output = wm8900_stop_output,
	.set_property = wm8900_set_property,
	.apply_properties = wm8900_apply_properties,
	.clear_errors = wm8900_clear_errors,
	.register_error_callback = wm8900_register_error_callback,
	.init = wm8900_init,
	.dai_config = wm8900_dai_config,
	.io_config = wm8900_io_config,
	.mute_config = wm8900_mute_config,
	.volume_config = wm8900_volume_config,
	.mic_config = wm8900_mic_config,
	.route_input = wm8900_route_input,
	.route_output = wm8900_route_output,
};

static void wm8900_write_reg(const struct device *dev, uint8_t reg, uint16_t val)
{
	const struct wm8900_driver_config *config = DEV_CFG(dev);
	uint8_t data[3];

	data[0] = reg;
	data[1] = (val >> 8) & 0xFF;
	data[2] = val & 0xFF;

	if (i2c_write_dt(&config->i2c, data, sizeof(data)) < 0) {
		LOG_ERR("Failed to write register 0x%02X", reg);
	}
}

static void wm8900_read_reg(const struct device *dev, uint8_t reg, uint16_t *val)
{
	const struct wm8900_driver_config *config = DEV_CFG(dev);
	uint8_t data[2];

	if (i2c_write_dt(&config->i2c, &reg, 1) < 0) {
		LOG_ERR("Failed to select register 0x%02X", reg);
		return;
	}

	if (i2c_read_dt(&config->i2c, data, sizeof(data)) < 0) {
		LOG_ERR("Failed to read register 0x%02X", reg);
		return;
	}

	*val = (data[0] << 8) | data[1];
}

static void wm8900_update_reg(const struct device *dev, uint8_t reg, uint16_t mask, uint16_t val)
{
	uint16_t reg_val;

	wm8900_read_reg(dev, reg, &reg_val);
	reg_val = (reg_val & ~mask) | (val & mask);
	wm8900_write_reg(dev, reg, reg_val);
}

static void wm8900_soft_reset(const struct device *dev)
{
	/* Write to reset register */
	wm8900_write_reg(dev, WM8900_REG_RESET, 0x0000);
}

static void wm8900_configure_output(const struct device *dev)
{
	/* Configure headphone outputs */
	wm8900_write_reg(dev, WM8900_REG_ANALOG_OUT1_LEFT,
			WM8900_REGVAL_OUT_VOL(0, 0, 0, WM8900_OUTPUT_VOLUME_DEFAULT));
	wm8900_write_reg(dev, WM8900_REG_ANALOG_OUT1_RIGHT,
			WM8900_REGVAL_OUT_VOL(0, 0, 0, WM8900_OUTPUT_VOLUME_DEFAULT));

	/* Configure line outputs */
	wm8900_write_reg(dev, WM8900_REG_ANALOG_OUT2_LEFT,
			WM8900_REGVAL_OUT_VOL(0, 0, 0, WM8900_OUTPUT_VOLUME_DEFAULT));
	wm8900_write_reg(dev, WM8900_REG_ANALOG_OUT2_RIGHT,
			WM8900_REGVAL_OUT_VOL(0, 0, 0, WM8900_OUTPUT_VOLUME_DEFAULT));

	/* Enable output stages */
	wm8900_update_reg(dev, WM8900_REG_POWER_MGMT_2, 0x0180, 0x0180);
}

static void wm8900_configure_input(const struct device *dev)
{
	/* Configure input volumes */
	wm8900_write_reg(dev, WM8900_REG_ANALOG_LEFT_IN_0,
			WM8900_REGVAL_IN_VOL(0, WM8900_INPUT_VOLUME_DEFAULT));
	wm8900_write_reg(dev, WM8900_REG_ANALOG_RIGHT_IN_0,
			WM8900_REGVAL_IN_VOL(0, WM8900_INPUT_VOLUME_DEFAULT));

	/* Enable input stages */
	wm8900_update_reg(dev, WM8900_REG_POWER_MGMT_0, 0x0003, 0x0003);
}

#define WM8900_DEFINE(inst)                                                                       \
	static const struct wm8900_driver_config wm8900_config_##inst = {                            \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                        \
		.clock_source = DT_INST_PROP(inst, clock_source),                                         \
		.mclk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                                     \
		.mclk_name = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(inst, name),                     \
	};                                                                                             \
	DEVICE_DT_INST_DEFINE(inst, wm8900_init, NULL, NULL, &wm8900_config_##inst, POST_KERNEL,      \
			    CONFIG_AUDIO_CODEC_INIT_PRIORITY, &wm8900_api);

DT_INST_FOREACH_STATUS_OKAY(WM8900_DEFINE)