/*
 * Copyright (c) 2021 NXP Semiconductor INC.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <errno.h>

#include <sys/util.h>

#include <device.h>
#include <drivers/i2c.h>
#include <drivers/gpio.h>

#include <audio/codec.h>

#include <fsl_clock.h>

#define LOG_LEVEL CONFIG_AUDIO_CODEC_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(wolfson_wm8960);

#include "wm8960.h"

#define DT_DRV_COMPAT wolfson_wm8960

#define CODEC_OUTPUT_VOLUME_MAX 0
#define CODEC_OUTPUT_VOLUME_MIN (-78 * 2)

struct codec_driver_config {
	const char *i2c_dev_name;
	uint8_t i2c_address;
	const char *gpio_dev_name;
	uint32_t gpio_pin;
	int gpio_flags;
};

struct codec_driver_data {
	struct device *i2c_device;
	struct device *gpio_device;
	uint16_t reg_cache[WM8960_CACHEREGNUM];
};

static struct codec_driver_config codec_device_config = {
	.i2c_dev_name = DT_INST_BUS_LABEL(0),
	.i2c_address = DT_INST_REG_ADDR(0),
};

static struct codec_driver_data codec_device_data;

static const uint16_t wm8960_reg[WM8960_CACHEREGNUM] = {
	0x0097, 0x0097, 0x0000, 0x0000, 0x0000, 0x0008, 0x0000, 0x000a,
	0x01c0, 0x0000, 0x00ff, 0x00ff, 0x0000, 0x0000, 0x0000, 0x0000,
	0x0000, 0x007b, 0x0100, 0x0032, 0x0000, 0x00c3, 0x00c3, 0x01c0,
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
	0x0100, 0x0100, 0x0050, 0x0050, 0x0050, 0x0050, 0x0000, 0x0000,
	0x0000, 0x0000, 0x0040, 0x0000, 0x0000, 0x0050, 0x0050, 0x0000,
	0x0002, 0x0037, 0x004d, 0x0080, 0x0008, 0x0031, 0x0026, 0x00e9,
};

#define DEV_CFG(dev) ((const struct codec_driver_config *const)dev->config)
#define DEV_DATA(dev) ((struct codec_driver_data *)dev->data)

static void codec_write_reg(const struct device *dev, uint8_t reg,
			    uint16_t val);
static void codec_read_reg(const struct device *dev, uint8_t reg,
			   uint16_t *val);
static void codec_update_reg(const struct device *dev, uint8_t reg,
			     uint16_t mask, uint16_t val);
static void codec_soft_reset(const struct device *dev);
static int codec_configure_dai(const struct device *dev, audio_dai_cfg_t *cfg);
static int codec_configure_clocks(const struct device *dev,
				  struct audio_codec_cfg *cfg);
static int codec_configure_filters(const struct device *dev,
				   audio_dai_cfg_t *cfg);

static void codec_configure_output(const struct device *dev);
static int codec_set_output_volume(const struct device *dev,
				   enum _wm8960_module module,
				   int volume);

#if (LOG_LEVEL >= LOG_LEVEL_DEBUG)
static void codec_read_all_regs(const struct device *dev);
#define CODEC_DUMP_REGS(dev) codec_read_all_regs((dev))
#else
#define CODEC_DUMP_REGS(dev)
#endif

static int codec_initialize(const struct device *dev)
{
	const struct codec_driver_config *dev_cfg = DEV_CFG(dev);
	struct codec_driver_data *dev_data = DEV_DATA(dev);

	/* bind I2C */
	dev_data->i2c_device =
		(struct device *)device_get_binding(dev_cfg->i2c_dev_name);

	if (dev_data->i2c_device == NULL) {
		LOG_ERR("CODEC I2C device binding error");
		LOG_ERR("name = %s", dev_cfg->i2c_dev_name);
		return -ENXIO;
	}

	memcpy(dev_data->reg_cache, wm8960_reg, sizeof(wm8960_reg));

	LOG_DBG("WM8960 init");

	return 0;
}

static int code_set_data_route(const struct device *dev,
			       enum audio_route_t dai_route)
{
	int ret = 0;

	switch (dai_route) {
	case AUDIO_ROUTE_BYPASS:
		/*
		 * Bypass means from line-in to HP
		 * Left LINPUT3 to left output mixer,
		 * LINPUT3 left output mixer volume = 0dB
		 */
		codec_write_reg(dev, WM8960_LOUTMIX, 0x80);

		/*
		 * Right RINPUT3 to right output mixer,
		 * RINPUT3 right output mixer volume = 0dB
		 */
		codec_write_reg(dev, WM8960_ROUTMIX, 0x80);
		break;
	case AUDIO_ROUTE_PLAYBACK:
		/*
		 * Data route I2S_IN-> DAC-> HP
		 * Left DAC to left output mixer,
		 * LINPUT3 left output mixer volume = 0dB
		 */
		codec_write_reg(dev, WM8960_LOUTMIX, 0x100);

		/*
		 * Right DAC to right output mixer,
		 * RINPUT3 right output mixer volume = 0dB
		 */
		codec_write_reg(dev, WM8960_ROUTMIX, 0x100);
		codec_write_reg(dev, WM8960_POWER3, 0x0C);
		/* Set power for DAC */
		codec_write_reg(dev, kWM8960_ModuleDAC, true);
		codec_write_reg(dev, kWM8960_ModuleOMIX, true);
		codec_write_reg(dev, kWM8960_ModuleLineOut, true);
		break;
	case AUDIO_ROUTE_PLAYBACK_CAPTURE:
		/*
		 * Left DAC to left output mixer,
		 * LINPUT3 left output mixer volume = 0dB
		 */
		codec_write_reg(dev, WM8960_LOUTMIX, 0x100);

		/*
		 * Right DAC to right output mixer,
		 * RINPUT3 right output mixer volume = 0dB
		 */
		codec_write_reg(dev, WM8960_ROUTMIX, 0x100);
		codec_write_reg(dev, WM8960_POWER3, 0x3C);
		codec_write_reg(dev, kWM8960_ModuleDAC, true);
		codec_write_reg(dev, kWM8960_ModuleADC, true);
		codec_write_reg(dev, kWM8960_ModuleLineIn, true);
		codec_write_reg(dev, kWM8960_ModuleOMIX, true);
		codec_write_reg(dev, kWM8960_ModuleLineOut, true);
		break;
	case AUDIO_ROUTE_CAPTURE:
		/*
		 * LINE_IN->ADC->I2S_OUT
		 * Left and right input boost, LIN3BOOST and RIN3BOOST = 0dB
		 */
		codec_write_reg(dev, WM8960_POWER3, 0x30);
		/* Power up ADC and AIN */
		codec_write_reg(dev, kWM8960_ModuleLineIn, true);
		codec_write_reg(dev, kWM8960_ModuleADC, true);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int codec_protol_config(const struct device *dev,
			       audio_dai_type_t dai_type)
{
	enum _wm8960_protocol protocol;

	switch (dai_type) {
	case AUDIO_DAI_TYPE_I2S:
		protocol = kWM8960_BusI2S;
		break;
	default:
		return -EINVAL;
	}
	codec_update_reg(dev, WM8960_IFACE1,
			 WM8960_IFACE1_FORMAT_MASK | WM8960_IFACE1_LRP_MASK,
			 protocol);
	LOG_DBG("protocol is 0x%x", protocol);
	return 0;
}

static int codec_configure(const struct device *dev,
			   struct audio_codec_cfg *cfg)
{
	int ret;

	if (cfg->dai_type != AUDIO_DAI_TYPE_I2S) {
		LOG_ERR("dai_type must be AUDIO_DAI_TYPE_I2S");
		return -EINVAL;
	}

	codec_soft_reset(dev);

	/*
	 * VMID=50K, Enable VREF, AINL, AINR, ADCL and ADCR
	 * I2S_IN (bit 0), I2S_OUT (bit 1), DAP (bit 4),
	 * DAC (bit 5), ADC (bit 6) are powered on
	 */
	codec_write_reg(dev, WM8960_POWER1, 0xFE);

	/*
	 * Enable DACL, DACR, LOUT1, ROUT1, PLL down
	 */
	codec_write_reg(dev, WM8960_POWER2, 0x1E0);
	/*
	 * Enable left and right channel input PGA, left and right output mixer
	 */
	codec_write_reg(dev, WM8960_POWER3, 0x3C);
	/* ADC and DAC uses same clock */
	codec_write_reg(dev, WM8960_IFACE2, 0x40);
	ret = code_set_data_route(dev, cfg->dai_route);
	if (ret == 0) {
		ret = codec_protol_config(dev, cfg->dai_type);
		LOG_DBG("config protol done");
	}
	if (ret == 0) {
		ret = codec_configure_filters(dev, &cfg->dai_cfg);
		LOG_DBG("config filter done");
	}
	codec_configure_output(dev);
	ret = codec_configure_clocks(dev, cfg);
	if (ret == 0) {
		ret = codec_configure_dai(dev, &cfg->dai_cfg);
		LOG_DBG("config dai done");
	}

	return ret;
}

static void codec_start_output(const struct device *dev)
{
	/* unmute DAC channels */
	codec_write_reg(dev, WM8960_LDAC, 0x1FF);
	codec_write_reg(dev, WM8960_RDAC, 0x1FF);
	CODEC_DUMP_REGS(dev);
}

static void codec_loopback_enable(const struct device *dev, bool val)
{
	uint16_t reg_val = val ? 0x41 : 0x0;

	codec_write_reg(dev, WM8960_IFACE2, (uint16_t)reg_val);
}

static void codec_stop_output(const struct device *dev)
{
	/* mute DAC channels */
	codec_write_reg(dev, WM8960_LDAC, 0x100);
	codec_write_reg(dev, WM8960_RDAC, 0x100);
}

static void codec_mute_output(const struct device *dev)
{
	/* mute DAC channels */
	codec_write_reg(dev, WM8960_LDAC, 0x100);
	codec_write_reg(dev, WM8960_RDAC, 0x100);
}

static void codec_unmute_output(const struct device *dev)
{
	/* unmute DAC channels */
	codec_write_reg(dev, WM8960_LDAC, 0x1FF);
	codec_write_reg(dev, WM8960_RDAC, 0x1FF);
}

static int codec_set_property(const struct device *dev,
			      audio_property_t property,
			      audio_channel_t channel,
			      audio_property_value_t val)
{
	int ret = 0;
	/* individual channel control not currently supported */
	if (channel != AUDIO_CHANNEL_ALL) {
		LOG_ERR("channel %u invalid. must be AUDIO_CHANNEL_ALL",
			channel);
		return -EINVAL;
	}

	switch (property) {
	case AUDIO_PROPERTY_OUTPUT_VOLUME:
		ret = codec_set_output_volume(dev, kWM8960_ModuleADC, val.vol);
		ret |= codec_set_output_volume(dev, kWM8960_ModuleDAC, val.vol);
		ret |= codec_set_output_volume(dev, kWM8960_ModuleHP, val.vol);
		ret |= codec_set_output_volume(dev, kWM8960_ModuleLineIn, val.vol);
		ret |= codec_set_output_volume(dev, kWM8960_ModuleSpeaker, val.vol);
		return ret;
	case AUDIO_PROPERTY_OUTPUT_MUTE:
		if (val.mute) {
			codec_mute_output(dev);
		} else {
			codec_unmute_output(dev);
		}
		return 0;
	case AUDIO_PROPERTY_LOOPBACK:
		codec_loopback_enable(dev, val.loopback);
		break;
	default:
		break;
	}

	return -EINVAL;
}

static int codec_apply_properties(const struct device *dev)
{
	/* nothing to do because there is nothing cached */
	return 0;
}

static void codec_write_reg(const struct device *dev, uint8_t reg, uint16_t val)
{
	struct codec_driver_data *const dev_data = DEV_DATA(dev);
	const struct codec_driver_config *const dev_cfg = DEV_CFG(dev);
	uint8_t data[2];
	int ret;

	/* set cache if different */
	if (dev_data->reg_cache[reg] != val) {
		dev_data->reg_cache[reg] = val;
	} else {
		return;
	}

	/* data is
	 *   D15..D9 WM8960 register offset
	 *   D8...D0 register data
	 */
	data[0] = (reg << 1) | ((val >> 8) & 0x0001);
	data[1] = val & 0x00ff;

	ret = i2c_reg_write_byte(dev_data->i2c_device, dev_cfg->i2c_address, data[0], data[1]);

	if (ret != 0) {
		LOG_ERR("i2c write to codec error %d", ret);
	}

	LOG_DBG("REG:%02u VAL:0x%02x", reg, val);
}

static void codec_read_reg(const struct device *dev, uint8_t reg, uint16_t *val)
{
	struct codec_driver_data *const dev_data = DEV_DATA(dev);

	/* read cache*/
	*val = dev_data->reg_cache[reg];
	LOG_DBG("REG:%02u VAL:0x%02x", reg, *val);
}

static void codec_update_reg(const struct device *dev, uint8_t reg,
			     uint16_t mask, uint16_t val)
{
	uint16_t reg_val = 0;

	codec_read_reg(dev, reg, &reg_val);
	reg_val &= (uint16_t)~mask;
	reg_val |= val;
	codec_write_reg(dev, reg, reg_val);
}

static void codec_soft_reset(const struct device *dev)
{
	/* soft reset the DAC */
	codec_write_reg(dev, WM8960_RESET, 0x00);
}

static void WM8960_SetLeftInput(const struct device *dev, enum _wm8960_input input)
{
	uint16_t val = 0;

	switch (input) {
	case kWM8960_InputSingleEndedMic:
		/* Only LMN1 enabled, LMICBOOST to 13db, LMIC2B enabled */
		codec_read_reg(dev, WM8960_POWER1, &val);
		val |= (WM8960_POWER1_AINL_MASK | WM8960_POWER1_ADCL_MASK |
			WM8960_POWER1_MICB_MASK);
		codec_write_reg(dev, WM8960_POWER1, val);
		codec_write_reg(dev, WM8960_LINPATH, 0x138);
		codec_write_reg(dev, WM8960_LINVOL, 0x117);
		break;
	case kWM8960_InputDifferentialMicInput2:
		codec_read_reg(dev, WM8960_POWER1, &val);
		val |= (WM8960_POWER1_AINL_MASK | WM8960_POWER1_ADCL_MASK |
			WM8960_POWER1_MICB_MASK);
		codec_write_reg(dev, WM8960_POWER1, val);
		codec_write_reg(dev, WM8960_LINPATH, 0x178);
		codec_write_reg(dev, WM8960_LINVOL, 0x117);
		break;
	case kWM8960_InputDifferentialMicInput3:
		codec_read_reg(dev, WM8960_POWER1, &val);
		val |= (WM8960_POWER1_AINL_MASK | WM8960_POWER1_ADCL_MASK |
			WM8960_POWER1_MICB_MASK);
		codec_write_reg(dev, WM8960_POWER1, val);
		codec_write_reg(dev, WM8960_LINPATH, 0x1B8);
		codec_write_reg(dev, WM8960_LINVOL, 0x117);
		break;
	case kWM8960_InputLineINPUT2:
		codec_read_reg(dev, WM8960_POWER1, &val);
		val |= (WM8960_POWER1_AINL_MASK | WM8960_POWER1_ADCL_MASK);
		codec_write_reg(dev, WM8960_POWER1, val);
		codec_read_reg(dev, WM8960_INBMIX1, &val);
		val |= 0xE;
		codec_write_reg(dev, WM8960_INBMIX1, val);
		break;
	case kWM8960_InputLineINPUT3:
		codec_read_reg(dev, WM8960_POWER1, &val);
		val |= (WM8960_POWER1_AINL_MASK | WM8960_POWER1_ADCL_MASK);
		codec_write_reg(dev, WM8960_POWER1, val);
		codec_read_reg(dev, WM8960_INBMIX1, &val);
		val |= 0x70;
		codec_write_reg(dev, WM8960_INBMIX1, val);
		break;
	default:
		break;
	}
}

static void WM8960_SetRightInput(const struct device *dev, enum _wm8960_input input)
{
	uint16_t val = 0;

	switch (input) {
	case kWM8960_InputSingleEndedMic:
		/* Only LMN1 enabled, LMICBOOST to 13db, LMIC2B enabled */
		codec_read_reg(dev, WM8960_POWER1, &val);
		val |= (WM8960_POWER1_AINR_MASK | WM8960_POWER1_ADCR_MASK |
			WM8960_POWER1_MICB_MASK);
		codec_write_reg(dev, WM8960_POWER1, val);
		codec_write_reg(dev, WM8960_RINPATH, 0x138);
		codec_write_reg(dev, WM8960_RINVOL, 0x117);
		break;
	case kWM8960_InputDifferentialMicInput2:
		codec_read_reg(dev, WM8960_POWER1, &val);
		val |= (WM8960_POWER1_AINR_MASK | WM8960_POWER1_ADCR_MASK |
			WM8960_POWER1_MICB_MASK);
		codec_write_reg(dev, WM8960_POWER1, val);
		codec_write_reg(dev, WM8960_RINPATH, 0x178);
		codec_write_reg(dev, WM8960_RINVOL, 0x117);
		break;
	case kWM8960_InputDifferentialMicInput3:
		codec_read_reg(dev, WM8960_POWER1, &val);
		val |= (WM8960_POWER1_AINR_MASK | WM8960_POWER1_ADCR_MASK |
			WM8960_POWER1_MICB_MASK);
		codec_write_reg(dev, WM8960_POWER1, val);
		codec_write_reg(dev, WM8960_RINPATH, 0x1B8);
		codec_write_reg(dev, WM8960_RINVOL, 0x117);
		break;
	case kWM8960_InputLineINPUT2:
		codec_read_reg(dev, WM8960_POWER1, &val);
		val |= (WM8960_POWER1_AINR_MASK | WM8960_POWER1_ADCR_MASK);
		codec_write_reg(dev, WM8960_POWER1, val);
		codec_read_reg(dev, WM8960_INBMIX2, &val);
		val |= 0xE;
		codec_write_reg(dev, WM8960_INBMIX2, val);
		break;
	case kWM8960_InputLineINPUT3:
		codec_read_reg(dev, WM8960_POWER1, &val);
		val |= (WM8960_POWER1_AINR_MASK | WM8960_POWER1_ADCR_MASK);
		codec_write_reg(dev, WM8960_POWER1, val);
		codec_read_reg(dev, WM8960_INBMIX2, &val);
		val |= 0x70;
		codec_write_reg(dev, WM8960_INBMIX2, val);
		break;
	default:
		break;
	}
}

static void WM8960_SetModule(const struct device *dev, enum _wm8960_module module,
			     bool isEnabled)
{
	switch (module) {
	case kWM8960_ModuleADC:
		codec_update_reg(dev, WM8960_POWER1, WM8960_POWER1_ADCL_MASK,
				 ((uint16_t)isEnabled
				  << WM8960_POWER1_ADCL_SHIFT));
		codec_update_reg(dev, WM8960_POWER1, WM8960_POWER1_ADCR_MASK,
				 ((uint16_t)isEnabled
				  << WM8960_POWER1_ADCR_SHIFT));
		break;
	case kWM8960_ModuleDAC:
		codec_update_reg(dev, WM8960_POWER2, WM8960_POWER2_DACL_MASK,
				 ((uint16_t)isEnabled
				  << WM8960_POWER2_DACL_SHIFT));
		codec_update_reg(dev, WM8960_POWER2, WM8960_POWER2_DACR_MASK,
				 ((uint16_t)isEnabled
				  << WM8960_POWER2_DACR_SHIFT));
		break;
	case kWM8960_ModuleVREF:
		codec_update_reg(dev, WM8960_POWER1, WM8960_POWER1_VREF_MASK,
				 ((uint16_t)isEnabled
				  << WM8960_POWER1_VREF_SHIFT));
		break;
	case kWM8960_ModuleLineIn:
		codec_update_reg(dev, WM8960_POWER1, WM8960_POWER1_AINL_MASK,
				 ((uint16_t)isEnabled
				  << WM8960_POWER1_AINL_SHIFT));
		codec_update_reg(dev, WM8960_POWER1, WM8960_POWER1_AINR_MASK,
				 ((uint16_t)isEnabled
				  << WM8960_POWER1_AINR_SHIFT));
		codec_update_reg(dev, WM8960_POWER3, WM8960_POWER3_LMIC_MASK,
				 ((uint16_t)isEnabled
				  << WM8960_POWER3_LMIC_SHIFT));
		codec_update_reg(dev, WM8960_POWER3, WM8960_POWER3_RMIC_MASK,
				 ((uint16_t)isEnabled
				  << WM8960_POWER3_RMIC_SHIFT));
		break;
	case kWM8960_ModuleLineOut:
		codec_update_reg(dev, WM8960_POWER2, WM8960_POWER2_LOUT1_MASK,
				 ((uint16_t)isEnabled
				  << WM8960_POWER2_LOUT1_SHIFT));
		codec_update_reg(dev, WM8960_POWER2, WM8960_POWER2_ROUT1_MASK,
				 ((uint16_t)isEnabled
				  << WM8960_POWER2_ROUT1_SHIFT));
		break;
	case kWM8960_ModuleMICB:
		codec_update_reg(dev, WM8960_POWER1, WM8960_POWER1_MICB_MASK,
				 ((uint16_t)isEnabled
				  << WM8960_POWER1_MICB_SHIFT));
		break;
	case kWM8960_ModuleSpeaker:
		codec_update_reg(dev, WM8960_POWER2, WM8960_POWER2_SPKL_MASK,
				 ((uint16_t)isEnabled
				  << WM8960_POWER2_SPKL_SHIFT));
		codec_update_reg(dev, WM8960_POWER2, WM8960_POWER2_SPKR_MASK,
				 ((uint16_t)isEnabled
				  << WM8960_POWER2_SPKR_SHIFT));
		codec_write_reg(dev, WM8960_CLASSD1, 0xF7);
		break;
	case kWM8960_ModuleOMIX:
		codec_update_reg(dev, WM8960_POWER3, WM8960_POWER3_LOMIX_MASK,
				 ((uint16_t)isEnabled
				  << WM8960_POWER3_LOMIX_SHIFT));
		codec_update_reg(dev, WM8960_POWER3, WM8960_POWER3_ROMIX_MASK,
				 ((uint16_t)isEnabled
				  << WM8960_POWER3_ROMIX_SHIFT));
		break;
	default:
		break;
	}
}

static int codec_configure_dai(const struct device *dev, audio_dai_cfg_t *cfg)
{
	struct i2s_config *i2s = &cfg->i2s;

	/* set master or slave */
	if (i2s->options & I2S_OPT_BIT_CLK_MASTER) {
		codec_update_reg(dev, WM8960_IFACE1, WM8960_IFACE1_MS_MASK,
				 WM8960_IFACE1_MS(WM8960_IFACE1_MASTER));
	} else {
		codec_update_reg(dev, WM8960_IFACE1, WM8960_IFACE1_MS_MASK,
				 WM8960_IFACE1_MS(WM8960_IFACE1_SLAVE));
	}

	/* select left input */
	WM8960_SetLeftInput(dev, CONFIG_WM8960_LEFT_INPUT);
	/* select right input */
	WM8960_SetRightInput(dev, CONFIG_WM8960_RIGHT_INPUT);

	switch (i2s->word_size) {
	case 16:
		codec_update_reg(dev, WM8960_IFACE1, WM8960_IFACE1_WL_MASK,
				 WM8960_IFACE1_WL(WM8960_IFACE1_WL_16BITS));
		break;
	case 20:
		codec_update_reg(dev, WM8960_IFACE1, WM8960_IFACE1_WL_MASK,
				 WM8960_IFACE1_WL(WM8960_IFACE1_WL_20BITS));
		break;
	case 24:
		codec_update_reg(dev, WM8960_IFACE1, WM8960_IFACE1_WL_MASK,
				 WM8960_IFACE1_WL(WM8960_IFACE1_WL_24BITS));
		break;
	case 32:
		codec_update_reg(dev, WM8960_IFACE1, WM8960_IFACE1_WL_MASK,
				 WM8960_IFACE1_WL(WM8960_IFACE1_WL_32BITS));
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int codec_configure_clocks(const struct device *dev,
				  struct audio_codec_cfg *cfg)
{
	struct i2s_config *i2s = &cfg->dai_cfg.i2s;
	uint16_t val;
	uint32_t divider = 0;

	LOG_DBG("MCLK %u Hz PCM Rate: %u Hz", cfg->mclk_freq,
		i2s->frame_clk_freq);
	/* Compute sample rate divider, dac and adc are the same sample rate */
	divider = cfg->mclk_freq / i2s->frame_clk_freq;
	if (divider == 256) {
		val = 0;
	} else if (divider > 256) {
		val = (((divider / 256U) << 6U) | ((divider / 256U) << 3U));
	} else {
		return -EINVAL;
	}

	codec_write_reg(dev, WM8960_CLOCK1, val);

	LOG_DBG("Timer MCLK Divider: %u", divider);

	return 0;
}

static int codec_configure_filters(const struct device *dev,
				   audio_dai_cfg_t *cfg)
{
	return 0;
}

static void codec_configure_output(const struct device *dev)
{
	/* speaker power */

	WM8960_SetModule(dev, kWM8960_ModuleSpeaker, true);

	codec_write_reg(dev, WM8960_ADDCTL1, 0x0C0);
	codec_write_reg(dev, WM8960_ADDCTL4, 0x40);

	codec_write_reg(dev, WM8960_BYPASS1, 0x0);
	codec_write_reg(dev, WM8960_BYPASS2, 0x0);
	/*
	 * ADC volume, 0dB
	 */
	codec_write_reg(dev, WM8960_LADC, 0x1C3);
	codec_write_reg(dev, WM8960_RADC, 0x1C3);

	/*
	 * Digital DAC volume, 0dB
	 */
	codec_write_reg(dev, WM8960_LDAC, 0x1E0);
	codec_write_reg(dev, WM8960_RDAC, 0x1E0);

	/*
	 * Headphone volume, LOUT1 and ROUT1, 0dB
	 */
	codec_write_reg(dev, WM8960_LOUT1, 0x16F);
	codec_write_reg(dev, WM8960_ROUT1, 0x16F);

	/* Unmute DAC. */
	codec_write_reg(dev, WM8960_DACCTL1, 0x0000);
	codec_write_reg(dev, WM8960_LINVOL, 0x117);
	codec_write_reg(dev, WM8960_RINVOL, 0x117);
}

static int codec_set_output_volume(const struct device *dev,
				   enum _wm8960_module module,
				   int volume)
{
	uint16_t vol = 0;
	int ret = 0;

	switch (module) {
	case kWM8960_ModuleADC:
		vol = volume;
		codec_write_reg(dev, WM8960_LADC, vol);
		codec_write_reg(dev, WM8960_RADC, vol);
		/* Update volume */
		vol = 0x100 | volume;
		codec_write_reg(dev, WM8960_LADC, vol);
		codec_write_reg(dev, WM8960_RADC, vol);
		break;
	case kWM8960_ModuleDAC:
		vol = volume;
		codec_write_reg(dev, WM8960_LDAC, vol);
		codec_write_reg(dev, WM8960_RDAC, vol);
		vol = 0x100 | volume;
		codec_write_reg(dev, WM8960_LDAC, vol);
		codec_write_reg(dev, WM8960_RDAC, vol);
		break;
	case kWM8960_ModuleHP:
		vol = volume;
		codec_write_reg(dev, WM8960_LOUT1, vol);
		codec_write_reg(dev, WM8960_ROUT1, vol);
		vol = 0x100 | volume;
		codec_write_reg(dev, WM8960_LOUT1, vol);
		codec_write_reg(dev, WM8960_ROUT1, vol);
		break;
	case kWM8960_ModuleLineIn:
		vol = volume;
		codec_write_reg(dev, WM8960_LINVOL, vol);
		codec_write_reg(dev, WM8960_RINVOL, vol);
		vol = 0x100 | volume;
		codec_write_reg(dev, WM8960_LINVOL, vol);
		codec_write_reg(dev, WM8960_RINVOL, vol);
		break;
	case kWM8960_ModuleSpeaker:
		vol = volume;
		codec_write_reg(dev, WM8960_LOUT2, vol);
		codec_write_reg(dev, WM8960_ROUT2, vol);
		vol = 0x100 | volume;
		codec_write_reg(dev, WM8960_LOUT2, vol);
		codec_write_reg(dev, WM8960_ROUT2, vol);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

#if (LOG_LEVEL >= LOG_LEVEL_DEBUG)
static void codec_read_all_regs(const struct device *dev)
{
	struct codec_driver_data *const dev_data = DEV_DATA(dev);
	int i = 0;

	for (i = 0; i < WM8960_CACHEREGNUM; i++) {
		LOG_DBG("%d = 0x%x\n", i, dev_data->reg_cache[i]);
	}
}
#endif

static const struct audio_codec_api codec_driver_api = {
	.configure = codec_configure,
	.start_output = codec_start_output,
	.stop_output = codec_stop_output,
	.set_property = codec_set_property,
	.apply_properties = codec_apply_properties,
};

DEVICE_DT_INST_DEFINE(0, codec_initialize, NULL, &codec_device_data,
		&codec_device_config, POST_KERNEL,
		CONFIG_AUDIO_CODEC_INIT_PRIORITY, &codec_driver_api);
