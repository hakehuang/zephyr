/*
 * Copyright 2025 NXP
 * SPDX-License-Identifier: Apache-2.0
 *
 * @brief ADC driver for the Texas Instruments ADS1256
 * 8-channel 24-bit SPI ADC
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/adc/ads1256.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

#define DT_DRV_COMPAT ti_ads1256

LOG_MODULE_REGISTER(ADS1256, CONFIG_ADC_LOG_LEVEL);

#define ADS1256_RESOLUTION (24U)
#define ADS1256_DT_MAX_CHANNELS (8U)
#define WAIT_DRDY_TIMES	(10U)

/* Status Register Bits */
#define ADS1256_STATUS_ORDER (1 << 3)
#define ADS1256_STATUS_ACAL  (1 << 2)
#define ADS1256_STATUS_BUFEN (1 << 1)
#define ADS1256_STATUS_DRDY  (1 << 0)

/* MUX Register Bits */
#define ADS1256_MUX_PGA_BYPASS (1 << 3)
#define ADS1256_MUX_SEL_AIN0   0x00
#define ADS1256_MUX_SEL_AIN1   0x01
#define ADS1256_MUX_SEL_AIN2   0x02
#define ADS1256_MUX_SEL_AIN3   0x03
#define ADS1256_MUX_SEL_AIN4   0x04
#define ADS1256_MUX_SEL_AIN5   0x05
#define ADS1256_MUX_SEL_AIN6   0x06
#define ADS1256_MUX_SEL_AIN7   0x07
#define ADS1256_MUX_SEL_AINCOM 0x08

/* ADCON Register Bits */
#define ADS1256_ADCON_DRATE_MASK 0xF0
#define ADS1256_ADCON_PGA_MASK   0x07

/* Default configuration values */
#define ADS1256_DEFAULT_GAIN      1
#define ADS1256_DEFAULT_DATA_RATE 7
#define ADS1256_DEFAULT_BUFFER    0
#define ADS1256_DEFAULT_ACAL      0
#define ADS1256_DEFAULT_ORDER     0

#define ADS1256_RESET_DELAY_MS     1

#define DT_DRV_COMPAT ti_ads1256

/* ADS1256 Commands */
enum ads1256_cmd {
	ADS1256_CMD_WAKEUP = 0xFF,
	ADS1256_CMD_RDATA = 0x01,
	ADS1256_CMD_RDATAC = 0x03,
	ADS1256_CMD_SDATAC = 0x0F,
	ADS1256_CMD_RREG = 0x10,
	ADS1256_CMD_WREG = 0x50,
	ADS1256_CMD_SELFCAL = 0xF0,
	ADS1256_CMD_SELFOCAL = 0xF1,
	ADS1256_CMD_SELFGCAL = 0xF2,
	ADS1256_CMD_SYSOCAL = 0xF3,
	ADS1256_CMD_SYSGCAL = 0xF4,
	ADS1256_CMD_SYNC = 0xFC,
	ADS1256_CMD_STANDBY = 0xFD,
	ADS1256_CMD_RESET = 0xFE,
};

/* ADS1256 Register Map */
enum ads1256_reg {
	ADS1256_REG_STATUS = 0x00,
	ADS1256_REG_MUX = 0x01,
	ADS1256_REG_ADCON = 0x02,
	ADS1256_REG_DRATE = 0x03,
	ADS1256_REG_IO = 0x04,
	ADS1256_REG_OFC0 = 0x05,
	ADS1256_REG_OFC1 = 0x06,
	ADS1256_REG_OFC2 = 0x07,
	ADS1256_REG_FSC0 = 0x08,
	ADS1256_REG_FSC1 = 0x09,
	ADS1256_REG_FSC2 = 0x0A,
};

struct ads1256_config {
	struct spi_dt_spec bus;
	struct gpio_dt_spec drdy_gpio;
	struct gpio_dt_spec reset_gpio;
	struct gpio_dt_spec sync_gpio;
	bool standby_on_boot;
	uint32_t data_rate;
};

struct ads1256_data {
	struct adc_context ctx;
	const struct device *dev;
	k_timeout_t ready_time;
	struct k_sem device_sem;
	struct k_sem drdy_sem;
	struct gpio_callback drdy_cb;
	uint32_t *buffer;
	uint32_t *repeat_buffer;
	bool differential;
};

#define ADS1256_CONFIG_MUX(x) (FIELD_PREP(BIT_MASK(3) << 5, x))

static int ads1256_wait_data_ready(const struct device *dev);

static int ads1256_read_register(const struct device *dev, uint8_t reg, uint8_t *value)
{
	const struct ads1256_config *config = dev->config;
	uint8_t tx_buf[2] = {ADS1256_CMD_RREG | reg, 0};
	uint8_t rx_buf[1] = {0};

	const struct spi_buf tx_bufs[] = {
		{.buf = tx_buf, .len = ARRAY_SIZE(tx_buf)},
		{.buf = NULL, .len = ARRAY_SIZE(rx_buf)}
	};
	const struct spi_buf rx_bufs[] = {
		{.buf = NULL, .len = ARRAY_SIZE(tx_buf)},
		{.buf = rx_buf, .len = ARRAY_SIZE(rx_buf)}
	};
	const struct spi_buf_set tx = {.buffers = tx_bufs, .count = ARRAY_SIZE(tx_bufs)};
	const struct spi_buf_set rx = {.buffers = rx_bufs, .count = ARRAY_SIZE(rx_bufs)};

	int ret = spi_transceive_dt(&config->bus, &tx, &rx);
	if (ret != 0) {
		LOG_ERR("Failed to read register 0x%02X: %d", reg, ret);
		return ret;
	}

	*value = rx_buf[0];
	return 0;
}

static int ads1256_write_register(const struct device *dev, uint8_t reg, uint8_t val)
{
	const struct ads1256_config *config = dev->config;
	uint8_t tx_buf[] = {ADS1256_CMD_WREG | reg, 0, val};
	const struct spi_buf tx_bufs[] = {{.buf = tx_buf, .len = ARRAY_SIZE(tx_buf)}};
	const struct spi_buf_set tx = {.buffers = tx_bufs, .count = ARRAY_SIZE(tx_bufs)};

	ads1256_wait_data_ready(dev);
	return spi_write_dt(&config->bus, &tx);
}

static int ads1256_exec_simple_cmd(const struct device *dev, uint8_t cmd)
{
	const struct ads1256_config *config = dev->config;
	uint8_t tx_buf[1] = {cmd};
	const struct spi_buf tx_bufs[] = {{.buf = tx_buf, .len = ARRAY_SIZE(tx_buf)}};
	const struct spi_buf_set tx = {.buffers = tx_bufs, .count = ARRAY_SIZE(tx_bufs)};

	return spi_write_dt(&config->bus, &tx);
}

static int ads1256_sync(const struct device *dev)
{
	return ads1256_exec_simple_cmd(dev, ADS1256_CMD_SYNC);
}

static int ads1256_standby(const struct device *dev)
{
	return ads1256_exec_simple_cmd(dev, ADS1256_CMD_STANDBY);
}

static int ads1256_wakeup(const struct device *dev)
{
	return ads1256_exec_simple_cmd(dev, ADS1256_CMD_WAKEUP);
}

static int ads1256_send_command(const struct device *dev, enum ads1256_cmd cmd)
{
	const struct ads1256_config *config = dev->config;
	uint8_t cmd_buf[1];
	const struct spi_buf tx_buf[] = {{.buf = cmd_buf, .len = ARRAY_SIZE(cmd_buf)}};
	const struct spi_buf_set tx = {.buffers = tx_buf, .count = ARRAY_SIZE(tx_buf)};

	cmd_buf[0] = cmd;
	return spi_write_dt(&config->bus, &tx);
}

/* Add status check function */
static int ads1256_check_status(const struct device *dev, uint8_t *status)
{
	return ads1256_read_register(dev, ADS1256_REG_STATUS, status);
}

static int ads1256_wait_data_ready(const struct device *dev)
{
	int ret;
	uint8_t status = 1;
	uint32_t count = 0;
	struct ads1256_data *data = dev->data;

	k_sleep(data->ready_time);


	while (status & ADS1256_STATUS_DRDY) {
		k_sleep(data->ready_time);
		ret = ads1256_check_status(dev, &status);
		if (ret != 0) {
			LOG_ERR("Failed to check status: %d", ret);
			return ret;
		}
		count++;
		if (count > WAIT_DRDY_TIMES) {
			LOG_WRN("Exceeded maximum DRDY checks (%d)", WAIT_DRDY_TIMES);
			return -ETIMEDOUT;
		}
	}

	return 0;
}

static const uint8_t data_rate_map[] = {
	[ADS1256_DRATE_30000] = ADS1256_DRATE_30000_VAL,
	[ADS1256_DRATE_15000] = ADS1256_DRATE_15000_VAL,
	[ADS1256_DRATE_1000] = ADS1256_DRATE_1000_VAL,
	[ADS1256_DRATE_100] = ADS1256_DRATE_100_VAL,
	[ADS1256_DRATE_60] = ADS1256_DRATE_60_VAL,
	[ADS1256_DRATE_50] = ADS1256_DRATE_50_VAL
};

static int ads1256_set_data_rate(const struct device *dev, uint32_t data_rate)
{
	if (data_rate >= ARRAY_SIZE(data_rate_map)) {
		data_rate = ADS1256_DRATE_50;
	}
	return ads1256_write_register(dev, ADS1256_REG_DRATE, data_rate_map[data_rate]);
}

static int ads1256_read_sample(const struct device *dev)
{
	const struct ads1256_config *config = dev->config;
	struct ads1256_data *data = dev->data;
	uint32_t buf;
	uint8_t tx_buf[1] = {ADS1256_CMD_RDATA};
	uint8_t rx_buf[3] = {1};
	const struct spi_buf tx_bufs[] = {
		{.buf = tx_buf, .len = ARRAY_SIZE(tx_buf)},
		{.buf = NULL, .len = ARRAY_SIZE(rx_buf)}
	};
	const struct spi_buf rx_bufs[] = {
		{.buf = NULL, .len = ARRAY_SIZE(tx_buf)},
		{.buf = rx_buf, .len = ARRAY_SIZE(rx_buf)}
	};
	const struct spi_buf_set tx = {.buffers = tx_bufs, .count = ARRAY_SIZE(tx_bufs)};
	const struct spi_buf_set rx = {.buffers = rx_bufs, .count = ARRAY_SIZE(rx_bufs)};

		/* Wait for device ready */
	k_sem_take(&data->device_sem, K_FOREVER);
	
	/* Wait for DRDY signal */
	if (ads1256_wait_data_ready(dev) != 0) {
		k_sem_give(&data->device_sem);
		return -EIO;
	}

	int ret = spi_transceive_dt(&config->bus, &tx, &rx);
	if (ret != 0) {
		LOG_ERR("Failed to read");
		return ret;
	}
	buf = sys_get_be24(rx_buf);
	if (data->repeat_buffer != NULL) {
		*data->repeat_buffer = buf;
	} else {
		*data->buffer = buf;
	}

	/* release device */
	k_sem_give(&data->device_sem);
	adc_context_on_sampling_done(&data->ctx, dev);

	return 0;
}

static int ads1256_set_gain(const struct device *dev, enum adc_gain gain)
{
	static const uint8_t gain_map[] = {
		[ADC_GAIN_1] = 0,
		[ADC_GAIN_2] = 1,
		[ADC_GAIN_4] = 2,
		[ADC_GAIN_8] = 4,
		[ADC_GAIN_16] = 8,
		[ADC_GAIN_32] = 16,
		[ADC_GAIN_64] = 32
	};
	uint8_t adcon;

	if (gain >= ARRAY_SIZE(gain_map) || gain_map[gain] == 0) {
		LOG_WRN("gain not support");
		return -EINVAL;
	}

	ads1256_read_register(dev, ADS1256_REG_ADCON, &adcon);
	adcon &= ~ADS1256_ADCON_PGA_MASK;
	adcon |= FIELD_PREP(ADS1256_ADCON_PGA_MASK, gain_map[gain]);
	return ads1256_write_register(dev, ADS1256_REG_ADCON, adcon);
}

static int ads1256_channel_setup(const struct device *dev,
				 const struct adc_channel_cfg *channel_cfg)
{
	struct ads1256_data *data = dev->data;
	int ret;

	ads1256_sync(dev);
	ads1256_wakeup(dev);

	if (channel_cfg->differential) {
		uint8_t mux =
			(channel_cfg->input_positive << 4) | (channel_cfg->input_negative & 0x0F);

		ret = ads1256_write_register(dev, ADS1256_REG_MUX, mux);
		if (ret) {
			return ret;
		}

		data->differential = true;
	} else {
		uint8_t mux = ((channel_cfg->input_positive << 4) | 0x8);

		ret = ads1256_write_register(dev, ADS1256_REG_MUX, mux);
		if (ret) {
			return ret;
		}

		data->differential = false;
	}

	ads1256_set_gain(dev, channel_cfg->gain);

	#if 0
	{
		uint8_t reg_val;
		for (uint8_t reg = ADS1256_REG_STATUS; reg <= ADS1256_REG_FSC2; reg++) {
			if (ads1256_read_register(dev, reg, &reg_val) == 0) {
				LOG_DBG("Reg 0x%02X: 0x%02X\n", reg, reg_val);
			}
		}
	}
	#endif
	return 0;
}



static int ads1256_calibrate(const struct device *dev)
{
	uint8_t status;

	ads1256_read_register(dev, ADS1256_REG_STATUS, &status);
	status |= ADS1256_STATUS_ACAL;
	ads1256_write_register(dev, ADS1256_REG_STATUS, status);
	k_sleep(K_MSEC(10));
	ads1256_send_command(dev, ADS1256_CMD_SELFCAL);
	ads1256_wait_data_ready(dev);
	ads1256_send_command(dev, ADS1256_CMD_SELFOCAL);
	ads1256_wait_data_ready(dev);
	return 0;
}

static int ad1256_validate_buffer_size(const struct device *dev,
	const struct adc_sequence *sequence)
{
	uint8_t channels;
	size_t needed;

	channels = POPCOUNT(sequence->channels);
	needed = channels * sizeof(uint32_t);

	if (sequence->buffer_size < needed) {
		return -ENOMEM;
	}

	return 0;
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct ads1256_data *data = CONTAINER_OF(ctx, struct ads1256_data, ctx);
	const struct device * dev = (struct device *)data->dev;
	int ret = 0;

#ifdef CONFIG_ADC_ASYNC
        if (ctx->asynchronous){
            k_sem_take(&data->drdy_sem, K_FOREVER);
        }
#endif
	while (true) {
		ret = ads1256_read_sample(dev);
		if (ret != -EIO) {
			break;
		}
	}
}

static int adc_ads1256_context_start_read(const struct device *dev,
					const struct adc_sequence *sequence,
					bool async)
{
	const struct ads1256_config *config = dev->config;
	struct ads1256_data *data = dev->data;
	int ret;

	ret = ad1256_validate_buffer_size(dev, sequence);
	if (ret < 0) {
		LOG_ERR("insufficient buffer size");
		return ret;
	}

	/* For continuous mode, setup buffer pointers */
	if (sequence->options && sequence->options->interval_us != 0) {
		data->repeat_buffer = sequence->buffer;
	} else {
		data->buffer = sequence->buffer;
	}

	/* Add sync and wakeup commands */
	ret = ads1256_sync(dev);
	if (ret < 0) {
		LOG_ERR("Failed to sync ADS1256");
		return ret;
	}

	ret = ads1256_wakeup(dev);
	if (ret < 0) {
		LOG_ERR("Failed to wakeup ADS1256");
		return ret;
	}

	if (sequence->calibrate) {
		ads1256_calibrate(dev);
	}
	ads1256_wait_data_ready(dev);

	if (async) {
		/* Enable DRDY interrupt for async operation */
		gpio_pin_interrupt_configure_dt(&config->drdy_gpio, GPIO_INT_EDGE_TO_ACTIVE);
		/* Send RDATAC command to start asynchronous conversion */
		//ads1256_send_command(dev, ADS1256_CMD_RDATAC);
	} 
#if 0
	else {
		/* Send RDATA command to start conversion */
		//ads1256_send_command(dev, ADS1256_CMD_RDATA);
	}
#endif

	adc_context_start_read(&data->ctx, sequence);

	return adc_context_wait_for_completion(&data->ctx);
}


static int ads1256_read(const struct device *dev, const struct adc_sequence *sequence)
{
	int ret;
	struct ads1256_data *data = dev->data;

	ads1256_sync(dev);
	ads1256_wakeup(dev);
	ads1256_wait_data_ready(dev);

	adc_context_lock(&data->ctx, false, NULL);
	ret = adc_ads1256_context_start_read(dev, sequence, false);
	adc_context_release(&data->ctx, ret);

	return ret;
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx,
							bool repeat_sampling)
{
	struct ads1256_data *data = CONTAINER_OF(ctx, struct ads1256_data, ctx);

	if (repeat_sampling) {
		/* For repeat sampling, keep using the same buffer */
		data->repeat_buffer = ctx->sequence.buffer;
	}
}

static void ads1256_gpio_data_ready_handler(const struct device *dev,
											struct gpio_callback *gpio_cb,
				    						uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(pins);

	struct ads1256_data *data = CONTAINER_OF(gpio_cb, struct ads1256_data, drdy_cb);

	k_sem_give(&data->drdy_sem);
}

static int ads1256_init(const struct device *dev)
{
	const struct ads1256_config *config = dev->config;
	struct ads1256_data *data = dev->data;

	if (!spi_is_ready_dt(&config->bus)) {
		LOG_ERR("SPI bus %s not ready", config->bus.bus->name);
		return -ENODEV;
	}

	adc_context_init(&data->ctx);

	k_sem_init(&data->device_sem, 1, 1);
	k_sem_init(&data->drdy_sem, 0, 1);

	/* Initialize GPIOs */
	if (!gpio_is_ready_dt(&config->drdy_gpio)) {
		LOG_ERR("DRDY GPIO not ready");
		return -ENODEV;
	}

	if (gpio_pin_configure_dt(&config->drdy_gpio, GPIO_INPUT) < 0) {
		LOG_ERR("Failed to configure DRDY GPIO");
		return -EIO;
	}

	gpio_init_callback(&data->drdy_cb, ads1256_gpio_data_ready_handler, BIT(config->drdy_gpio.pin));

	gpio_add_callback(config->drdy_gpio.port, &data->drdy_cb);

	/* Hardware reset if RESET pin is configured */
	if (config->reset_gpio.port) {
		if (!gpio_is_ready_dt(&config->reset_gpio)) {
			LOG_ERR("Reset GPIO not ready");
			return -ENODEV;
		}

		if (gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_INACTIVE) < 0) {
			LOG_ERR("Failed to configure reset GPIO");
			return -EIO;
		}

		/* Pulse reset */
		gpio_pin_set_dt(&config->reset_gpio, 1);
		k_busy_wait(100);
		gpio_pin_set_dt(&config->reset_gpio, 0);
		k_busy_wait(1000); /* Wait for power-on reset */
	}

	/* Initialize SYNC pin if configured */
	if (config->sync_gpio.port) {
		if (!gpio_is_ready_dt(&config->sync_gpio)) {
			LOG_ERR("Sync GPIO not ready");
			return -ENODEV;
		}

		if (gpio_pin_configure_dt(&config->sync_gpio, GPIO_OUTPUT_INACTIVE) < 0) {
			LOG_ERR("Failed to configure sync GPIO");
			return -EIO;
		}
	}

	if (config->standby_on_boot) {
		ads1256_standby(dev);
	}

	ads1256_set_data_rate(dev, config->data_rate);
    data->dev = dev;
	data->ready_time = K_USEC(1000);
    adc_context_unlock_unconditionally(&data->ctx);
	LOG_INF("ADS1256 initialized successfully");
	return 0;
}

#ifdef CONFIG_ADC_ASYNC
static int ads1256_read_async(const struct device *dev, const struct adc_sequence *sequence,
			      struct k_poll_signal *async)
{
	struct ads1256_data *data = dev->data;
	int ret;

	adc_context_lock(&data->ctx, async ? true : false, async);
	ret = adc_ads1256_context_start_read(dev, sequence, true);
	adc_context_release(&data->ctx, ret);

	return 0;
}
#endif

static const struct adc_driver_api ads1256_driver_api = {
	.channel_setup = ads1256_channel_setup,
	.read = ads1256_read,
#ifdef CONFIG_ADC_ASYNC
	.read_async = ads1256_read_async,
#endif
};

#define ADS1256_INIT(n)                                                                            \
	static struct ads1256_data ads1256_data_##n = {};                                          \
	static const struct ads1256_config ads1256_config_##n = {                                  \
		.bus = SPI_DT_SPEC_INST_GET(n, SPI_OP_MODE_MASTER | SPI_MODE_CPHA | SPI_WORD_SET(8)),           \
		.drdy_gpio = GPIO_DT_SPEC_INST_GET(n, drdy_gpios),                                 \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {0}),                       \
		.sync_gpio = GPIO_DT_SPEC_INST_GET_OR(n, sync_gpios, {0}),                         \
		.data_rate = DT_INST_STRING_TOKEN(n, data_rate),					   \
		.standby_on_boot = DT_INST_PROP(n, standby_on_boot),                               \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, ads1256_init, NULL, &ads1256_data_##n, &ads1256_config_##n,       \
			      POST_KERNEL, CONFIG_ADC_INIT_PRIORITY, &ads1256_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ADS1256_INIT)
