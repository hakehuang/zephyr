/*
 * Copyright (c) 2025 Hake Huang <hake.huang@gmail.com>.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ads1256_async_sample, CONFIG_ADC_LOG_LEVEL);

#define ADC_NODE DT_ALIAS(adc0)

#define BUFFER_SIZE 1

static int32_t m_sample_buffer[BUFFER_SIZE] = {0};

int main(void)
{
	const struct device *adc = DEVICE_DT_GET(ADC_NODE);
	int ret;
	int32_t voltage = 0;
#if 1
	if (!device_is_ready(adc)) {
		printk("ADC device not ready");
		return -ENODEV;
	}

	struct adc_sequence_options options = {
		.user_data = NULL,
	};

	struct adc_sequence sequence = {
		.options = &options,
		.buffer = m_sample_buffer,
		.buffer_size = sizeof(m_sample_buffer),
		.resolution = 24,
		.channels = BIT(0),
		.calibrate = true,
	};

	struct adc_channel_cfg channel_cfg = {
		.gain = ADC_GAIN_1,
		.reference = ADC_REF_INTERNAL,
		.acquisition_time = ADC_ACQ_TIME_DEFAULT,
		.channel_id = 0,
		.input_positive = 0
	};

#if 0
	ret = adc_channel_setup(adc, &channel_cfg);
	if (ret < 0) {
		printk("Failed to setup ADC channel: %d\n", ret);
		return ret;
	}
#endif
	printk("Starting ADC\n");

	uint8_t input_channel = 0;
	while (1) {
		channel_cfg.input_positive = input_channel < 8 ? input_channel : 0;
		adc_channel_setup(adc, &channel_cfg);
		if (ret < 0) {
			printk("Failed to setup ADC channel: %d\n", ret);
			return ret;
		}
		ret = adc_read(adc, &sequence);
		if (ret < 0) {
			printk("Failed to read ADC async: %d\n", ret);
			return ret;
		}
		voltage = m_sample_buffer[0];
		/* ads1256 has difference cal mode we need adjust a bit 
		 * and enlarge the first paramter ref_mv to hav emore procise result
		 */
		adc_raw_to_millivolts(5000000, channel_cfg.gain, 23, &voltage);
		printk("\rADC input %d value: %d uv", input_channel, voltage);
		k_sleep(K_MSEC(1000));
		input_channel++;
		if (input_channel > 7) {
			input_channel = 0;
		}
	}
#endif
	return 0;
}
