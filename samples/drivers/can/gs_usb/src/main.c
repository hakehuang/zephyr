/*
 * Copyright (c) 2024 Alexander Kozhinov
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(gs_usb_can_sample, CONFIG_GS_USB_LOG_LEVEL);

#include <zephyr/kernel.h>

#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/gs_usb.h>
#include <zephyr/drivers/gpio.h>


int main(void)
{
	int ret = 0;
	const struct device *can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

	ret = device_is_ready(can_dev);
	if (!ret) {
		printk("CAN: Device %s not ready.\n", can_dev->name);
		return ret;
	}

	const struct gpio_dt_spec rx_led = GPIO_DT_SPEC_GET(DT_ALIAS(rxled), gpios);
	ret = gpio_is_ready_dt(&rx_led);
	if (!ret) {
		printk("rx_led device not ready\n");
		return ret;
	}

	const struct gpio_dt_spec tx_led = GPIO_DT_SPEC_GET(DT_ALIAS(txled), gpios);
	ret = gpio_is_ready_dt(&tx_led);
	if (!ret) {
		printk("tx_led device not ready\n");
		return ret;
	}

	const struct gpio_dt_spec term_sw = GPIO_DT_SPEC_GET(DT_ALIAS(termsw), gpios);
	ret = gpio_is_ready_dt(&term_sw);
	if (!ret) {
		printk("termsw device not ready\n");
		return ret;
	}

	ret = gs_usb_init(can_dev, &rx_led, &tx_led, &term_sw);
	if (ret != 0) {
		LOG_ERR("Failed to initialize gs_usb");
		return ret;
	}

	ret = gs_usb_start(0);
	if (ret != 0) {
		LOG_ERR("Failed to start gs_usb");
		return ret;
	}

	k_sleep(K_FOREVER);
	return ret;
}
