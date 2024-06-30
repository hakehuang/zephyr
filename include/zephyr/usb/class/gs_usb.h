/*
 * Copyright (c) 2024 Alexander Kozhinov
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_GS_USB_CLASS_DEVICE_H_
#define ZEPHYR_INCLUDE_GS_USB_CLASS_DEVICE_H_

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

/**
 * @brief Initializes gs_usb
 *
 * @param can_dev - can device to be configured
 * @param rx_led - RX LED GPIO specification from devicetree
 * @param tx_led - TX LED GPIO specification from devicetree
 * @param term_res_gpio - termination resistance GPIO specification from devicetree
 * @return int - return code (ok: 0)
 */
int gs_usb_init(const struct device *can_dev,
				const struct gpio_dt_spec *rx_led,
				const struct gpio_dt_spec *tx_led,
				const struct gpio_dt_spec *term_res_gpio);

/**
 * @brief Starts gs_usb in particular: CAN and USB buses
 *
 * @param ch - cjannel number to be started
 * @return int - return code (ok: 0)
 */
int gs_usb_start(const uint32_t ch);

#endif  /* ZEPHYR_INCLUDE_GS_USB_CLASS_DEVICE_H_ */
