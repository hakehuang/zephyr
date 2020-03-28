/*
 * Copyright (c) 2016 Freescale Semiconductor, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <device.h>
#include <drivers/pinmux.h>
#include <fsl_common.h>
#include <fsl_clock.h>

struct pinmux_imx_config {
	clock_ip_name_t clock_ip_name;
	void *base;
};

static int pinmuxgpr_imx_set(struct device *dev, u32_t offset, u32_t value)
{
	const struct pinmux_imx_config *config = dev->config->config_info;
	u32_t * base = (IOMUXC_GPR_Type *)config->base;

	*(base + offset) = value;

	return 0;
}

static int pinmuxgpr_imx_get(struct device *dev, u32_t offset, u32_t *value)
{
	const struct pinmux_imx_config *config = dev->config->config_info;
	u32_t * base = (IOMUXC_GPR_Type *)config->base;

	*value = *(base + offset);

	return 0;
}

static int pinmuxgpr_imx_pullup(struct device *dev, u32_t offset, u8_t value)
{
	return -ENOTSUP;
}

static int pinmuxgpr_imx_input(struct device *dev, u32_t offset, u8_t value)
{
	const struct pinmux_imx_config *config = dev->config->config_info;
	u32_t * base = (IOMUXC_GPR_Type *)config->base;
	*(base + offset) = value;
}

static int pinmuxgpr_imx_init(struct device *dev)
{
	const struct pinmux_imx_config *config = dev->config->config_info;

	CLOCK_EnableClock(config->clock_ip_name);

	return 0;
}

static const struct pinmux_driver_api pinmuxgpr_imx_driver_api = {
	.set = pinmuxgpr_imx_set,
	.get = pinmuxgpr_imx_get,
	.pullup = pinmuxgpr_imx_pullup,
	.input = pinmuxgpr_imx_input,
};

static const struct pinmux_imx_config pinmuxgpr_imx_config = {
	.base = IOMUXC_GPR,
	.clock_ip_name = kCLOCK_IomuxcGpr,
};

DEVICE_AND_API_INIT(pinmux_gpr, NULL,
		    &pinmuxgpr_imx_init,
		    NULL, &pinmuxgpr_imx_config,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &pinmuxgpr_imx_driver_api);

