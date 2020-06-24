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

#define LOG_DOMAIN dev_iomcux
#define LOG_LEVEL 4
#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_DOMAIN);

struct pinmux_mcux_rt_config {
	IOMUXC_GPR_Type *base;
};

static int pinmux_mcux_rt_set(const struct device *dev, uint32_t pin, uint32_t func)
{
	const struct pinmux_mcux_rt_config *config = dev->config;
	IOMUXC_GPR_Type *base = config->base;

	uint32_t * gpr_addr = (uint32_t *)((uint32_t *)base + pin);
	*gpr_addr |= func;

	return 0;
}

static int pinmux_mcux_rt_get(const struct device *dev, uint32_t pin, uint32_t *func)
{
	const struct pinmux_mcux_rt_config *config = dev->config;
	IOMUXC_GPR_Type *base = config->base;

	uint32_t * gpr_addr = (uint32_t *)((uint32_t *)base + pin);

	*func = *gpr_addr;

	return 0;
}

static int pinmux_mcux_rt_pullup(const struct device *dev, uint32_t pin, uint8_t func)
{
	return -ENOTSUP;
}

static int pinmux_mcux_rt_input(const struct device *dev, uint32_t pin, uint8_t func)
{
	return -ENOTSUP;
}

static int pinmux_mcux_rt_init(const struct device *dev)
{	
	printk("rt iomcux init as %s\n", dev->name);
	return 0;
}

static const struct pinmux_driver_api pinmux_mcux_rt_driver_api = {
	.set = pinmux_mcux_rt_set,
	.get = pinmux_mcux_rt_get,
	.pullup = pinmux_mcux_rt_pullup,
	.input = pinmux_mcux_rt_input,
};

static const struct pinmux_mcux_rt_config pinmux_mcux_rt = {
	.base = IOMUXC_GPR
};

DEVICE_AND_API_INIT(iomuxcgpr, CONFIG_PINMUX_MCUX_RT_GPR_NAME,
		    &pinmux_mcux_rt_init,
		    NULL, &pinmux_mcux_rt,
		    POST_KERNEL, CONFIG_PINMUX_INIT_PRIORITY,
		    &pinmux_mcux_rt_driver_api);
