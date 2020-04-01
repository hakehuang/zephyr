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

#define DEV_CFG(dev)                                                           \
	((const struct pinmux_imx_config *const)(dev)->config->config_info)
#define DEV_BASE(dev) ((void *)DEV_CFG(dev)->base)

static int pinmuxgpr_imx_set(struct device *dev, u32_t offset, u32_t value)
{
	u32_t *base = (u32_t *)DEV_BASE(dev);

	*(base + offset) = value;

	return 0;
}

static int pinmuxgpr_imx_get(struct device *dev, u32_t offset, u32_t *value)
{
	u32_t *base = (u32_t *)DEV_BASE(dev);

	*value = *(base + offset);

	return 0;
}

static int pinmux_imx_pullup(struct device *dev, u32_t offset, u8_t value)
{
	return -ENOTSUP;
}

static int pinmux_imx_input(struct device *dev, u32_t offset, u8_t value)
{
	return -ENOTSUP;
}

static int pinmuxgpr_imx_init(struct device *dev)
{
	const struct pinmux_imx_config *config = DEV_CFG(dev);

	CLOCK_EnableClock(config->clock_ip_name);

	return 0;
}

static const struct pinmux_driver_api pinmuxgpr_imx_driver_api = {
	.set = pinmuxgpr_imx_set,
	.get = pinmuxgpr_imx_get,
};

static const struct pinmux_imx_config pinmuxgpr_imx_config = {
	.base = IOMUXC_GPR,
	.clock_ip_name = kCLOCK_IomuxcGpr,
};

DEVICE_AND_API_INIT(pinmux_gpr, DT_INST_0_NXP_IMX_PINMUX_LABEL,
		    &pinmuxgpr_imx_init, NULL, &pinmuxgpr_imx_config,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &pinmuxgpr_imx_driver_api);
