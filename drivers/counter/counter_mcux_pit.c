/*
 * Copyright (c) 2020 NXP Semiconductor INC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_kinetis_pit

#include <drivers/counter.h>
#include <fsl_pit.h>

#define LOG_MODULE_NAME counter_pit
#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME, CONFIG_COUNTER_LOG_LEVEL);


struct mcux_pit_config {
	struct counter_config_info info;
	PIT_Type *base;
	bool enableRunInDebug;
	pit_chnl_t pit_channel;
	void (*irq_config_func)(struct device *dev);
};

struct mcux_pit_data {
	counter_top_callback_t top_callback;
	void *top_user_data;
};

static int mcux_pit_start(struct device *dev)
{
	const struct mcux_pit_config *config = dev->config_info;

	PIT_EnableInterrupts(config->base, config->pit_channel,
			     kPIT_TimerInterruptEnable);

	PIT_StartTimer(config->base, config->pit_channel);

	return 0;
}

static int mcux_pit_stop(struct device *dev)
{
	const struct mcux_pit_config *config = dev->config_info;

	PIT_DisableInterrupts(config->base, config->pit_channel,
			      kPIT_TimerInterruptEnable);
	PIT_StopTimer(config->base, config->pit_channel);

	return 0;
}

static int mcux_pit_get_value(struct device *dev, u32_t *ticks)
{
	const struct mcux_pit_config *config = dev->config_info;

	*ticks = PIT_GetCurrentTimerCount(config->base, config->pit_channel);

	return 0;
}

static int mcux_pit_set_top_value(struct device *dev,
				  const struct counter_top_cfg *cfg)
{
	const struct mcux_pit_config *config = dev->config_info;
	struct mcux_pit_data *data = dev->driver_data;
	pit_chnl_t channel = config->pit_channel;

	if (cfg->ticks == 0) {
		return -EINVAL;
	}

	data->top_callback = cfg->callback;
	data->top_user_data = cfg->user_data;

	if (config->base->CHANNEL[channel].TCTRL & PIT_TCTRL_TEN_MASK) {
		/* Timer already enabled, check flags before resetting */
		if (cfg->flags & COUNTER_TOP_CFG_DONT_RESET) {
			return -ENOTSUP;
		}
		PIT_StopTimer(config->base, channel);
		PIT_SetTimerPeriod(config->base, channel, cfg->ticks);
		PIT_StartTimer(config->base, channel);
	} else {
		PIT_SetTimerPeriod(config->base, channel, cfg->ticks);
	}

	return 0;
}

static u32_t mcux_pit_get_pending_int(struct device *dev)
{
	const struct mcux_pit_config *config = dev->config_info;
	u32_t mask = PIT_TFLG_TIF_MASK;
	u32_t flags;

	flags = PIT_GetStatusFlags(config->base, config->pit_channel);

	return ((flags & mask) == mask);
}

static u32_t mcux_pit_get_top_value(struct device *dev)
{
	const struct mcux_pit_config *config = dev->config_info;
	pit_chnl_t channel = config->pit_channel;

	return config->base->CHANNEL[channel].LDVAL;
}

static u32_t mcux_pit_get_max_relative_alarm(struct device *dev)
{
	ARG_UNUSED(dev);

	/* Alarms not supported */
	return 0;
}

static void mcux_pit_isr(void *arg)
{
	struct device *dev = arg;
	const struct mcux_pit_config *config = dev->config_info;
	struct mcux_pit_data *data = dev->driver_data;
	u32_t flags;

	flags = PIT_GetStatusFlags(config->base, config->pit_channel);
	PIT_ClearStatusFlags(config->base, config->pit_channel, flags);

	if (data->top_callback) {
		data->top_callback(dev, data->top_user_data);
	}
}

static int mcux_pit_init(struct device *dev)
{
	struct mcux_pit_config *config = (struct mcux_pit_config *)dev->config_info;
	pit_config_t pit_config;

	PIT_GetDefaultConfig(&pit_config);
	pit_config.enableRunInDebug = config->enableRunInDebug;

	PIT_Init(config->base, &pit_config);

	config->irq_config_func(dev);

	/*hard code to 1s*/
	PIT_SetTimerPeriod(config->base, config->pit_channel,
		USEC_TO_COUNT(1000000U, CLOCK_GetFreq(kCLOCK_BusClk)));
        config->info.freq = CLOCK_GetFreq(kCLOCK_BusClk);

	return 0;
}

static const struct counter_driver_api mcux_pit_driver_api = {
	.start = mcux_pit_start,
	.stop = mcux_pit_stop,
	.get_value = mcux_pit_get_value,
	.set_top_value = mcux_pit_set_top_value,
	.get_pending_int = mcux_pit_get_pending_int,
	.get_top_value = mcux_pit_get_top_value,
	.get_max_relative_alarm = mcux_pit_get_max_relative_alarm,
};

/*
 * This driver is single-instance. If the devicetree contains multiple
 * instances, this will fail and the driver needs to be revisited.
 */
BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) <= 1,
	     "unsupported pit instance");

#if DT_NODE_HAS_STATUS(DT_DRV_INST(0), okay)
static struct mcux_pit_data mcux_pit_data_0;

static void mcux_pit_irq_config_0(struct device *dev);

static struct mcux_pit_config mcux_pit_config_0 = {
	.info = {
		.max_top_value = UINT32_MAX,
		.channels = 0,
	},
	.base = (PIT_Type *)DT_INST_REG_ADDR(0),
	.pit_channel = CONFIG_PIT_CHANNEL,
	.irq_config_func = mcux_pit_irq_config_0,
};

DEVICE_AND_API_INIT(mcux_pit_0, DT_INST_LABEL(0), &mcux_pit_init,
		    &mcux_pit_data_0, &mcux_pit_config_0, POST_KERNEL,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &mcux_pit_driver_api);

static void mcux_pit_irq_config_0(struct device *dev)
{
	IRQ_CONNECT(DT_INST_IRQ_BY_IDX(0, 0, irq),
				DT_INST_IRQ_BY_IDX(0, 0, priority),
				mcux_pit_isr, DEVICE_GET(mcux_pit_0), 0);
	irq_enable(DT_INST_IRQ_BY_IDX(0, 0, irq));
	IRQ_CONNECT(DT_INST_IRQ_BY_IDX(0, 1, irq),
				DT_INST_IRQ_BY_IDX(0, 1, priority),
				mcux_pit_isr, DEVICE_GET(mcux_pit_0), 0);
	irq_enable(DT_INST_IRQ_BY_IDX(0, 1, irq));
	IRQ_CONNECT(DT_INST_IRQ_BY_IDX(0, 2, irq),
				DT_INST_IRQ_BY_IDX(0, 2, priority),
				mcux_pit_isr, DEVICE_GET(mcux_pit_0), 0);
	irq_enable(DT_INST_IRQ_BY_IDX(0, 2, irq));
	IRQ_CONNECT(DT_INST_IRQ_BY_IDX(0, 3, irq),
				DT_INST_IRQ_BY_IDX(0, 3, priority),
				mcux_pit_isr, DEVICE_GET(mcux_pit_0), 0);
	irq_enable(DT_INST_IRQ_BY_IDX(0, 3, irq));
}
#endif /* DT_NODE_HAS_STATUS(DT_DRV_INST(0), okay) */
