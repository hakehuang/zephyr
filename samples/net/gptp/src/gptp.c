/*
 * Copyright (c) 2020 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <logging/log.h>
LOG_MODULE_DECLARE(net_gptp_sample);

#include <zephyr.h>
#include <errno.h>
#include <stdlib.h>

#include <net/net_core.h>
#include <net/gptp.h>
#include <drivers/ptp_clock.h>

#include "ethernet/gptp/gptp_messages.h"
#include "ethernet/gptp/gptp_data_set.h"

/*USER BEGIN INCLUDES*/
#include <net/ptp_time.h>
#include <sys/printk.h>
#include <sys/util.h>
/*USER END INCLUDES*/

static int run_duration = CONFIG_NET_SAMPLE_RUN_DURATION;
static struct k_work_delayable stop_sample;
static struct k_sem quit_lock;

/*USER BEGIN VARIABLES*/

static void stop_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	k_sem_give(&quit_lock);
}

static int get_current_status(void)
{
	struct gptp_domain *domain;
	struct gptp_port_ds *port_ds;
	int ret, port;

	port = 1;

	domain = gptp_get_domain();

	ret = gptp_get_port_data(domain, port, &port_ds,
				 NULL, NULL, NULL, NULL);
	if (ret < 0) {
		LOG_WRN("Cannot get gPTP information for port %d (%d)",
			port, ret);
		return ret;
	}

	if (port != port_ds->port_id.port_number) {
		return -EINVAL;
	}

	switch (GPTP_GLOBAL_DS()->selected_role[port]) {
	case GPTP_PORT_INITIALIZING:
	case GPTP_PORT_FAULTY:
	case GPTP_PORT_DISABLED:
	case GPTP_PORT_LISTENING:
	case GPTP_PORT_PRE_MASTER:
	case GPTP_PORT_PASSIVE:
	case GPTP_PORT_UNCALIBRATED:
		return 0;
	case GPTP_PORT_MASTER:
		return 1;
	case GPTP_PORT_SLAVE:
		return 2;
	}

	return -1;
}

void init_testing(void)
{
	uint32_t uptime = k_uptime_get_32();
	int ret;
	const struct device *clk;	

	clk = device_get_binding("PTP_CLOCK");
	if (clk != NULL) {
		k_object_access_grant(clk, k_current_get());
	}

	if (run_duration == 0) {
		/* USER BEGIN MAIN.C*/
		while(1){
			//static struct net_ptp_time slave_time;
			//ptp_clock_get(clk, &slave_time);
			k_msleep(200); //sleep time in ms
		}
	}

	k_sem_init(&quit_lock, 0, K_SEM_MAX_LIMIT);

	k_work_init_delayable(&stop_sample, stop_handler);
	k_work_reschedule(&stop_sample, K_SECONDS(run_duration));

	k_sem_take(&quit_lock, K_FOREVER);

	LOG_INF("Stopping after %u seconds",
		(k_uptime_get_32() - uptime) / 1000);

	/* Try to figure out what is the sync state.
	 * Return:
	 *  <0 - configuration error
	 *   0 - not time sync
	 *   1 - we are MASTER
	 *   2 - we are SLAVE
	 */
	ret = get_current_status();

	exit(ret);
}
