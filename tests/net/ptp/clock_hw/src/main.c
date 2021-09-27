/* main.c - Application main entry point */

/*
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define NET_LOG_LEVEL CONFIG_NET_L2_ETHERNET_LOG_LEVEL

#include <logging/log.h>
LOG_MODULE_REGISTER(net_test, NET_LOG_LEVEL);

#include <zephyr/types.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/printk.h>
#include <linker/sections.h>

#include <ztest.h>

#include <drivers/ptp_clock.h>
#include <net/ptp_time.h>

#include <net/ethernet.h>
#include <net/buf.h>
#include <net/net_ip.h>
#include <net/net_l2.h>

#include <random/rand32.h>

#define NET_LOG_ENABLED 1
#include "net_private.h"

#if NET_LOG_LEVEL >= LOG_LEVEL_DBG
#define DBG(fmt, ...) printk(fmt, ##__VA_ARGS__)
#else
#define DBG(fmt, ...)
#endif

#define MAX_NUM_INTERFACES 3

static ZTEST_BMEM struct net_ptp_time tm;

static uint64_t timestamp_to_nsec(struct net_ptp_time *ts)
{
	if (!ts) {
		return 0;
	}

	return (ts->second * NSEC_PER_SEC) + ts->nanosecond;
}



void test_main(void)
{
	const struct device *clk;

	clk = device_get_binding("PTP_CLOCK");
	if (clk != NULL) {
		k_object_access_grant(clk, k_current_get());
	}
	while(1) {
		(void)memset(&tm, 0, sizeof(tm));
		ptp_clock_get(clk, &tm);
		printk("now time is %llu.%u\n", tm.second, tm.nanosecond);
		k_msleep(100);
	
	}
}
