/*
 * Copyright 2025 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 *
 * Single-thread system workqueue.
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>

struct k_work_q k_sys_work_q;

static int k_sys_work_q_init(void)
{
	/* In single-thread mode, we don't need to initialize a work queue */
	return 0;
}

SYS_INIT(k_sys_work_q_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
