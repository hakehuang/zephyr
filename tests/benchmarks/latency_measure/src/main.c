/*
 * Copyright (c) 2012-2015 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * @file
 * This file contains the main testing module that invokes all the tests.
 */

#include <timestamp.h>
#include "utils.h"
#include <tc_util.h>

#define STACK_SIZE (1024 + CONFIG_TEST_EXTRA_STACKSIZE)

uint32_t tm_off; /* time necessary to read the time */
int error_count; /* track number of errors */

extern void thread_switch_yield(void);
extern void int_to_thread(void);
extern void int_to_thread_evt(void);
extern void sema_test_signal(void);
extern void mutex_lock_unlock(void);
extern int coop_ctx_switch(void);
extern int sema_test(void);
extern int sema_context_switch(void);
extern int suspend_resume(void);

void test_thread(void *arg1, void *arg2, void *arg3)
{
	uint32_t freq;

	timing_init();

	bench_test_init();

	freq = timing_freq_get_mhz();

	TC_START("Time Measurement");
	TC_PRINT("Timing results: Clock frequency: %u MHz\n", freq);

	thread_switch_yield();

	coop_ctx_switch();
#if 0
	int_to_thread();

	int_to_thread_evt();

	suspend_resume();
#endif
	sema_test_signal();

#if 0
	sema_context_switch();
#endif
	mutex_lock_unlock();

	TC_END_REPORT(error_count);
}

K_THREAD_DEFINE(test_thread_id, STACK_SIZE, test_thread, NULL, NULL, NULL, K_PRIO_PREEMPT(10), 0, 0);

void main(void)
{
}
