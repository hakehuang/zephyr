/*
 * Copyright 2025 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file @brief Single-thread mutex kernel services
 *
 * Simplified mutex implementation for single-thread environments
 */

#include <zephyr/kernel.h>
#include <zephyr/kernel_structs.h>
#include <zephyr/toolchain.h>
#include <zephyr/sys/check.h>
#include <zephyr/tracing/tracing.h>

int z_impl_k_mutex_init(struct k_mutex *mutex)
{
	mutex->owner = NULL;
	mutex->lock_count = 0U;

	k_object_init(mutex);

	SYS_PORT_TRACING_OBJ_INIT(k_mutex, mutex, 0);

	return 0;
}

int z_impl_k_mutex_lock(struct k_mutex *mutex, k_timeout_t timeout)
{
	__ASSERT(!arch_is_in_isr(), "mutexes cannot be used inside ISRs");

	SYS_PORT_TRACING_OBJ_FUNC_ENTER(k_mutex, lock, mutex, timeout);

	if (mutex->lock_count == 0U || mutex->owner == _current) {
		mutex->owner = _current;
		mutex->lock_count++;
		SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_mutex, lock, mutex, timeout, 0);
		return 0;
	}

	if (K_TIMEOUT_EQ(timeout, K_NO_WAIT)) {
		SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_mutex, lock, mutex, timeout, -EBUSY);
		return -EBUSY;
	}

	/* In single-thread mode, we can't block */
	SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_mutex, lock, mutex, timeout, -EAGAIN);
	return -EAGAIN;
}

int z_impl_k_mutex_unlock(struct k_mutex *mutex)
{
	__ASSERT(!arch_is_in_isr(), "mutexes cannot be used inside ISRs");

	SYS_PORT_TRACING_OBJ_FUNC_ENTER(k_mutex, unlock, mutex);

	CHECKIF(mutex->owner == NULL) {
		SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_mutex, unlock, mutex, -EINVAL);
		return -EINVAL;
	}

	CHECKIF(mutex->owner != _current) {
		SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_mutex, unlock, mutex, -EPERM);
		return -EPERM;
	}

	__ASSERT_NO_MSG(mutex->lock_count > 0U);

	if (mutex->lock_count > 1U) {
		mutex->lock_count--;
	} else {
		mutex->lock_count = 0U;
		mutex->owner = NULL;
	}

	SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_mutex, unlock, mutex, 0);

	return 0;
}
