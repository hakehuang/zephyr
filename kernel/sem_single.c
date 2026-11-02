/*
 * Copyright 2025 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 *
 * @brief Single-thread kernel semaphore object.
 */

#include <zephyr/kernel.h>
#include <zephyr/kernel_structs.h>
#include <zephyr/toolchain.h>
#include <zephyr/sys/check.h>
#include <zephyr/internal/syscall_handler.h>
#include <zephyr/tracing/tracing.h>

int z_impl_k_sem_init(struct k_sem *sem, unsigned int initial_count, unsigned int limit)
{
	CHECKIF(limit == 0U || initial_count > limit) {
		SYS_PORT_TRACING_OBJ_FUNC(k_sem, init, sem, -EINVAL);
		return -EINVAL;
	}

	sem->count = initial_count;
	sem->limit = limit;

	SYS_PORT_TRACING_OBJ_FUNC(k_sem, init, sem, 0);

	k_object_init(sem);
	return 0;
}

void z_impl_k_sem_give(struct k_sem *sem)
{
	SYS_PORT_TRACING_OBJ_FUNC_ENTER(k_sem, give, sem);

	if (sem->count < sem->limit) {
		sem->count++;
	}

	SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_sem, give, sem);
}

int z_impl_k_sem_take(struct k_sem *sem, k_timeout_t timeout)
{
	SYS_PORT_TRACING_OBJ_FUNC_ENTER(k_sem, take, sem, timeout);

	__ASSERT(((arch_is_in_isr() == false) ||
		  K_TIMEOUT_EQ(timeout, K_NO_WAIT)), "");


	if (sem->count > 0U) {
		sem->count--;
		SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_sem, take, sem, timeout, 0);
		return 0;
	}

	if (K_TIMEOUT_EQ(timeout, K_NO_WAIT)) {
		SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_sem, take, sem, timeout, -EBUSY);
		return -EBUSY;
	} else {
		k_sleep(timeout);
		if (sem->count > 0U) {
			sem->count--;
			SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_sem, take, sem, timeout, 0);
			return 0;	
		}
	}

	/* In single-thread mode, we can't block */
	SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_sem, take, sem, timeout, -EAGAIN);
	return -EAGAIN;
}

void z_impl_k_sem_reset(struct k_sem *sem)
{
	sem->count = 0;
	SYS_PORT_TRACING_OBJ_FUNC(k_sem, reset, sem);
}

#ifdef CONFIG_USERSPACE
static inline int z_vrfy_k_sem_init(struct k_sem *sem, unsigned int initial_count,
				    unsigned int limit)
{
	K_OOPS(K_SYSCALL_OBJ_INIT(sem, K_OBJ_SEM));
	return z_impl_k_sem_init(sem, initial_count, limit);
}
#include <zephyr/syscalls/k_sem_init_mrsh.c>

static inline void z_vrfy_k_sem_give(struct k_sem *sem)
{
	K_OOPS(K_SYSCALL_OBJ(sem, K_OBJ_SEM));
	z_impl_k_sem_give(sem);
}
#include <zephyr/syscalls/k_sem_give_mrsh.c>

static inline int z_vrfy_k_sem_take(struct k_sem *sem, k_timeout_t timeout)
{
	K_OOPS(K_SYSCALL_OBJ(sem, K_OBJ_SEM));
	return z_impl_k_sem_take(sem, timeout);
}
#include <zephyr/syscalls/k_sem_take_mrsh.c>

static inline void z_vrfy_k_sem_reset(struct k_sem *sem)
{
	K_OOPS(K_SYSCALL_OBJ(sem, K_OBJ_SEM));
	z_impl_k_sem_reset(sem);
}
#include <zephyr/syscalls/k_sem_reset_mrsh.c>

static inline unsigned int z_vrfy_k_sem_count_get(struct k_sem *sem)
{
	K_OOPS(K_SYSCALL_OBJ(sem, K_OBJ_SEM));
	return z_impl_k_sem_count_get(sem);
}
#include <zephyr/syscalls/k_sem_count_get_mrsh.c>
#endif /* CONFIG_USERSPACE */
