/*
 * Copyright 2025 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/kernel_structs.h>
#include <zephyr/kernel_structs.h>
#include <zephyr/toolchain.h>
#include <zephyr/internal/syscall_handler.h>
#include <zephyr/init.h>

/* Single-thread scheduler implementation */
static struct k_thread _dummy_thread;


void z_dummy_thread_init(struct k_thread *thread)
{
	thread->base.prio = 0;
}


void z_impl_k_thread_priority_set(k_tid_t thread, int prio)
{
	SYS_PORT_TRACING_OBJ_FUNC(k_thread, sched_priority_set, thread, prio);
	thread->base.prio = prio;
}

#ifdef CONFIG_USERSPACE
static inline void z_vrfy_k_thread_priority_set(k_tid_t thread, int prio)
{
	K_OOPS(K_SYSCALL_OBJ(thread, K_OBJ_THREAD));
	z_impl_k_thread_priority_set(thread, prio);
}
#include <zephyr/syscalls/k_thread_priority_set_mrsh.c>
#endif /* CONFIG_USERSPACE */

void z_impl_k_thread_suspend(k_tid_t thread)
{
	SYS_PORT_TRACING_OBJ_FUNC_ENTER(k_thread, suspend, thread);
	SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_thread, suspend, thread);
}

#ifdef CONFIG_USERSPACE
static inline void z_vrfy_k_thread_suspend(k_tid_t thread)
{
	K_OOPS(K_SYSCALL_OBJ(thread, K_OBJ_THREAD));
	z_impl_k_thread_suspend(thread);
}
#include <zephyr/syscalls/k_thread_suspend_mrsh.c>
#endif /* CONFIG_USERSPACE */

void z_impl_k_thread_resume(k_tid_t thread)
{
	SYS_PORT_TRACING_OBJ_FUNC_ENTER(k_thread, resume, thread);
	SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_thread, resume, thread);
}

#ifdef CONFIG_USERSPACE
static inline void z_vrfy_k_thread_resume(k_tid_t thread)
{
	K_OOPS(K_SYSCALL_OBJ(thread, K_OBJ_THREAD));
	z_impl_k_thread_resume(thread);
}
#include <zephyr/syscalls/k_thread_resume_mrsh.c>
#endif /* CONFIG_USERSPACE */

void z_impl_k_yield(void)
{
	SYS_PORT_TRACING_FUNC(k_thread, yield);
}

#ifdef CONFIG_USERSPACE
static inline void z_vrfy_k_yield(void)
{
	z_impl_k_yield();
}
#include <zephyr/syscalls/k_yield_mrsh.c>
#endif /* CONFIG_USERSPACE */

/* k_sleep implementation is provided in nothread.c */
#ifdef CONFIG_USERSPACE
static inline int32_t z_vrfy_k_sleep(k_timeout_t timeout)
{
	return z_impl_k_sleep(timeout);
}
#include <zephyr/syscalls/k_sleep_mrsh.c>
#endif /* CONFIG_USERSPACE */

int32_t z_impl_k_usleep(int32_t us)
{
	SYS_PORT_TRACING_FUNC_ENTER(k_thread, usleep, us);
	SYS_PORT_TRACING_FUNC_EXIT(k_thread, usleep, us, 0);
	return 0;
}

#ifdef CONFIG_USERSPACE
static inline int32_t z_vrfy_k_usleep(int32_t us)
{
	return z_impl_k_usleep(us);
}
#include <zephyr/syscalls/k_usleep_mrsh.c>
#endif /* CONFIG_USERSPACE */

void z_impl_k_wakeup(k_tid_t thread)
{
	SYS_PORT_TRACING_OBJ_FUNC(k_thread, wakeup, thread);
}

#ifdef CONFIG_USERSPACE
static inline void z_vrfy_k_wakeup(k_tid_t thread)
{
	K_OOPS(K_SYSCALL_OBJ(thread, K_OBJ_THREAD));
	z_impl_k_wakeup(thread);
}
#include <zephyr/syscalls/k_wakeup_mrsh.c>
#endif /* CONFIG_USERSPACE */

k_tid_t z_impl_k_sched_current_thread_query(void)
{
	return _current;
}

#ifdef CONFIG_USERSPACE
static inline k_tid_t z_vrfy_k_sched_current_thread_query(void)
{
	return z_impl_k_sched_current_thread_query();
}
#include <zephyr/syscalls/k_sched_current_thread_query_mrsh.c>
#endif /* CONFIG_USERSPACE */

int z_impl_k_thread_join(struct k_thread *thread, k_timeout_t timeout)
{
	SYS_PORT_TRACING_OBJ_FUNC_ENTER(k_thread, join, thread, timeout);
	SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_thread, join, thread, timeout, 0);
	return 0;
}

#ifdef CONFIG_USERSPACE
static inline int z_vrfy_k_thread_join(struct k_thread *thread, k_timeout_t timeout)
{
	return z_impl_k_thread_join(thread, timeout);
}
#include <zephyr/syscalls/k_thread_join_mrsh.c>
#endif /* CONFIG_USERSPACE */

void z_sched_init(void)
{
	/* Initialize dummy thread structure */
	z_dummy_thread_init(&_dummy_thread);
}

#if CONFIG_POLL
void z_unpend_thread(struct k_thread *thread)
{
	ARG_UNUSED(thread);

	return;
}

void z_reschedule(struct k_spinlock *lock, k_spinlock_key_t key)
{
	ARG_UNUSED(lock);
	ARG_UNUSED(key);
}

void z_ready_thread(struct k_thread *thread)
{
	ARG_UNUSED(thread);
}
#endif