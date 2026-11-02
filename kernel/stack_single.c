/*
 * Copyright 2025 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Single-thread fixed-size stack object
 */

#include <zephyr/kernel.h>
#include <zephyr/kernel_structs.h>
#include <zephyr/toolchain.h>
#include <zephyr/sys/check.h>
#include <zephyr/internal/syscall_handler.h>
#include <kernel_internal.h>

void k_stack_init(struct k_stack *stack, stack_data_t *buffer, uint32_t num_entries)
{
	stack->next = buffer;
	stack->base = buffer;
	stack->top = stack->base + num_entries;
	stack->flags = 0;

	SYS_PORT_TRACING_OBJ_INIT(k_stack, stack);
	k_object_init(stack);
}

int32_t z_impl_k_stack_alloc_init(struct k_stack *stack, uint32_t num_entries)
{
	void *buffer;

	SYS_PORT_TRACING_OBJ_FUNC_ENTER(k_stack, alloc_init, stack);

	buffer = k_malloc(num_entries * sizeof(stack_data_t));
	if (buffer != NULL) {
		k_stack_init(stack, buffer, num_entries);
		stack->flags = K_STACK_FLAG_ALLOC;
		SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_stack, alloc_init, stack, 0);
		return 0;
	}

	SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_stack, alloc_init, stack, -ENOMEM);
	return -ENOMEM;
}

int k_stack_cleanup(struct k_stack *stack)
{
	SYS_PORT_TRACING_OBJ_FUNC_ENTER(k_stack, cleanup, stack);

	if ((stack->flags & K_STACK_FLAG_ALLOC) != (uint8_t)0) {
		k_free(stack->base);
		stack->base = NULL;
		stack->flags &= ~K_STACK_FLAG_ALLOC;
	}

	SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_stack, cleanup, stack, 0);
	return 0;
}

int z_impl_k_stack_push(struct k_stack *stack, stack_data_t data)
{
	SYS_PORT_TRACING_OBJ_FUNC_ENTER(k_stack, push, stack);

	if (stack->next == stack->top) {
		SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_stack, push, stack, -ENOMEM);
		return -ENOMEM;
	}

	*(stack->next) = data;
	stack->next++;

	SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_stack, push, stack, 0);
	return 0;
}

int z_impl_k_stack_pop(struct k_stack *stack, stack_data_t *data, k_timeout_t timeout)
{
	SYS_PORT_TRACING_OBJ_FUNC_ENTER(k_stack, pop, stack, timeout);

	if (stack->next > stack->base) {
		stack->next--;
		*data = *(stack->next);
		SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_stack, pop, stack, timeout, 0);
		return 0;
	}

	if (K_TIMEOUT_EQ(timeout, K_NO_WAIT)) {
		SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_stack, pop, stack, timeout, -EBUSY);
		return -EBUSY;
	}

	/* In single-thread mode, we can't block */
	SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_stack, pop, stack, timeout, -EAGAIN);
	return -EAGAIN;
}

#ifdef CONFIG_USERSPACE
static inline int32_t z_vrfy_k_stack_alloc_init(struct k_stack *stack, uint32_t num_entries)
{
	size_t total_size;

	K_OOPS(K_SYSCALL_OBJ_NEVER_INIT(stack, K_OBJ_STACK));
	K_OOPS(K_SYSCALL_VERIFY(num_entries > 0));
	K_OOPS(K_SYSCALL_VERIFY(
		!size_mul_overflow(num_entries, sizeof(stack_data_t), &total_size)));
	return z_impl_k_stack_alloc_init(stack, num_entries);
}
#include <zephyr/syscalls/k_stack_alloc_init_mrsh.c>

static inline int z_vrfy_k_stack_push(struct k_stack *stack, stack_data_t data)
{
	K_OOPS(K_SYSCALL_OBJ(stack, K_OBJ_STACK));
	return z_impl_k_stack_push(stack, data);
}
#include <zephyr/syscalls/k_stack_push_mrsh.c>

static inline int z_vrfy_k_stack_pop(struct k_stack *stack, stack_data_t *data, k_timeout_t timeout)
{
	K_OOPS(K_SYSCALL_OBJ(stack, K_OBJ_STACK));
	K_OOPS(K_SYSCALL_MEMORY_WRITE(data, sizeof(stack_data_t)));
	return z_impl_k_stack_pop(stack, data, timeout);
}
#include <zephyr/syscalls/k_stack_pop_mrsh.c>
#endif /* CONFIG_USERSPACE */
