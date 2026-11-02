/*
 * Copyright 2025 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 *
 * @brief Single-thread dynamic-size QUEUE object.
 */

#include <zephyr/kernel.h>
#include <zephyr/kernel_structs.h>
#include <zephyr/toolchain.h>
#include <zephyr/sys/check.h>
#include <zephyr/internal/syscall_handler.h>
#include <kernel_internal.h>

struct alloc_node {
	sys_sfnode_t node;
	void *data;
};

void *z_queue_node_peek(sys_sfnode_t *node, bool needs_free)
{
	void *ret;

	if ((node != NULL) && (sys_sfnode_flags_get(node) != (uint8_t)0)) {
		struct alloc_node *anode = CONTAINER_OF(node, struct alloc_node, node);
		ret = anode->data;
		if (needs_free) {
			k_free(anode);
		}
	} else {
		ret = (void *)node;
	}

	return ret;
}

void z_impl_k_queue_init(struct k_queue *queue)
{
	sys_sflist_init(&queue->data_q);

	SYS_PORT_TRACING_OBJ_INIT(k_queue, queue);

	k_object_init(queue);
}

static int32_t queue_insert(struct k_queue *queue, void *prev, void *data, bool alloc,
			    bool is_append)
{
	if (alloc) {
		struct alloc_node *anode = k_malloc(sizeof(*anode));
		if (anode == NULL) {
			return -ENOMEM;
		}
		anode->data = data;
		sys_sfnode_init(&anode->node, 0x1);
		data = anode;
	} else {
		sys_sfnode_init(data, 0x0);
	}

	if (is_append) {
		sys_sflist_append(&queue->data_q, data);
	} else {
		sys_sflist_prepend(&queue->data_q, data);
	}

	return 0;
}

void k_queue_insert(struct k_queue *queue, void *prev, void *data)
{
	(void)queue_insert(queue, prev, data, false, false);
}

void k_queue_append(struct k_queue *queue, void *data)
{
	(void)queue_insert(queue, NULL, data, false, true);
}

void k_queue_prepend(struct k_queue *queue, void *data)
{
	(void)queue_insert(queue, NULL, data, false, false);
}

int32_t z_impl_k_queue_alloc_append(struct k_queue *queue, void *data)
{
	return queue_insert(queue, NULL, data, true, true);
}

int32_t z_impl_k_queue_alloc_prepend(struct k_queue *queue, void *data)
{
	return queue_insert(queue, NULL, data, true, false);
}

int k_queue_append_list(struct k_queue *queue, void *head, void *tail)
{
	if ((head == NULL) || (tail == NULL)) {
		return -EINVAL;
	}

	sys_sflist_append_list(&queue->data_q, head, tail);
	return 0;
}

int k_queue_merge_slist(struct k_queue *queue, sys_slist_t *list)
{
	if (sys_slist_is_empty(list)) {
		return -EINVAL;
	}

	int ret = k_queue_append_list(queue, list->head, list->tail);
	if (ret == 0) {
		sys_slist_init(list);
	}
	return ret;
}

void *z_impl_k_queue_get(struct k_queue *queue, k_timeout_t timeout)
{
	if (sys_sflist_is_empty(&queue->data_q)) {
		if (K_TIMEOUT_EQ(timeout, K_NO_WAIT)) {
			return NULL;
		}
		/* In single-thread mode, we can't block */
		return NULL;
	}

	sys_sfnode_t *node = sys_sflist_get(&queue->data_q);
	return z_queue_node_peek(node, true);
}

bool k_queue_remove(struct k_queue *queue, void *data)
{
	return sys_sflist_find_and_remove(&queue->data_q, (sys_sfnode_t *)data);
}

bool k_queue_unique_append(struct k_queue *queue, void *data)
{
	sys_sfnode_t *test;

	SYS_SFLIST_FOR_EACH_NODE(&queue->data_q, test) {
		if (test == (sys_sfnode_t *)data) {
			return false;
		}
	}

	k_queue_append(queue, data);
	return true;
}

void *z_impl_k_queue_peek_head(struct k_queue *queue)
{
	return z_queue_node_peek(sys_sflist_peek_head(&queue->data_q), false);
}

void *z_impl_k_queue_peek_tail(struct k_queue *queue)
{
	return z_queue_node_peek(sys_sflist_peek_tail(&queue->data_q), false);
}
