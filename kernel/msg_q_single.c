/*
 * Copyright 2025 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Single-thread message queues.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <string.h>
#include <zephyr/sys/check.h>
#include <zephyr/sys/math_extras.h>

void k_msgq_init(struct k_msgq *msgq, char *buffer, size_t msg_size, uint32_t max_msgs)
{
	msgq->msg_size = msg_size;
	msgq->max_msgs = max_msgs;
	msgq->buffer_start = buffer;
	msgq->buffer_end = buffer + (max_msgs * msg_size);
	msgq->read_ptr = buffer;
	msgq->write_ptr = buffer;
	msgq->used_msgs = 0;
	msgq->flags = 0;

	k_object_init(msgq);
}

int z_impl_k_msgq_put(struct k_msgq *msgq, const void *data, k_timeout_t timeout)
{
	if (msgq->used_msgs >= msgq->max_msgs) {
		if (K_TIMEOUT_EQ(timeout, K_NO_WAIT)) {
			return -ENOMSG;
		}
		/* In single-thread mode, we can't block */
		return -EAGAIN;
	}

	/* Put message in queue */
	(void)memcpy(msgq->write_ptr, (char *)data, msgq->msg_size);
	msgq->write_ptr += msgq->msg_size;
	if (msgq->write_ptr == msgq->buffer_end) {
		msgq->write_ptr = msgq->buffer_start;
	}
	msgq->used_msgs++;

	return 0;
}

int z_impl_k_msgq_get(struct k_msgq *msgq, void *data, k_timeout_t timeout)
{
	if (msgq->used_msgs == 0) {
		if (K_TIMEOUT_EQ(timeout, K_NO_WAIT)) {
			return -ENOMSG;
		}
		/* In single-thread mode, we can't block */
		return -EAGAIN;
	}

	/* Get message from queue */
	(void)memcpy((char *)data, msgq->read_ptr, msgq->msg_size);
	msgq->read_ptr += msgq->msg_size;
	if (msgq->read_ptr == msgq->buffer_end) {
		msgq->read_ptr = msgq->buffer_start;
	}
	msgq->used_msgs--;

	return 0;
}

void z_impl_k_msgq_purge(struct k_msgq *msgq)
{
	msgq->used_msgs = 0;
	msgq->read_ptr = msgq->write_ptr;
}

void z_impl_k_msgq_get_attrs(struct k_msgq *msgq, struct k_msgq_attrs *attrs)
{
	attrs->msg_size = msgq->msg_size;
	attrs->max_msgs = msgq->max_msgs;
	attrs->used_msgs = msgq->used_msgs;
}

int z_impl_k_msgq_peek(struct k_msgq *msgq, void *data)
{
	if (msgq->used_msgs == 0) {
		return -ENOMSG;
	}

	(void)memcpy((char *)data, msgq->read_ptr, msgq->msg_size);
	return 0;
}

int z_impl_k_msgq_peek_at(struct k_msgq *msgq, void *data, uint32_t idx)
{
	uint32_t bytes_to_end;
	uint32_t byte_offset;
	char *start_addr;

	if (msgq->used_msgs <= idx) {
		return -ENOMSG;
	}

	bytes_to_end = (msgq->buffer_end - msgq->read_ptr);
	byte_offset = idx * msgq->msg_size;
	start_addr = msgq->read_ptr;

	if (bytes_to_end <= byte_offset) {
		byte_offset -= bytes_to_end;
		start_addr = msgq->buffer_start;
	}

	(void)memcpy(data, start_addr + byte_offset, msgq->msg_size);
	return 0;
}

int k_msgq_cleanup(struct k_msgq *msgq)
{
	if ((msgq->flags & K_MSGQ_FLAG_ALLOC) != 0U) {
		k_free(msgq->buffer_start);
		msgq->flags &= ~K_MSGQ_FLAG_ALLOC;
	}

	return 0;
}

int z_impl_k_msgq_alloc_init(struct k_msgq *msgq, size_t msg_size, uint32_t max_msgs)
{
	void *buffer;
	int ret;
	size_t total_size;

	if (size_mul_overflow(msg_size, max_msgs, &total_size)) {
		ret = -EINVAL;
	} else {
		buffer = k_malloc(total_size);
		if (buffer != NULL) {
			k_msgq_init(msgq, buffer, msg_size, max_msgs);
			msgq->flags = K_MSGQ_FLAG_ALLOC;
			ret = 0;
		} else {
			ret = -ENOMEM;
		}
	}

	return ret;
}
