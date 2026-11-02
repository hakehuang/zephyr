/*
 * Copyright 2025 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/dlist.h>
#include <zephyr/sys/slist.h>
#include <zephyr/sys/util.h>
#include <zephyr/tracing/tracing.h>

/* Single-thread work queue implementation */

/* System work queue - not actually used in single-thread mode */
extern struct k_work_q k_sys_work_q;

/* Simplified work initialization */
void k_work_init(struct k_work *work, k_work_handler_t handler)
{
	__ASSERT_NO_MSG(work != NULL);
	__ASSERT_NO_MSG(handler != NULL);

	*work = (struct k_work){
		.handler = handler,
		.flags = 0,
	};

	SYS_PORT_TRACING_OBJ_INIT(k_work, work);
}

/* Submit work to a queue but do not yield the current thread.
 *
 * Intended for internal use.
 *
 * See also submit_to_queue_locked().
 *
 * @param queuep pointer to a queue reference.
 * @param work the work structure to be submitted
 *
 * @retval see submit_to_queue_locked()
 */
int z_work_submit_to_queue(struct k_work_q *queue,
		  struct k_work *work)
{
	__ASSERT_NO_MSG(work != NULL);
	__ASSERT_NO_MSG(work->handler != NULL);

	work->queue = queue;

	return 0;
}

/* Simplified work submission - executes immediately */
int k_work_submit_to_queue(struct k_work_q *queue, struct k_work *work)
{
	__ASSERT_NO_MSG(queue != NULL);
	__ASSERT_NO_MSG(work != NULL);
	__ASSERT_NO_MSG(work->handler != NULL);

	SYS_PORT_TRACING_OBJ_FUNC_ENTER(k_work, submit_to_queue, queue, work);

	/* Execute handler immediately */
	work->handler(work);

	SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_work, submit_to_queue, queue, work, 0);

	return 0;
}

/* Simplified work submission to system queue */
int k_work_submit(struct k_work *work)
{
	return k_work_submit_to_queue(&k_sys_work_q, work);
}

/* Simplified work initialization for delayable work */
void k_work_init_delayable(struct k_work_delayable *dwork,
				  void (*handler)(struct k_work *))
{
	__ASSERT_NO_MSG(dwork != NULL);
	__ASSERT_NO_MSG(handler != NULL);

	*dwork = (struct k_work_delayable){
		.work =
			{
				.handler = handler,
				.flags = K_WORK_DELAYABLE,
			},
	};

	SYS_PORT_TRACING_OBJ_INIT(k_work_delayable, dwork);
}

/* Simplified work scheduling - executes immediately */
int k_work_schedule_for_queue(struct k_work_q *queue,
				  struct k_work_delayable *dwork,
				  k_timeout_t delay)
{
	__ASSERT_NO_MSG(queue != NULL);
	__ASSERT_NO_MSG(dwork != NULL);

	SYS_PORT_TRACING_OBJ_FUNC_ENTER(k_work, schedule_for_queue, queue, dwork, delay);

	/* Execute handler immediately */
	dwork->work.handler(&dwork->work);

	SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_work, schedule_for_queue, queue, dwork, delay, 0);

	return 0;
}

/* Simplified work scheduling to system queue */
int k_work_schedule(struct k_work_delayable *dwork,
			   k_timeout_t delay)
{
	return k_work_schedule_for_queue(&k_sys_work_q, dwork, delay);
}

/* Work queue initialization - no-op in single-thread mode */
void k_work_queue_init(struct k_work_q *queue)
{
	__ASSERT_NO_MSG(queue != NULL);
}

/* System work queue initialization - no-op in single-thread mode */
int k_sys_work_q_init(void)
{
	return 0;
}
