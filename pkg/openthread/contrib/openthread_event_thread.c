/*
 * Copyright (C) 2018 UC Berkeley
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @{
 * @ingroup     BETS
 * @file
 * @brief       Implementation of OpenThread preevent thread
 *
 * @author      Hyung-Sin Kim <hs.kim@cs.berkeley.edu>
 * @}
 */

#include "ot.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#define OPENTHREAD_EVENT_QUEUE_LEN (2)
static msg_t _queue[OPENTHREAD_EVENT_QUEUE_LEN];
static kernel_pid_t _ot_event_pid;

/* get OpenThread Event Thread pid */
kernel_pid_t openthread_get_event_pid(void) {
    return _ot_event_pid;
}

/* OpenThread Event Thread
 * This thread receives event messages directly from interrupt handlers (timer and radio) and
 * delivers them to OpenThread Main Thread. Even though we have OpenThread Main Thread to
 * process events, this additional thread is necessary for safe operation.
 *
 * Note that RIOT's message delivery from an interrupt handler to a thread is failed when
 * the thread's msg_queue is full, while that from a thread to another thread is safe even when
 * the receiving thread's msg_queue is full thanks to backpressure.
 *
 * Thus, sending all types of event messages directly from interrupt handlers to Main Thread
 * can miss important events. Specifically, when the radio receives many packets and
 * Main Thread's msg_queue is full of received packets, timer or tx_complete event can be
 * dropped, resulting in malfunction.
 *
 * Given that this thread manages urgent requests and does a very simple job, it preempts both
 * OpenThread Main Thread and OpenThread Task Thread.
 *
 * The msg_queue size of this thread can be bounded by the number of event types it handles, '2'.
 * 1) OpenThread exposes only one timer to RIOT at a time.
 * 2) OpenThread does not send a packet before receiving tx_complete event for the previous packet.
**/
static void *_openthread_event_thread(void *arg) {
    _ot_event_pid = thread_getpid();

    msg_init_queue(_queue, OPENTHREAD_EVENT_QUEUE_LEN);
    msg_t msg;

    DEBUG("ot_event: START!\n");

    while (1) {
        msg_receive(&msg);
        switch (msg.type) {
            case OPENTHREAD_MILLITIMER_MSG_TYPE_EVENT:
                /* Tell event_thread a time event was received */
                DEBUG("ot_event: OPENTHREAD_MILLITIMER_MSG_TYPE_EVENT received\n");
                msg.type = OPENTHREAD_MILLITIMER_MSG_TYPE_EVENT;
                msg_send(&msg, openthread_get_main_pid());
                break;
            case OPENTHREAD_SERIAL_MSG_TYPE_EVENT:
                /* Tell OpenThread about the reception of a CLI command */
                DEBUG("\not_event: OPENTHREAD_SERIAL_MSG_TYPE received\n");
                msg.type = OPENTHREAD_SERIAL_MSG_TYPE_EVENT;
                msg_send(&msg, openthread_get_main_pid());
                break;
        }
    }

    return NULL;
}

/* starts OpenThread Event Thread */
int openthread_event_init(char *stack, int stacksize, char priority, const char *name) {

    _ot_event_pid = thread_create(stack, stacksize, priority, THREAD_CREATE_STACKTEST,
                         _openthread_event_thread, NULL, name);

    if (_ot_event_pid <= 0) {
        return -EINVAL;
    }

    return _ot_event_pid;
}
