/*
 * Copyright (C) 2017 Fundacion Inria Chile
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
 * @brief       Implementation of OpenThread Main thread
 *
 * @author      Jose Ignacio Alamos <jialamos@uc.cl>
 * @author      Hyung-Sin Kim <hs.kim@cs.berkeley.edu>
 * @}
 */

#include "openthread/platform/alarm-milli.h"
#include "openthread/platform/alarm-micro.h"
#include "openthread/platform/uart.h"
#include "openthread/cli.h"
#include "openthread/tasklet.h"
#include "openthread/ip6.h"
#include "openthread/thread.h"
#include "openthread/instance.h"
#include "xtimer.h"
#include "ot.h"

#ifdef MODULE_OPENTHREAD_NCP_FTD
#include "openthread/ncp.h"
#include "openthread/commissioner.h"
extern void wdt_clear(void);
#endif

#ifdef MODULE_AT86RF2XX
#include "at86rf2xx.h"
#endif

#define ENABLE_DEBUG (0)
#include "debug.h"

#define OPENTHREAD_MAIN_QUEUE_LEN (16)
static msg_t _queue[OPENTHREAD_MAIN_QUEUE_LEN];
static kernel_pid_t _ot_main_pid;

static otInstance *sInstance;
static bool otTaskPending = false;

/* get OpenThread instance */
otInstance* openthread_get_instance(void) {
    return sInstance;
}

/* get OpenThread Main Thread pid */
kernel_pid_t openthread_get_main_pid(void) {
    return _ot_main_pid;
}

/* OpenThread will call this when switching state from empty tasklet to non-empty tasklet. */
void otTaskletsSignalPending(otInstance *aInstance) {
    (void) aInstance;
    if (thread_getpid() != openthread_get_main_pid() && !otTaskPending) {
        otTaskPending = true;
        msg_t msg;
        msg.type = OPENTHREAD_TASK_MSG_TYPE_EVENT;
        msg_send(&msg, openthread_get_main_pid());
    }
}

/* OpenThread Main Thread
 * This thread processes all events by calling proper functions of OpenThread.
 * It is preempted by OpenThread Event Thread.
**/
static void *_openthread_main_thread(void *arg) {
    _ot_main_pid = thread_getpid();

    msg_init_queue(_queue, OPENTHREAD_MAIN_QUEUE_LEN);
    msg_t msg, reply;

    ot_job_t *job;
    serial_msg_t* serialBuffer;

    DEBUG("ot_main: START!\n");
    /* Wait until other threads are initialized */
    xtimer_usleep(100000);

    /* Init OpenThread instance */
    sInstance = otInstanceInitSingle();
    DEBUG("OT-instance setting is OK\n");

    /* Init default parameters */
    otPanId panid = OPENTHREAD_PANID;
    uint8_t channel = OPENTHREAD_CHANNEL;
    otLinkSetPanId(sInstance, panid);
    otLinkSetChannel(sInstance, channel);

#if defined(MODULE_OPENTHREAD_CLI_FTD) || defined(MODULE_OPENTHREAD_CLI_MTD)
    otCliUartInit(sInstance);
    DEBUG("OT-UART initialization is OK\n");
    /* Bring up the IPv6 interface  */
    otIp6SetEnabled(sInstance, true);
    DEBUG("OT-IPv6 setting is OK\n");
    /* Start Thread operation */
    otThreadSetEnabled(sInstance, true);
    DEBUG("OT-FTD/MTD initialization is OK\n");
#endif

#ifdef MODULE_OPENTHREAD_NCP_FTD
    otNcpInit(sInstance);
    DEBUG("OT-NCP initialization is OK\n");
    otCommissionerStart(sInstance);
    DEBUG("OT-Commisioner initialization is OK\n");
#endif

#if OPENTHREAD_ENABLE_DIAG
    diagInit(sInstance);
#endif

    while (1) {
        while(otTaskletsArePending(openthread_get_instance())) {
            otTaskletsProcess(openthread_get_instance());
        }

        openthread_coarse_unlock_buffer_mutex();
        msg_receive(&msg);
        openthread_coarse_lock_buffer_mutex();

        switch (msg.type) {
            case OPENTHREAD_TASK_MSG_TYPE_EVENT:
                /* Process OpenThread tasks (pre-processing a sending packet) */
                DEBUG("\not_main: OPENTHREAD_TASK_MSG_TYPE_EVENT received\n");
                otTaskPending = false;
                break;
            case OPENTHREAD_NETDEV_MSG_TYPE_EVENT:
                /* Received an event from radio driver */
                DEBUG("\not_main: OPENTHREAD_NETDEV_MSG_TYPE_EVENT received\n");
                /* Wait until the task thread finishes accessing the shared resoure (radio) */
                openthread_get_netdev()->driver->isr(openthread_get_netdev());
#ifdef MODULE_OPENTHREAD_FTD
                if (msg.content.value) {
                    unsigned state = irq_disable();
                    ((at86rf2xx_t *)openthread_get_netdev())->pending_irq--;
                    irq_restore(state);
                }
#endif
                break;
            case OPENTHREAD_LINK_RETRY_TIMEOUT:
                DEBUG("\not_main: OPENTHREAD_LINK_RETRY_TIMEOUT\n");
                sent_pkt(openthread_get_instance(), NETDEV_EVENT_TX_NOACK);
                break;
            case OPENTHREAD_NETDEV_MSG_TYPE_RADIO_BUSY:
                /* Radio is busy */
                DEBUG("\not_main: OPENTHREAD_NETDEV_MSG_TYPE_RADIO_BUSY received\n");
                sent_pkt(openthread_get_instance(), NETDEV_EVENT_TX_MEDIUM_BUSY);
                break;
            case OPENTHREAD_MILLITIMER_MSG_TYPE_EVENT:
                /* Tell OpenThread a millisec time event was received */
                DEBUG("\not_main: OPENTHREAD_MILLITIMER_MSG_TYPE_EVENT received\n");
                otPlatAlarmMilliFired(sInstance);
                break;
#ifdef MODULE_OPENTHREAD_FTD
            case OPENTHREAD_MICROTIMER_MSG_TYPE_EVENT:
                /* Tell OpenThread a microsec time event was received (CSMA timer)
                 * It checks the current time and executes callback functions of
                 * only expired timers. */
                DEBUG("\not_main: OPENTHREAD_MICROTIMER_MSG_TYPE_EVENT received\n");
                otPlatAlarmMicroFired(openthread_get_instance());
                break;
#endif
            case OPENTHREAD_SERIAL_MSG_TYPE_EVENT:
                /* Tell OpenThread about the reception of a CLI command */
                DEBUG("\not_main: OPENTHREAD_SERIAL_MSG_TYPE received\n");
#ifdef MODULE_OPENTHREAD_NCP_FTD
                wdt_clear();
#endif
                serialBuffer = (serial_msg_t*)msg.content.ptr;
                DEBUG("%s", serialBuffer->buf);
                otPlatUartReceived((uint8_t*) serialBuffer->buf,serialBuffer->length);
                serialBuffer->serial_buffer_status = OPENTHREAD_SERIAL_BUFFER_STATUS_FREE;
                break;
            case OPENTHREAD_JOB_MSG_TYPE_EVENT:
                DEBUG("\not_main: OPENTHREAD_JOB_MSG_TYPE_EVENT received\n");
                job = msg.content.ptr;
                reply.content.value = ot_exec_command(sInstance, job->command, job->arg, job->answer);
                msg_reply(&msg, &reply);
                break;
        }
    }

    return NULL;
}

/* starts OpenThread Main thread */
int openthread_main_init(char *stack, int stacksize, char priority, const char *name) {

    _ot_main_pid = thread_create(stack, stacksize, priority, THREAD_CREATE_STACKTEST,
                         _openthread_main_thread, NULL, name);

    if (_ot_main_pid <= 0) {
        return -EINVAL;
    }

    return _ot_main_pid;
}
