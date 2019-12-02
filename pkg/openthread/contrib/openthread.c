/*
 * Copyright (C) 2017 Fundacion Inria Chile
 * Copyright (C) 2017 UC Berkeley
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @{
 * @ingroup     net
 * @file
 * @brief       Implementation of OpenThread main functions
 *
 * @author      Jose Ignacio Alamos <jialamos@uc.cl>
 * @author      Hyung-Sin Kim <hs.kim@cs.berkeley.edu>
 * @}
 */

#include <assert.h>

#include "openthread/platform/uart.h"
#include "ot.h"
#include "ot_hooks.h"
#include "random.h"
#include "xtimer.h"
#include "irq.h"

#ifdef MODULE_AT86RF2XX
#include "at86rf2xx.h"
#include "at86rf2xx_params.h"
#endif

#define ENABLE_DEBUG (1)
#include "debug.h"

static msg_t millitimer_msg;
static xtimer_t ot_millitimer;
#ifdef OPENTHREAD_CONFIG_LINK_RETRY_DELAY
static msg_t linkretry_timer_msg;
static xtimer_t ot_linkretry_timer;
#endif
#ifdef MODULE_OPENTHREAD_FTD
static msg_t microtimer_msg;
static xtimer_t ot_microtimer;
#endif

#ifdef MODULE_AT86RF2XX     /* is mutual exclusive with above ifdef */
#define OPENTHREAD_NETIF_NUMOF        (sizeof(at86rf2xx_params) / sizeof(at86rf2xx_params[0]))
static at86rf2xx_t at86rf2xx_dev;
#endif
static msg_t radio_rx_msg;
static msg_t radio_tx_msg;

static mutex_t buffer_mutex = MUTEX_INIT;
static mutex_t radio_mutex = MUTEX_INIT;
#if OPENTHREAD_FINEGRAINED_LOCK
static mutex_t uart_buffer_mutex = MUTEX_INIT;
static mutex_t tasklet_mutex = MUTEX_INIT;
#endif

static char ot_main_thread_stack[THREAD_STACKSIZE_MAIN];
static char ot_task_thread_stack[THREAD_STACKSIZE_MAIN];

void print_active_pid(void) {
    unsigned int pid = sched_active_pid;
    if (pid == 5) {
        printf("pid is 5\n");
    }
    printf("pid %u\n", pid);
}

/* lock Openthread buffer mutex */
void lock_radio_mutex(void) {
    mutex_lock(&radio_mutex);
}

/* unlock Openthread buffer mutex */
void unlock_radio_mutex(void) {
    mutex_unlock(&radio_mutex);
}

/* lock Openthread buffer mutex (roughly) */
void openthread_coarse_lock_buffer_mutex(void) {
#if (!OPENTHREAD_FINEGRAINED_LOCK)
    mutex_lock(&buffer_mutex);
#endif
}

/* unlock Openthread buffer mutex (roughly) */
void openthread_coarse_unlock_buffer_mutex(void) {
#if (!OPENTHREAD_FINEGRAINED_LOCK)
    mutex_unlock(&buffer_mutex);
#endif
}

/* lock Openthread buffer mutex */
void openthread_lock_buffer_mutex(void) {
#if OPENTHREAD_FINEGRAINED_LOCK
    mutex_lock(&buffer_mutex);
#endif
}

/* unlock Openthread buffer mutex */
void openthread_unlock_buffer_mutex(void) {
#if OPENTHREAD_FINEGRAINED_LOCK
    mutex_unlock(&buffer_mutex);
#endif
}

/* lock Openthread uart buffer mutex */
void openthread_lock_uart_buffer_mutex(void) {
#if OPENTHREAD_FINEGRAINED_LOCK
    mutex_lock(&uart_buffer_mutex);
#endif
}

/* unlock Openthread uart buffer mutex */
void openthread_unlock_uart_buffer_mutex(void) {
#if OPENTHREAD_FINEGRAINED_LOCK
    mutex_unlock(&uart_buffer_mutex);
#endif
}

/* lock Openthread tasklet mutex */
void openthread_lock_tasklet_mutex(void) {
#if OPENTHREAD_FINEGRAINED_LOCK
    mutex_unlock(&tasklet_mutex);
#endif
}

/* unlock Openthread tasklet mutex */
void openthread_unlock_tasklet_mutex(void) {
#if OPENTHREAD_FINEGRAINED_LOCK
    mutex_unlock(&tasklet_mutex);
#endif
}

/* get OpenThread netdev */
netdev_t* openthread_get_netdev(void) {
    return (netdev_t*) &at86rf2xx_dev;
}

/* get OpenThread timer */
xtimer_t* openthread_get_millitimer(void) {
    return &ot_millitimer;
}

#ifdef OPENTHREAD_CONFIG_LINK_RETRY_DELAY
/* get OpenThread timer */
xtimer_t* openthread_get_linkretry_timer(void) {
    return &ot_linkretry_timer;
}

/* Interupt handler for OpenThread linkretry-timer event */
static void _linkretry_timer_cb(void* arg) {
    linkretry_timer_msg.type = OPENTHREAD_LINK_RETRY_TIMEOUT;
  	if (msg_send(&linkretry_timer_msg, openthread_get_task_pid()) <= 0) {
        while (1) {
            printf("ot_task: possibly lost timer interrupt.\n");
        }
    }
}
#endif

/* Interupt handler for OpenThread milli-timer event */
static void _millitimer_cb(void* arg) {
    millitimer_msg.type = OPENTHREAD_MILLITIMER_MSG_TYPE_EVENT;
	  if (msg_send(&millitimer_msg, openthread_get_main_pid()) <= 0) {
        while (1) {
            printf("ot_event: possibly lost timer interrupt.\n");
        }
    }
}

#ifdef MODULE_OPENTHREAD_FTD
/* get OpenThread timer */
xtimer_t* openthread_get_microtimer(void) {
    return &ot_microtimer;
}

/* Interupt handler for OpenThread micro-timer event */
static void _microtimer_cb(void* arg) {
   	microtimer_msg.type = OPENTHREAD_MICROTIMER_MSG_TYPE_EVENT;
	  if (msg_send(&microtimer_msg, openthread_get_task_pid()) <= 0) {
        while(1) {
            printf("ot_task: possibly lost timer interrupt.\n");
        }
    }
}
#endif

/* Interupt handler for OpenThread event thread */
static void _event_cb(netdev_t *dev, netdev_event_t event) {
    switch (event) {
        case NETDEV_EVENT_ISR:
            {
                radio_rx_msg.type = OPENTHREAD_NETDEV_MSG_TYPE_EVENT;
                radio_rx_msg.content.ptr = dev;
                radio_rx_msg.content.value = 1;
#ifdef MODULE_OPENTHREAD_FTD
                unsigned irq_state = irq_disable();
                ((at86rf2xx_t *)dev)->pending_irq++;
                irq_restore(irq_state);
#endif
                if (msg_send(&radio_rx_msg, openthread_get_main_pid()) <= 0) {
                    printf("ot_main: possibly lost radio interrupt.\n");
#ifdef MODULE_OPENTHREAD_FTD
                    unsigned irq_state = irq_disable();
                    ((at86rf2xx_t *)dev)->pending_irq--;
                    irq_restore(irq_state);
#endif
                }
                break;
            }
        case NETDEV_EVENT_ISR2:
            {
                radio_tx_msg.type = OPENTHREAD_NETDEV_MSG_TYPE_EVENT;
                radio_tx_msg.content.ptr = dev;
                radio_tx_msg.content.value = 0;
                if (msg_send(&radio_tx_msg, openthread_get_task_pid()) <= 0) {
                    printf("ot_task: possibly lost radio interrupt.\n");
                }
                break;
            }
        case NETDEV_EVENT_RX_COMPLETE:
            recv_pkt(openthread_get_instance(), dev);
            break;
        case NETDEV_EVENT_TX_COMPLETE:
        case NETDEV_EVENT_TX_COMPLETE_DATA_PENDING:
        case NETDEV_EVENT_TX_NOACK:
        case NETDEV_EVENT_TX_MEDIUM_BUSY:
            sent_pkt(openthread_get_instance(), event);
            break;
        default:
            break;
    }
}

uint8_t ot_call_command(char* command, void *arg, void* answer) {
    ot_job_t job;

    job.command = command;
    job.arg = arg;
    job.answer = answer;

    msg_t msg, reply;
    msg.type = OPENTHREAD_JOB_MSG_TYPE_EVENT;
    msg.content.ptr = &job;
    msg_send_receive(&msg, &reply, openthread_get_main_pid());
    return (uint8_t)reply.content.value;
}

void openthread_bootstrap(void)
{
    DEBUG("OT init start\n");
    /* init random */
    ot_random_init();

    /* set openthread timer callback */
    ot_millitimer.callback = _millitimer_cb;
#ifdef MODULE_OPENTHREAD_FTD
    ot_microtimer.callback = _microtimer_cb;
#endif
#ifdef OPENTHREAD_CONFIG_LINK_RETRY_DELAY
    ot_linkretry_timer.callback = _linkretry_timer_cb;
#endif

    /* setup netdev modules */
#ifdef MODULE_AT86RF2XX
    at86rf2xx_setup(&at86rf2xx_dev, &at86rf2xx_params[0]);
    netdev_t *netdev = (netdev_t *) &at86rf2xx_dev;
#endif
    netdev->driver->init(netdev);
    netdev->event_callback = _event_cb;
    netopt_enable_t enable = NETOPT_ENABLE;
    netdev->driver->set(netdev, NETOPT_RX_END_IRQ, &enable, sizeof(enable));
    netdev->driver->set(netdev, NETOPT_TX_END_IRQ, &enable, sizeof(enable));
    openthread_radio_init(netdev);
    DEBUG("OT-RADIO setting is OK\n");

    /* enable OpenThread UART */
    otPlatUartEnable();
    DEBUG("OT-UART setting is OK\n");

    /* init three threads for openthread */
    openthread_task_init(ot_task_thread_stack, sizeof(ot_task_thread_stack),
                        THREAD_PRIORITY_MAIN - 1, "openthread_task");
    openthread_main_init(ot_main_thread_stack, sizeof(ot_main_thread_stack),
                         THREAD_PRIORITY_MAIN - 2, "openthread_main");
}
