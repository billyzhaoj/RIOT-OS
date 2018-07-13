/**
 * Copyright (C) 2015 Kaspar Schleiser <kaspar@schleiser.de>
 * Copyright (C) 2016 Eistec AB
 * Copyright (C) 2018 UC Berkeley
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup sys_xtimer
 *
 * @{
 * @file
 * @brief xtimer core functionality
 * @author Kaspar Schleiser <kaspar@schleiser.de>
 * @author Joakim Nohlgård <joakim.nohlgard@eistec.se>
 * @author Hyung-Sin Kim <hs.kim@cs.berkeley.edu>
 * @}
 */

#include <stdint.h>
#include <string.h>
#include "board.h"
#include "periph/timer.h"
#include "periph_conf.h"

#include "xtimer.h"
#include "irq.h"

/* WARNING! enabling this will have side effects and can lead to timer underflows. */
#define ENABLE_DEBUG 0
#include "debug.h"

static volatile int _in_handler = 0;

volatile uint64_t _xtimer_current_time = 0;
volatile uint32_t _long_cnt = 0;
#if XTIMER_MASK
volatile uint32_t _xtimer_high_cnt = 0;
#endif

static xtimer_t *timer_list_head = NULL;
static xtimer_t *long_list_head = NULL;

static void _add_timer_to_list(xtimer_t **list_head, xtimer_t *timer);
static void _add_timer_to_long_list(xtimer_t **list_head, xtimer_t *timer);
static void _shoot(xtimer_t *timer);
static void _remove(xtimer_t *timer);
static void _update_short_timers(void);
static inline void _lltimer_set(uint32_t target);

static void _timer_callback(void);
static void _periph_timer_callback(void *arg, int chan);

static uint32_t _current_target = 0;

static inline int _is_set(xtimer_t *timer)
{
    return (timer->target || timer->offset);
}

void xtimer_init(void)
{
    /* initialize low-level timer */
    timer_init(XTIMER_DEV, XTIMER_HZ, _periph_timer_callback, NULL);

    /* register initial overflow tick */
    _lltimer_set(0xFFFFFFFF);
}

uint64_t _xtimer_now64(void)
{
    uint32_t short_term, long_term;

    /* time sensitive since _long_cnt is used */
    uint8_t state = irq_disable();
    short_term = _xtimer_now();
    long_term = _long_cnt;
    irq_restore(state);

    return ((uint64_t)long_term<<32) + short_term;
}

void _xtimer_set64(xtimer_t *timer, uint32_t offset, uint32_t long_offset)
{
    DEBUG(" _xtimer_set64() offset=%" PRIu32 " long_offset=%" PRIu32 "\n", offset, long_offset);
    if (!long_offset) {
        /* timer fits into the short timer */
        _xtimer_set(timer, offset);
    }
    else {
        xtimer_remove(timer);
        
        /* time sensitive */
        uint8_t state = irq_disable();
        timer->start_time = _xtimer_now();
        timer->offset = offset;
        timer->long_offset = long_offset;
        timer->target = timer->start_time + offset;

        _add_timer_to_long_list(&long_list_head, timer);
        irq_restore(state);
    }
}

void _xtimer_set(xtimer_t *timer, uint32_t offset)
{
    DEBUG("timer_set(): offset=%" PRIu32 " now=%" PRIu32 " (%" PRIu32 ")\n",
          offset, xtimer_now().ticks32, _xtimer_lltimer_now());
    if (!timer->callback) {
        DEBUG("timer_set(): timer has no callback.\n");
        return;
    }

    xtimer_remove(timer);

    if (offset < XTIMER_BACKOFF) {
        _xtimer_spin(offset);
        _shoot(timer);
    }
    else {
        /* time sensitive from "now" access to hardware timer set */
        uint8_t state = irq_disable();
        timer->start_time = _xtimer_now();
        timer->offset = offset;
        timer->long_offset = 0;
        timer->target = timer->start_time + offset;

        _add_timer_to_list(&timer_list_head, timer);

        if (timer_list_head == timer) {
            DEBUG("timer_set_absolute(): timer is new list head. updating lltimer.\n");
            if (timer->offset <= _xtimer_lltimer_mask(0xFFFFFFFF)) {
                /* schedule callback on next timer target time */
                _lltimer_set(timer->target);
            }
            else {
                /* schedule callback after max_low_level_time/2
                 * to update _long_cnt and/or _xtimer_high_cnt */
                _lltimer_set(timer->start_time + (_xtimer_lltimer_mask(0xFFFFFFFF)>>1));
            }
        }
        irq_restore(state);
    }
}

static void _periph_timer_callback(void *arg, int chan)
{
    (void)arg;
    (void)chan;
    _timer_callback();
}

static void _shoot(xtimer_t *timer)
{
    timer->callback(timer->arg);
}

static inline void _lltimer_set(uint32_t target)
{
    if (_in_handler) {
        printf("weird operation\n");
        return;
    }
    DEBUG("_lltimer_set(): setting %" PRIu32 "\n", _xtimer_lltimer_mask(_current_target));
    timer_set_absolute(XTIMER_DEV, XTIMER_CHAN, _xtimer_lltimer_mask(target));
    uint8_t state = irq_disable();
    _current_target = target;
    irq_restore(state);
}

static void _add_timer_to_list(xtimer_t **list_head, xtimer_t *timer)
{
    while (*list_head
        && (*list_head)->target - timer->start_time <= timer->target - timer->start_time) {
        list_head = &((*list_head)->next);
    }

    timer->next = *list_head;
    *list_head = timer;
}

static void _add_timer_to_long_list(xtimer_t **list_head, xtimer_t *timer)
{
    while (*list_head
        && (((*list_head)->long_offset < timer->long_offset)
        || (((*list_head)->long_offset == timer->long_offset)
            && ((*list_head)->target - timer->start_time <= timer->target - timer->start_time)))) {
        list_head = &((*list_head)->next);
    }

    timer->next = *list_head;
    *list_head = timer;
}

static int _remove_timer_from_list(xtimer_t **list_head, xtimer_t *timer)
{
    while (*list_head) {
        if (*list_head == timer) {
            *list_head = timer->next;
            timer->target = 0;
            timer->offset = 0;
            timer->long_offset = 0;
            timer->next = NULL;
            return 1;
        }
        list_head = &((*list_head)->next);
    }

    return 0;
}

static void _remove(xtimer_t *timer)
{
    if (timer == timer_list_head) {
        timer_list_head = timer->next;
        timer->target = 0;
        timer->offset = 0;
        timer->long_offset = 0;
        timer->next = NULL;

        _update_short_timers();

        if (timer_list_head) {
            if (timer_list_head->offset <= _xtimer_lltimer_mask(0xFFFFFFFF)) {
                /* schedule callback on next timer target time */
                _lltimer_set(timer_list_head->target);
            }
            else {
                /* schedule callback after max_low_level_time/2
                 * to update _long_cnt and/or _xtimer_high_cnt */
                _lltimer_set(_xtimer_now() + (_xtimer_lltimer_mask(0xFFFFFFFF)>>1));
            }
        }
        else {
            /* schedule callback after max_low_level_time/2
             * to update _long_cnt and/or _xtimer_high_cnt */
            _lltimer_set(_xtimer_now() + (_xtimer_lltimer_mask(0xFFFFFFFF)>>1));
        }
    }
    else {
        if (!_remove_timer_from_list(&timer_list_head, timer)) {
            _remove_timer_from_list(&long_list_head, timer);
        }
    }
}

void xtimer_remove(xtimer_t *timer)
{
    /* time sensitive since the target timer can be fired */
    int state = irq_disable();
    if (_is_set(timer)) {
        _remove(timer);
    }
    irq_restore(state);
}

/**
 * @brief update long timers' offsets and switch those that will expire in
 *        one short timer period to the short timer list
 */
static void _update_long_timers(void)
{
    if (long_list_head) {
        xtimer_t *curr_timer = long_list_head;
        uint32_t now = _xtimer_now();

        while (curr_timer) {
            uint32_t elapsed = now - curr_timer->start_time;
            if (curr_timer->offset <= elapsed) {
                curr_timer->long_offset--;
            }
            curr_timer->offset -= elapsed;
            curr_timer->start_time = now;

            if (!curr_timer->long_offset) {
                printf("switch from long to short\n");
                assert(curr_timer == long_list_head);
 
                xtimer_t *trans_timer = curr_timer;
                curr_timer = curr_timer->next;
                _remove_timer_from_list(&long_list_head, trans_timer);
                _add_timer_to_list(&timer_list_head, trans_timer);
            }
            else {
                curr_timer = curr_timer->next;
            }
        }
    }
}

/**
 * @brief update short timers' offsets and fire those that is close to expiry
 */
static void _update_short_timers(void) {
    while (timer_list_head) {
        uint32_t now = _xtimer_now();
        uint32_t elapsed = now - timer_list_head->start_time;

        if (timer_list_head->offset <= elapsed ||
            timer_list_head->offset - elapsed < XTIMER_ISR_BACKOFF) {            
            /* make sure we don't fire too early */
            while(_xtimer_now() - timer_list_head->start_time < timer_list_head->offset) {}

            /* pick first timer in list */
            xtimer_t *timer = timer_list_head;

            /* advance list */
            timer_list_head = timer->next;

            /* make sure timer is recognized as being already fired */
            timer->target = 0;
            timer->offset = 0;
            timer->long_offset = 0;
            timer->next = NULL;

            /* fire timer */
            _shoot(timer);
        }
        else {
            timer_list_head->offset -= elapsed;
            timer_list_head->start_time = now;
            return;
        }
    }
}

/**
 * @brief main xtimer callback function
 */
static void _timer_callback(void)
{
    uint32_t next_target, now;
    _in_handler = 1;
    _current_target = 0;

overflow:
    /* check if any timer is close to expiring */
    _update_short_timers();

    _update_long_timers();

    now = _xtimer_now();

    if (timer_list_head) {
        /* make sure we're not setting a time in the past */
        uint32_t elapsed = now - timer_list_head->start_time;
        if (timer_list_head->offset <= elapsed ||
            timer_list_head->offset - elapsed <= XTIMER_ISR_BACKOFF) {
            goto overflow;
        }
        else {
            timer_list_head->offset -= elapsed;
            timer_list_head->start_time = now;
        }
 
        if (timer_list_head->offset <= _xtimer_lltimer_mask(0xFFFFFFFF)) {
            /* schedule callback on next timer target time */
            next_target = timer_list_head->target;
        }
        else {
            /* schedule callback after max_low_level_time/2
             * to update _long_cnt and/or _xtimer_high_cnt */
            next_target = now + (_xtimer_lltimer_mask(0xFFFFFFFF)>>1);
        }
    }
    else {
        /* there's no timer planned for this timer period */
        /* schedule callback after max_low_level_time/2
         * to update _long_cnt and/or _xtimer_high_cnt */
        next_target = now + (_xtimer_lltimer_mask(0xFFFFFFFF)>>1);
    }

    _in_handler = 0;

    /* set low level timer */
    _lltimer_set(next_target);
}
