/*
 * Event groups on RT-Thread.
 *
 * The mapping is nearly direct: rt_event already supports AND/OR matching and
 * clear-on-match, which are exactly xEventGroupWaitBits' xWaitForAllBits and
 * xClearOnExit.
 *
 * Two details that do not map one to one:
 *
 * 1. xEventGroupClearBits() clears only the given bits, while rt_event_control
 *    with RT_IPC_CMD_RESET clears the whole set (src/ipc.c:2312 does
 *    `event->set = 0`). So the partial clear is done here, under the same
 *    spinlock rt_event uses internally rather than with a bare interrupt lock,
 *    to stay correct if this ever runs on both cores.
 *
 * 2. FreeRTOS returns the whole bit set as it stands when the wait finishes,
 *    not just the bits that matched. rt_event_recv only reports the matched
 *    bits, so the value is read back from the event object after the call. With
 *    xClearOnExit the read happens after the clear, matching FreeRTOS.
 */

#include <rtthread.h>

#include "ft_core.h"

struct ft_event
{
    struct rt_event event;
    rt_uint8_t      in_use;
};

struct ft_event *ft_event_create(void)
{
    struct ft_event *e = rt_malloc(sizeof(*e));

    if (e == RT_NULL)
    {
        return RT_NULL;
    }

    rt_memset(e, 0, sizeof(*e));

    if (rt_event_init(&e->event, "ftev", RT_IPC_FLAG_PRIO) != RT_EOK)
    {
        rt_free(e);
        return RT_NULL;
    }

    e->in_use = 1;
    return e;
}

void ft_event_delete(struct ft_event *event)
{
    if (event == RT_NULL)
    {
        return;
    }

    rt_event_detach(&event->event);
    rt_free(event);
}

rt_uint32_t ft_event_set_bits(struct ft_event *event, rt_uint32_t bits)
{
    if (event == RT_NULL)
    {
        return 0;
    }

    if (bits != 0)
    {
        rt_event_send(&event->event, bits);
    }

    return event->event.set;
}

rt_uint32_t ft_event_clear_bits(struct ft_event *event, rt_uint32_t bits)
{
    rt_base_t level;

    if (event == RT_NULL)
    {
        return 0;
    }

    level = rt_spin_lock_irqsave(&event->event.spinlock);
    event->event.set &= ~bits;
    rt_spin_unlock_irqrestore(&event->event.spinlock, level);

    return event->event.set;
}

rt_uint32_t ft_event_get_bits(struct ft_event *event)
{
    if (event == RT_NULL)
    {
        return 0;
    }

    return event->event.set;
}

rt_uint32_t ft_event_wait_bits(struct ft_event *event, rt_uint32_t bits,
                               ft_base_t clear_on_exit, ft_base_t wait_for_all,
                               ft_tick_t ticks)
{
    rt_uint32_t option = wait_for_all ? RT_EVENT_FLAG_AND : RT_EVENT_FLAG_OR;
    rt_uint32_t received = 0;

    if (event == RT_NULL)
    {
        return 0;
    }

    /* Waiting for no bits at all is a poll of the current value. rt_event_recv
     * with a zero mask would wait for nothing and can return immediately with
     * RT_EOK, which is not what a caller expects. */
    if (bits == 0)
    {
        return event->event.set;
    }

    if (clear_on_exit)
    {
        option |= RT_EVENT_FLAG_CLEAR;
    }

    if (rt_event_recv(&event->event, bits, option, ft_ticks_to_rt(ticks),
                      &received) != RT_EOK)
    {
        return 0;
    }

    return event->event.set;
}
