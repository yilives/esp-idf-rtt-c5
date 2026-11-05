#include <rthw.h>
#include <rtthread.h>

/*
 * `s31con` - report the console/uart0 device state, and optionally watch it.
 *
 * Why this exists
 * ---------------
 * `console set uart0` (an upstream msh builtin) wedges the shell on this port: the
 * command prints "console change to uart0" and nothing is ever printed by the shell
 * again. The board does not die - the shell's read path does.
 *
 * Measured state after `console set uart0`:
 *
 *     s31con
 *     uart0      : flag=0x0113 open_flag=0x014b ref=1     <- before
 *
 * and the interaction, from source:
 *
 *   the shell opened uart0 with RDWR|INT_RX|STREAM      -> open_flag 0x14b
 *   rt_console_set_device() opens it with RDWR|STREAM   -> oflag     0x043
 *
 * rt_device_open() compares through RT_DEVICE_OFLAG_MASK, sees a different flag set,
 * and calls rt_serial_open() again. That function begins with
 *
 *     dev->open_flag = oflag & 0xff;          -- 0x043, INT_RX is gone
 *
 * then takes the "ring already exists" branch and leaves serial_rx allocated but no
 * longer selected by open_flag.
 *
 * rt_serial_read() picks its path from `dev->open_flag & RT_DEVICE_FLAG_INT_RX`, so
 * the shell silently switches from the interrupt ring to _serial_poll_rx(), which
 * reads the UART FIFO that s31_uart_isr keeps draining into the ring. The shell's read
 * returns 0 forever and it blocks on rx_sem waiting for a character the ISR already
 * took.
 *
 * `s31con arm` prints the state once a second from a timer, so the change can be
 * observed AFTER the shell stops answering - at that point the shell itself can no
 * longer report anything.
 */

static void con_dump(const char *tag)
{
    rt_device_t dev = rt_device_find("uart0");
    rt_device_t con = rt_console_get_device();

    if (dev == RT_NULL)
    {
        rt_kprintf("%s uart0: not found\n", tag);
        return;
    }

    rt_kprintf("%s flag=0x%04x open=0x%04x ref=%u int_rx=%s stream=%s con=%s\n",
               tag,
               (unsigned)dev->flag, (unsigned)dev->open_flag,
               (unsigned)dev->ref_count,
               (dev->open_flag & RT_DEVICE_FLAG_INT_RX) ? "yes" : "NO",
               (dev->open_flag & RT_DEVICE_FLAG_STREAM) ? "yes" : "no",
               con ? con->parent.name : "(none)");
}

static void con_timer_cb(void *param)
{
    static rt_uint32_t n;

    (void)param;

    n++;
    con_dump("[watch]");
}

static rt_timer_t s_con_timer;

static int cmd_con(int argc, char **argv)
{
    if (argc >= 2 && rt_strcmp(argv[1], "arm") == 0)
    {
        if (s_con_timer == RT_NULL)
        {
            s_con_timer = rt_timer_create("conwatch", con_timer_cb, RT_NULL,
                                          rt_tick_from_millisecond(1000),
                                          RT_TIMER_FLAG_PERIODIC);
        }

        if (s_con_timer != RT_NULL)
        {
            rt_timer_start(s_con_timer);
            rt_kprintf("watching uart0 once per second\n");
        }
        else
        {
            rt_kprintf("timer create failed\n");
        }

        return 0;
    }

    con_dump("[now]  ");
    return 0;
}
MSH_CMD_EXPORT_ALIAS(cmd_con, s31con, show or watch the console and uart0 state);

void s31_con_cmd_link_anchor(void)
{
}
