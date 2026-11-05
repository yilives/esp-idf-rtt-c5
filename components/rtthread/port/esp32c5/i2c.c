/*
 * ESP32-C5 I2C master: rt_i2c_bus_device_ops over the I2C LL.
 *
 * Same reasoning as the UART, GPIO and PWM drivers in this directory:
 * esp_driver_i2c sits on FreeRTOS primitives (xQueue for its command queue,
 * xQueueSendFromISR in its ISR), so driving the LL directly is smaller and
 * auditable. Everything below touches registers only.
 *
 * Ported from port/esp32s31/i2c.c. Four things differ on this chip and each of
 * them is a silent failure if carried over unchanged:
 *
 * 1. THERE IS ONE HP I2C INSTANCE, NOT TWO. SOC_HP_I2C_NUM is 1 on the C5;
 *    SOC_I2C_NUM is 2 because it counts the LP controller, which has a separate
 *    register block (LP_I2C) and is out of scope here. The S31 file indexes two
 *    HP buses and would read `&I2C1`, which does not exist on this target -
 *    I2C_LL_GET_HW() maps port 1 to &LP_I2C.
 *
 * 2. THE CLOCK SOURCE IS SELECTED THROUGH PCR, AND DEFAULTS TO XTAL. The S31
 *    gates I2C through HP_SYS_CLKRST with its own divider. Here
 *    i2c_ll_set_source_clk() writes PCR.i2c[0].i2c_sclk_conf.i2c_sclk_sel, and
 *    the choice is XTAL (0) or RC_FAST (1). XTAL is used for the same reason as
 *    everywhere else in this port: it is exact, so the bus rate does not move
 *    when DFS changes the CPU frequency. The divider has to be computed from
 *    that same source, which is why esp_clk_xtal_freq() is read at run time -
 *    CONFIG_XTAL_FREQ is 0 on this board (CONFIG_XTAL_FREQ_AUTO).
 *
 * 3. THE INTERRUPT STATUS BITS ARE AT DIFFERENT POSITIONS. On the S31 this file
 *    tested bit 1 for NACK and bit 5 for timeout. On the C5 those are
 *    txfifo_wm_int_st and arbitration_lost_int_st; NACK is bit 10 and the two
 *    timeouts are bits 13 and 14. Testing the S31 masks here would report a
 *    NACK whenever the TX FIFO watermark fired. The struct field names are used
 *    instead of magic numbers so the positions cannot drift again.
 *
 * 4. THE PAD SIGNALS COME FROM A TABLE, NOT FROM io_mux_reg.h. The S31 file
 *    takes I2C0_SCL_PAD_OUT_IDX from soc/gpio_sig_map.h; on the C5 those names
 *    do not exist and the indices live in i2c_periph_signal[], whose HP entry
 *    is I2CEXT0_{SCL,SDA}_{IN,OUT}_IDX.
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#include "sdkconfig.h"
#include "soc/soc.h"
#include "soc/soc_caps.h"
#include "soc/i2c_struct.h"
#include "hal/gpio_ll.h"
#include "hal/i2c_ll.h"
#include "hal/i2c_types.h"
#include "hal/i2c_periph.h"
#include "esp_rom_gpio.h"
#include "esp_private/esp_clk.h"

#if defined(SOC_I2C_SUPPORTED)

#include "port_i2c.h"

/* HP instances only. The LP controller is a different register block, so
 * SOC_I2C_NUM (2) is deliberately not used as the array bound - C5_I2C_HP_NUM
 * comes from port_i2c.h, which documents that value. */

struct c5_i2c
{
    struct rt_i2c_bus_device parent;
    i2c_dev_t               *hw;
    rt_uint32_t              port;
    rt_uint32_t              freq;
};

static struct c5_i2c s_i2c[C5_I2C_HP_NUM];

/* Route SCL/SDA: IO_MUX to GPIO function, then the matrix carries the I2C
 * signals out. SDA is bidirectional, so both pads keep their input enable and
 * the output-enable source is pointed at the peripheral. */
static rt_err_t c5_i2c_set_pins(rt_uint32_t port, rt_base_t scl, rt_base_t sda)
{
    const i2c_signal_conn_t *sig = &i2c_periph_signal[port];

    if (scl < 0 || sda < 0 || scl >= SOC_GPIO_PIN_COUNT || sda >= SOC_GPIO_PIN_COUNT ||
        ((SOC_GPIO_VALID_GPIO_MASK >> scl) & 1ULL) == 0 ||
        ((SOC_GPIO_VALID_GPIO_MASK >> sda) & 1ULL) == 0)
    {
        return -RT_EINVAL;
    }

    /* Both pads: GPIO function, input on (SDA is read back even while driven),
     * pull-ups on. The internal pull-up is weak (~45 kOhm) - fine for a bench
     * check, not for a real bus; an external 4.7 kOhm is still required. */
    gpio_ll_func_sel(&GPIO, (uint8_t)scl, PIN_FUNC_GPIO);
    gpio_ll_func_sel(&GPIO, (uint8_t)sda, PIN_FUNC_GPIO);
    gpio_ll_pullup_en(&GPIO, scl);
    gpio_ll_pullup_en(&GPIO, sda);
    gpio_ll_input_enable(&GPIO, scl);
    gpio_ll_input_enable(&GPIO, sda);
    gpio_ll_od_enable(&GPIO, scl);
    gpio_ll_od_enable(&GPIO, sda);

    /* Output-enable comes from the peripheral, not from the GPIO register. */
    gpio_ll_set_output_enable_ctrl(&GPIO, (uint8_t)scl, true, true);
    gpio_ll_set_output_enable_ctrl(&GPIO, (uint8_t)sda, true, true);

    esp_rom_gpio_connect_out_signal((uint32_t)scl, sig->scl_out_sig, false, false);
    esp_rom_gpio_connect_out_signal((uint32_t)sda, sig->sda_out_sig, false, false);
    esp_rom_gpio_connect_in_signal((uint32_t)scl, sig->scl_in_sig, false);
    esp_rom_gpio_connect_in_signal((uint32_t)sda, sig->sda_in_sig, false);

    return RT_EOK;
}

/* Program divider + SCL timing for a bus frequency. */
static void c5_i2c_apply_freq(struct c5_i2c *bus, rt_uint32_t freq)
{
    i2c_hal_clk_config_t clk = {0};
    rt_uint32_t src_hz = (rt_uint32_t)esp_clk_xtal_freq();

    if (freq == 0 || freq > 1000000u)
    {
        freq = 100000u;
    }

    i2c_ll_master_cal_bus_clk(src_hz, freq, &clk);
    i2c_ll_master_set_bus_timing(bus->hw, &clk);
    i2c_ll_set_scl_clk_timing(bus->hw, clk.scl_high, clk.scl_low, clk.scl_wait_high);
    i2c_ll_master_set_start_timing(bus->hw, clk.setup, clk.hold);
    i2c_ll_master_set_stop_timing(bus->hw, clk.setup, clk.hold);
    i2c_ll_set_sda_timing(bus->hw, clk.sda_sample, clk.sda_hold);
    i2c_ll_set_tout(bus->hw, clk.tout);
    i2c_ll_update(bus->hw);

    bus->freq = freq;
}

/* Reset the controller's FIFOs and clear stale status. A previous timeout can
 * leave commands pending and the bus-busy flag set. */
static void c5_i2c_reset_state(struct c5_i2c *bus)
{
    /* FSM reset first: a previous timed-out transfer leaves the command state
     * machine mid-sequence, and clearing the FIFOs alone does not bring it back.
     * IDF exposes the same step as i2c_hal_master_fsm_rst(). */
    i2c_ll_master_fsm_rst(bus->hw);
    i2c_ll_txfifo_rst(bus->hw);
    i2c_ll_rxfifo_rst(bus->hw);
    i2c_ll_clear_intr_mask(bus->hw, 0xFFFFFFFFu);
    i2c_ll_update(bus->hw);
}

/*
 * One message. Returns RT_EOK, or -RT_EIO when the slave did not ACK.
 *
 * The command list has 8 entries and a message is at most
 * { RSTART, WRITE addr, WRITE|READ data, STOP, END }, which fits because one
 * command carries up to 255 bytes (byte_num is 8 bits).
 */
static rt_err_t c5_i2c_xfer_msg(struct c5_i2c *bus, struct rt_i2c_msg *msg)
{
    i2c_dev_t *hw = bus->hw;
    rt_uint32_t idx = 0;
    rt_uint8_t  addr = (rt_uint8_t)(msg->addr << 1);
    i2c_ll_hw_cmd_t cmd;
    rt_uint32_t guard;
    rt_bool_t   nack = RT_FALSE;

    c5_i2c_reset_state(bus);

    /* --- address phase --- */
    cmd.val         = 0;
    cmd.op_code     = I2C_LL_CMD_RESTART;
    cmd.byte_num    = 0;
    cmd.ack_en      = 0;
    cmd.ack_exp     = 0;
    i2c_ll_master_write_cmd_reg(hw, cmd, (int)idx++);

    cmd.val         = 0;
    cmd.op_code     = I2C_LL_CMD_WRITE;
    cmd.byte_num    = 1;
    cmd.ack_en      = 1;
    cmd.ack_exp     = 0;            /* expect ACK (0) */
    cmd.ack_val     = 0;
    i2c_ll_master_write_cmd_reg(hw, cmd, (int)idx++);
    i2c_ll_write_txfifo(hw, &addr, 1);

    /* --- data phase --- */
    if (msg->flags & RT_I2C_RD)
    {
        cmd.val         = 0;
        cmd.op_code     = I2C_LL_CMD_READ;
        cmd.byte_num    = (rt_uint8_t)msg->len;
        cmd.ack_en      = 0;
        cmd.ack_val     = 1;        /* 1 = NACK after the last byte */
        i2c_ll_master_write_cmd_reg(hw, cmd, (int)idx++);
    }
    else if (msg->len > 0)
    {
        cmd.val         = 0;
        cmd.op_code     = I2C_LL_CMD_WRITE;
        cmd.byte_num    = (rt_uint8_t)msg->len;
        cmd.ack_en      = 1;
        cmd.ack_exp     = 0;
        i2c_ll_master_write_cmd_reg(hw, cmd, (int)idx++);

        i2c_ll_write_txfifo(hw, msg->buf, (uint8_t)msg->len);
    }
    /* msg->len == 0: no data-phase command at all.
     *
     * This is the bus-scan case - "does this address ACK" - and it was the bug
     * that made the whole driver look dead. A WRITE command with byte_num == 0
     * is emitted but never retired: measured, the command state machine
     * completed the RESTART and the address byte and then sat on the
     * zero-length WRITE forever, with int_status completely clear (no NACK, no
     * timeout), so every probe burned the full guard budget and reported
     * -RT_ETIMEOUT.
     *
     * The register dump that found it:
     *
     *   ctr=0x00000010 status=0x00000000 scl_low=240 scl_high=122
     *   cmd[0]=0x80003000   RESTART, done=1
     *   cmd[1]=0x80000901   WRITE 1 byte (the address), done=1
     *   cmd[2]=0x00000900   WRITE 0 bytes, done=0   <- stuck here
     *   cmd[3]=0x00001000   STOP,  done=0
     *   cmd[4]=0x00002000   END,   done=0
     *
     * done=1 on the first two also proves the controller really clocked the bus,
     * so the pads, the clock and the timing were fine the whole time.
     *
     * The address-phase ACK is what the caller wants, and ack_en on that WRITE
     * already checks it, so omitting the empty data command loses nothing. */

    /* --- stop + end --- */
    cmd.val         = 0;
    cmd.op_code     = I2C_LL_CMD_STOP;
    cmd.byte_num    = 0;
    i2c_ll_master_write_cmd_reg(hw, cmd, (int)idx++);

    cmd.val         = 0;
    cmd.op_code     = I2C_LL_CMD_END;
    cmd.byte_num    = 0;
    i2c_ll_master_write_cmd_reg(hw, cmd, (int)idx++);

    i2c_ll_update(hw);
    i2c_ll_start_trans(hw);

    /* The first transfer after init frequently needs the FSM kicked: trans_start
     * is a self-clearing bit and the controller can miss it if the previous
     * state machine was reset by i2c_ll_master_fsm_rst() in the same pass. */
    if (!hw->ctr.trans_start)
    {
        i2c_ll_start_trans(hw);
    }

    /*
     * Wait for the END command to retire, with a bound derived from the bus
     * speed rather than a fixed loop count. A message of `len` bytes takes at
     * most (len + 2) * 9 SCL periods; at 100 kHz that is tens of microseconds,
     * so a few milliseconds is generous. The S31 file learned this the hard way:
     * a fixed 2,000,000-iteration bound made a bus scan of 112 non-ACKing
     * addresses look like a hang.
     *
     * A missing ACK makes the controller abandon the remaining commands, so END
     * never sets command_done - which is why the status bits are checked inside
     * the loop and not only after it.
     */
    {
        rt_uint32_t budget = ((msg->len + 4u) * 9u * 1000u) / (bus->freq / 1000u);

        if (budget < 20000u)
        {
            budget = 20000u;
        }

        guard = 0;
        while (!hw->command[idx - 1].command_done)
        {
            /* NACK, or the bus held low past the timeout: either way the
             * transfer is over and failed. Field names, not bit numbers - the C5
             * moved these relative to the S31. */
            /* int_raw, not int_status.
             *
             * int_status is the *masked* view: it only shows an event whose
             * enable bit is set in int_ena, and this driver never enables any
             * interrupt (it polls). Reading int_status was therefore always
             * zero - measured in the timeout dump, status=0x00000000 while the
             * NACK the hardware had certainly raised was invisible. int_raw is
             * the unmasked status and is what a polling driver has to use.
             *
             * IDF reaches the same facts through i2c_ll_master_get_event(),
             * which reads hw->int_status.val - but only after
             * i2c_ll_master_enable_tx_it() has set the enable bits. */
            if (hw->int_raw.nack_int_raw ||
                hw->int_raw.time_out_int_raw ||
                hw->int_raw.scl_st_to_int_raw ||
                hw->int_raw.arbitration_lost_int_raw)
            {
                break;
            }

            if (++guard > budget)
            {
                static int dumped;

                /* Dump the controller state once per boot. A timeout with
                 * command_done=0 and ctr.trans_start=0 means the FSM never
                 * started; a timeout with trans_start=1 means it started and
                 * stalled, which points at the pads or the timing. */
                if (!dumped)
                {
                    rt_uint32_t k;

                    dumped = 1;
                    rt_kprintf("i2c     : timeout dump ctr=0x%08x status=0x%08x "
                               "scl_low=%u scl_high=%u\n",
                               (unsigned)hw->ctr.val,
                               (unsigned)hw->int_raw.val,
                               (unsigned)hw->scl_low_period.scl_low_period,
                               (unsigned)hw->scl_high_period.scl_high_period);
                    for (k = 0; k < idx; k++)
                    {
                        rt_kprintf("i2c     : cmd[%u]=0x%08x\n",
                                   (unsigned)k, (unsigned)hw->command[k].val);
                    }
                }

                c5_i2c_reset_state(bus);
                return -RT_ETIMEOUT;
            }
        }
    }

    nack = hw->int_raw.nack_int_raw ? RT_TRUE : RT_FALSE;

    i2c_ll_clear_intr_mask(hw, 0xFFFFFFFFu);

    if (msg->flags & RT_I2C_RD)
    {
        uint32_t cnt = 0;

        if (nack)
        {
            return -RT_EIO;
        }

        i2c_ll_get_rxfifo_cnt(hw, &cnt);
        if (cnt < msg->len)
        {
            return -RT_EIO;         /* short read: bus glitch or NACK */
        }

        i2c_ll_read_rxfifo(hw, msg->buf, (uint8_t)msg->len);
    }
    else if (nack)
    {
        return -RT_EIO;
    }

    return RT_EOK;
}

static rt_ssize_t c5_i2c_master_xfer(struct rt_i2c_bus_device *bus,
                                     struct rt_i2c_msg msgs[], rt_uint32_t num)
{
    struct c5_i2c *dev = (struct c5_i2c *)bus;
    rt_uint32_t i;

    for (i = 0; i < num; i++)
    {
        rt_err_t rc;

        /* 10-bit addressing is supported by the hardware but not implemented
         * here: the address phase would need a two-byte WRITE. Refusing is
         * better than sending a wrong address. */
        if (msgs[i].addr > 0x7F || msgs[i].len > 255)
        {
            return -RT_EINVAL;
        }

        rc = c5_i2c_xfer_msg(dev, &msgs[i]);
        if (rc != RT_EOK)
        {
            return (i == 0) ? rc : (rt_ssize_t)i;
        }
    }

    return (rt_ssize_t)num;
}

static rt_err_t c5_i2c_bus_control(struct rt_i2c_bus_device *bus, int cmd, void *args)
{
    struct c5_i2c *dev = (struct c5_i2c *)bus;

    switch (cmd)
    {
    case RT_I2C_DEV_CTRL_CLK:
        if (args == RT_NULL)
        {
            return -RT_EINVAL;
        }
        c5_i2c_apply_freq(dev, *(rt_uint32_t *)args);
        return RT_EOK;

    case RT_I2C_DEV_CTRL_TIMEOUT:
        /* Not settable: the hardware timeout is derived from the bus frequency
         * by i2c_ll_master_cal_bus_clk(), which is the value it needs for its
         * own counting. */
        return RT_EOK;

    default:
        return -RT_ENOSYS;
    }
}

static const struct rt_i2c_bus_device_ops c5_i2c_ops =
{
    .master_xfer     = c5_i2c_master_xfer,
    .slave_xfer      = RT_NULL,
    .i2c_bus_control = c5_i2c_bus_control,
};

rt_err_t c5_i2c_init_port(rt_uint32_t port, rt_base_t scl, rt_base_t sda,
                          rt_uint32_t freq)
{
    struct c5_i2c *bus;
    char name[8];

    if (port >= C5_I2C_HP_NUM)
    {
        return -RT_EINVAL;
    }

    bus = &s_i2c[port];
    bus->hw   = I2C_LL_GET_HW((int)port);
    bus->port = port;

    if (c5_i2c_set_pins(port, scl, sda) != RT_EOK)
    {
        return -RT_EINVAL;
    }

    /* Bus clock and register reset, then pick XTAL as the source before any
     * timing is computed from it. */
    i2c_ll_enable_bus_clock((int)port, true);
    i2c_ll_reset_register((int)port);
    i2c_ll_set_source_clk(bus->hw, I2C_CLK_SRC_XTAL);

    /* The controller clock is separate from the bus clock on this chip, and
     * skipping it is invisible: every register read-back looks correct while no
     * transfer ever retires. That was the first version of this file - measured
     * as 112 timeouts out of 112 addresses with idle levels reading 1/1.
     *
     * The four calls below mirror i2c_hal_master_init() exactly, because IDF's
     * own sequence is the only description of what this hardware needs:
     * master mode, open-drain pads, arbitration off (single master), and the
     * receive-ack level. */
    i2c_ll_enable_controller_clock(bus->hw, true);
    i2c_ll_set_mode(bus->hw, I2C_BUS_MODE_MASTER);
    i2c_ll_enable_pins_open_drain(bus->hw, true);
    i2c_ll_enable_arbitration(bus->hw, false);
    i2c_ll_master_rx_full_ack_level(bus->hw, false);

    i2c_ll_enable_fifo_mode(bus->hw, true);
    i2c_ll_set_data_mode(bus->hw, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
    i2c_ll_set_txfifo_empty_thr(bus->hw, 4);
    i2c_ll_set_rxfifo_full_thr(bus->hw, 8);

    c5_i2c_apply_freq(bus, freq ? freq : 100000u);
    i2c_ll_update(bus->hw);

    bus->parent.ops     = &c5_i2c_ops;
    bus->parent.timeout = 100;

    name[0] = 'i';
    name[1] = '2';
    name[2] = 'c';
    name[3] = (char)('0' + port);
    name[4] = '\0';

    return rt_i2c_bus_device_register(&bus->parent, name);
}

rt_err_t rt_hw_i2c_init(void)
{
    /* Pads are configurable on this chip, so a wrong guess drives somebody
     * else's peripheral. The two chosen here are free on the devkit and are
     * documented in port_i2c.h; they avoid the console (IO11/IO12), the PWM
     * test pins (IO3/IO5/IO7) and the in-package flash pads. */
    rt_err_t rc = c5_i2c_init_port(0, C5_I2C0_SCL_PIN, C5_I2C0_SDA_PIN, 100000u);

    if (rc != RT_EOK)
    {
        rt_kprintf("i2c0: init failed %d\n", rc);
    }

    return rc;
}
INIT_BOARD_EXPORT(rt_hw_i2c_init);

#endif /* SOC_I2C_SUPPORTED */


/* ------------------------------------------------------------------ msh -- */

/* `i2cscan [port]` - probe every 7-bit address on the bus.
 *
 * A zero-length write is exactly "does this address ACK": the hardware still
 * sends the address byte and checks the ACK, which is what every bus scanner
 * does. This command is not in the upstream i2c framework (only S3/S31 boards
 * carried one), so it lives here next to the driver it probes.
 */
static int cmd_i2cscan(int argc, char **argv)
{
    rt_uint32_t port = 0;
    struct rt_i2c_bus_device *bus;
    char name[8];
    int i;

    if (argc >= 2)
    {
        port = (rt_uint32_t)atoi(argv[1]);
    }

    if (port >= C5_I2C_HP_NUM)
    {
        rt_kprintf("usage: i2cscan [port 0..%d]\n", C5_I2C_HP_NUM - 1);
        return -1;
    }

    name[0] = 'i'; name[1] = '2'; name[2] = 'c';
    name[3] = (char)('0' + port); name[4] = '\0';

    bus = (struct rt_i2c_bus_device *)rt_device_find(name);
    if (bus == RT_NULL)
    {
        rt_kprintf("i2cscan: %s not registered\n", name);
        return -1;
    }

    rt_kprintf("i2cscan: probing %s at %u Hz (SCL=IO%d SDA=IO%d)\n",
               name, (unsigned)s_i2c[port].freq,
               (int)C5_I2C0_SCL_PIN, (int)C5_I2C0_SDA_PIN);

    /* Report the idle levels before probing.
     *
     * "0 devices" on its own is ambiguous: a bus with nothing attached and a bus
     * whose pads are stuck low both report zero. With the pull-ups enabled and
     * the controller idle, both lines must read 1; anything else means the pad
     * is not actually released from the peripheral, and every address would
     * fail for a reason that has nothing to do with the slaves. Printing the two
     * levels turns the ambiguous zero into a diagnosis. */
    {
        int scl_lvl = gpio_ll_get_level(&GPIO, (uint32_t)C5_I2C0_SCL_PIN);
        int sda_lvl = gpio_ll_get_level(&GPIO, (uint32_t)C5_I2C0_SDA_PIN);

        rt_kprintf("i2cscan: idle level SCL=%d SDA=%d (both 1 = bus released)\n",
                   scl_lvl, sda_lvl);

        if (!scl_lvl || !sda_lvl)
        {
            rt_kprintf("i2cscan: WARNING a line is held low - check pull-ups "
                       "and that nothing else drives IO%d/IO%d\n",
                       (int)C5_I2C0_SCL_PIN, (int)C5_I2C0_SDA_PIN);
        }
    }

    {
        int found = 0;
        int nacks = 0;
        int timeouts = 0;
        int other = 0;
        rt_tick_t t0 = rt_tick_get();

        for (i = 0x08; i <= 0x77; i++)
        {
            struct rt_i2c_msg msg;
            rt_uint8_t dummy = 0;

            msg.addr  = (rt_uint16_t)i;
            msg.flags = RT_I2C_WR;
            msg.buf   = &dummy;
            msg.len   = 0;

            {
                rt_ssize_t rc = rt_i2c_transfer(bus, &msg, 1);

                if (rc == 1)
                {
                    rt_kprintf("  0x%02x ACK\n", i);
                    found++;
                }
                else if (rc == -RT_EIO)
                {
                    nacks++;        /* the address byte went out and was NACKed */
                }
                else if (rc == -RT_ETIMEOUT)
                {
                    timeouts++;     /* the controller never retired the command */
                }
                else
                {
                    other++;
                }
            }
        }

        /* The elapsed time is a second, independent check that the controller
         * is really driving the bus rather than timing out on every address.
         *
         * 112 addresses at 100 kHz, each costing start + 9 bits + stop, is about
         * 35 ms of wire time. A scan that returns in tens of milliseconds did
         * real NACKs; one that takes seconds per address is hitting the
         * controller timeout and the "0 devices" means nothing. */
        rt_kprintf("i2cscan: %d device(s) on %s in %u ms "
                   "(nack=%d timeout=%d other=%d)\n", found, name,
                   (unsigned)((rt_tick_get() - t0) * 1000u / RT_TICK_PER_SECOND),
                   nacks, timeouts, other);

        /* The three counts answer three different questions and only one of them
         * means "the bus works":
         *
         *   nack    - the address byte was clocked out and nothing acknowledged
         *             it. That is what an empty bus looks like, and it proves the
         *             controller drove SCL.
         *   timeout - the command never retired. Nothing was clocked out, so the
         *             zero is meaningless - the bus is not running at all.
         *   other   - driver refused the transfer before it started.
         *
         * Without this split, "0 device(s)" reads as success on a bus that never
         * moved a single edge. */
        if (timeouts > 0)
        {
            rt_kprintf("i2cscan: FAIL, %d address(es) timed out - the "
                       "controller is not completing transfers\n", timeouts);
            return -1;
        }
    }

    return 0;
}
MSH_CMD_EXPORT_ALIAS(cmd_i2cscan, i2cscan, scan an I2C bus for ACKing addresses);














