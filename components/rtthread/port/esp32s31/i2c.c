/*
 * ESP32-S31 I2C master driver: rt_i2c_bus_device_ops over the I2C LL.
 *
 * Same reasoning as the UART, GPIO, PWM and TIMG drivers in this directory:
 * esp_driver_i2c calls FreeRTOS primitives (its command queue is an xQueue and
 * its ISR uses xQueueSendFromISR), so the LL layer is both smaller and directly
 * auditable. Everything below touches registers only.
 *
 * Three S31 specific points, all of which are silent failures if skipped:
 *
 * 1. The I2C clock is NOT the APB clock. S31 routes it through HP_SYS_CLKRST
 *    with its own divider, and i2c_ll_master_cal_bus_clk() computes the divider
 *    plus the SCL timing from the *source* frequency, which therefore has to be
 *    passed in rather than assumed. It is XTAL (40 MHz) here, so the bus rate
 *    does not move when DFS changes the CPU/APB frequency.
 *
 * 2. register writes only take effect after i2c_ll_update() (ctr.conf_upgate).
 *    IDF calls it from i2c_hal_master_trans_start(); forgetting it leaves the
 *    old timing in place and the bus runs at whatever the divider was before.
 *
 * 3. The command list is a hardware state machine: each entry in
 *    hw->command[] carries op_code/byte_num/ack_en and a `done` bit the
 *    hardware sets as it retires them. A transfer has to end with an END
 *    command, and the driver has to wait for END's done bit rather than for a
 *    duration - a missing ACK shows up as the END command never completing.
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#include "soc/soc.h"
#include "soc/soc_caps.h"
#include "soc/gpio_sig_map.h"
#include "soc/io_mux_reg.h"
#include "soc/i2c_struct.h"
#include "hal/gpio_ll.h"
#include "hal/i2c_ll.h"
#include "hal/i2c_types.h"
#include "esp_rom_gpio.h"

#if defined(SOC_I2C_SUPPORTED)

#include "port_i2c.h"

/* HP I2C instances. SOC_I2C_NUM counts the LP one too, which has a different
 * register block and is out of scope here. */
#define S31_I2C_HP_NUM      2

/* The I2C peripheral clock source. XTAL is exact (40 MHz), APB moves with DFS -
 * and a bus rate that silently changes under a peripheral is the kind of bug
 * that only shows up as an intermittent NACK. */
#define S31_I2C_SRC_HZ      (CONFIG_XTAL_FREQ * 1000000u)

struct s31_i2c
{
    struct rt_i2c_bus_device parent;
    i2c_dev_t               *hw;
    rt_uint32_t              port;
    rt_uint32_t              freq;
};

static struct s31_i2c s_i2c[S31_I2C_HP_NUM];

static i2c_dev_t *s31_i2c_hw(rt_uint32_t port)
{
    return (port == 0) ? &I2C0 : &I2C1;
}

/* Route the pads: IO_MUX to GPIO function, then the matrix carries the I2C
 * signals out. Directions are fixed by the peripheral (SDA is bidirectional),
 * so only the output-enable source needs pointing at the peripheral. */
static rt_err_t s31_i2c_set_pins(rt_uint32_t port, rt_base_t scl, rt_base_t sda)
{
    if (scl < 0 || sda < 0 || scl >= SOC_GPIO_PIN_COUNT || sda >= SOC_GPIO_PIN_COUNT ||
        ((SOC_GPIO_VALID_GPIO_MASK >> scl) & 1ULL) == 0 ||
        ((SOC_GPIO_VALID_GPIO_MASK >> sda) & 1ULL) == 0)
    {
        return -RT_EINVAL;
    }

    /* Both pads: GPIO function, input on (SDA is read back even while driven),
     * pull-ups on. The internal pull-up is weak (~45 kOhm) - fine for a bench
     * test, not for a real bus; an external 4.7 kOhm is still required. */
    gpio_ll_func_sel(&GPIO, (uint8_t)scl, PIN_FUNC_GPIO);
    gpio_ll_func_sel(&GPIO, (uint8_t)sda, PIN_FUNC_GPIO);
    gpio_ll_pullup_en(&GPIO, scl);
    gpio_ll_pullup_en(&GPIO, sda);
    gpio_ll_input_enable(&GPIO, scl);
    gpio_ll_input_enable(&GPIO, sda);
    gpio_ll_od_enable(&GPIO, scl);
    gpio_ll_od_enable(&GPIO, sda);

    /* Output-enable comes from the peripheral, not the GPIO register. */
    gpio_ll_set_output_enable_ctrl(&GPIO, (uint8_t)scl, true, true);
    gpio_ll_set_output_enable_ctrl(&GPIO, (uint8_t)sda, true, true);

    esp_rom_gpio_connect_out_signal((uint32_t)scl,
                                    (port == 0) ? I2C0_SCL_PAD_OUT_IDX : I2C1_SCL_PAD_OUT_IDX,
                                    false, false);
    esp_rom_gpio_connect_out_signal((uint32_t)sda,
                                    (port == 0) ? I2C0_SDA_PAD_OUT_IDX : I2C1_SDA_PAD_OUT_IDX,
                                    false, false);
    esp_rom_gpio_connect_in_signal((uint32_t)scl,
                                   (port == 0) ? I2C0_SCL_PAD_IN_IDX : I2C1_SCL_PAD_IN_IDX,
                                   false);
    esp_rom_gpio_connect_in_signal((uint32_t)sda,
                                   (port == 0) ? I2C0_SDA_PAD_IN_IDX : I2C1_SDA_PAD_IN_IDX,
                                   false);

    return RT_EOK;
}

/* Program divider + SCL timing for a bus frequency. */
static void s31_i2c_apply_freq(struct s31_i2c *bus, rt_uint32_t freq)
{
    i2c_hal_clk_config_t clk = {0};

    if (freq == 0 || freq > 1000000u)
    {
        freq = 100000u;
    }

    i2c_ll_master_cal_bus_clk(S31_I2C_SRC_HZ, freq, &clk);
    i2c_ll_master_set_bus_timing(bus->hw, &clk);
    i2c_ll_set_scl_clk_timing(bus->hw, clk.scl_high, clk.scl_low, clk.scl_wait_high);
    i2c_ll_master_set_start_timing(bus->hw, clk.setup, clk.hold);
    i2c_ll_master_set_stop_timing(bus->hw, clk.setup, clk.hold);
    i2c_ll_set_sda_timing(bus->hw, clk.sda_sample, clk.sda_hold);
    i2c_ll_set_tout(bus->hw, clk.tout);
    i2c_ll_update(bus->hw);

    bus->freq = freq;
}

/*
 * One message. Returns RT_EOK, or -RT_EIO when the slave did not ACK.
 *
 * The command list has at most 8 entries and a message is at most
 * { RSTART, WRITE addr, N x (WRITE|READ) , STOP, END }, which fits because each
 * command can carry up to 255 bytes (byte_num is 8 bits). A 1-byte write and a
 * 1-byte read therefore cost one command each, not one per byte.
 */
static rt_err_t s31_i2c_xfer_msg(struct s31_i2c *bus, struct rt_i2c_msg *msg)
{
    i2c_dev_t *hw = bus->hw;
    rt_uint32_t idx = 0;
    rt_uint8_t  addr = (rt_uint8_t)(msg->addr << 1);
    i2c_ll_hw_cmd_t cmd;
    rt_uint32_t guard;
    rt_bool_t   nack = RT_FALSE;

    /* Clear stale state: a previous timeout can leave commands pending and the
     * bus-busy flag set. */
    i2c_ll_txfifo_rst(hw);
    i2c_ll_rxfifo_rst(hw);
    i2c_ll_clear_intr_mask(hw, 0xFFFFFFFFu);

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
        if (msg->len > 1)
        {
            /* Everything but the final byte is ACKed; the hardware applies
             * ack_val to the last byte, and IDF splits the command when len > 1
             * only for that reason - here ack_val=1 covers it. */
            cmd.ack_val = 1;
        }
        i2c_ll_master_write_cmd_reg(hw, cmd, (int)idx++);
    }
    else
    {
        cmd.val         = 0;
        cmd.op_code     = I2C_LL_CMD_WRITE;
        cmd.byte_num    = (rt_uint8_t)msg->len;
        cmd.ack_en      = 1;
        cmd.ack_exp     = 0;
        i2c_ll_master_write_cmd_reg(hw, cmd, (int)idx++);
        i2c_ll_write_txfifo(hw, msg->buf, msg->len);
    }

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

    /*
     * Wait for the END command to retire, with a bound derived from the bus
     * speed rather than a fixed loop count.
     *
     * The first version used 2,000,000 iterations. Harmless for one transfer,
     * catastrophic for a bus scan: 112 addresses that do not ACK (the normal
     * case on an empty bus) multiplied by that bound made `i2cscan` look like it
     * had hung. A message of `len` bytes takes at most (len + 2) * 9 SCL
     * periods; at 100 kHz that is tens of microseconds, so a few milliseconds is
     * already generous.
     *
     * A missing ACK makes the controller abandon the remaining commands, so END
     * never sets command_done - which is why the status bits are checked inside
     * the loop and not just after it.
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
            rt_uint32_t st = hw->int_status.val;

            /* NACK (bit 1) or the bus held low past the timeout (bit 5):
             * either way the transfer is over and failed. */
            if (st & (0x2u | 0x20u))
            {
                break;
            }

            if (++guard > budget)
            {
                i2c_ll_clear_intr_mask(hw, 0xFFFFFFFFu);
                i2c_ll_txfifo_rst(hw);
                i2c_ll_rxfifo_rst(hw);
                return -RT_ETIMEOUT;
            }
        }
    }

    /* ack_en on a WRITE command is the ACK *checker*: if the slave did not
     * drive SDA low, the hardware records it in int_status. */
    {
        rt_uint32_t st = hw->int_status.val;

        if (st & 0x1u)              /* I2C_NACK_INT_ST */
        {
            nack = RT_TRUE;
        }
    }

    i2c_ll_clear_intr_mask(hw, 0xFFFFFFFFu);

    if (msg->flags & RT_I2C_RD)
    {
        if (nack)
        {
            return -RT_EIO;
        }

        {
            uint32_t cnt = 0;

            i2c_ll_get_rxfifo_cnt(hw, &cnt);
            if (cnt < msg->len)
            {
                return -RT_EIO;     /* short read: bus glitch or NACK */
            }
        }

        i2c_ll_read_rxfifo(hw, msg->buf, (uint8_t)msg->len);
    }
    else if (nack)
    {
        return -RT_EIO;
    }

    return RT_EOK;
}

static rt_ssize_t s31_i2c_master_xfer(struct rt_i2c_bus_device *bus,
                                      struct rt_i2c_msg msgs[], rt_uint32_t num)
{
    struct s31_i2c *dev = (struct s31_i2c *)bus;
    rt_uint32_t i;

    for (i = 0; i < num; i++)
    {
        rt_err_t rc;

        /* 10-bit addressing is supported by the hardware but not implemented
         * here: the address phase would need I2C_LL_CMD_WRITE with two bytes.
         * Refusing is better than sending a wrong address. */
        if (msgs[i].addr > 0x7F || msgs[i].len > 255)
        {
            return -RT_EINVAL;
        }

        rc = s31_i2c_xfer_msg(dev, &msgs[i]);
        if (rc != RT_EOK)
        {
            return (i == 0) ? rc : (rt_ssize_t)i;
        }
    }

    return (rt_ssize_t)num;
}

static rt_err_t s31_i2c_bus_control(struct rt_i2c_bus_device *bus, int cmd, void *args)
{
    struct s31_i2c *dev = (struct s31_i2c *)bus;

    switch (cmd)
    {
    case RT_I2C_DEV_CTRL_CLK:
        if (args == RT_NULL)
        {
            return -RT_EINVAL;
        }
        s31_i2c_apply_freq(dev, *(rt_uint32_t *)args);
        return RT_EOK;

    case RT_I2C_DEV_CTRL_TIMEOUT:
        /* Not implemented as a settable value: the timeout is derived from the
         * bus frequency by i2c_ll_master_cal_bus_clk(), which is the value the
         * hardware needs for its own counting. */
        return RT_EOK;

    default:
        return -RT_ENOSYS;
    }
}

static const struct rt_i2c_bus_device_ops s31_i2c_ops =
{
    .master_xfer     = s31_i2c_master_xfer,
    .slave_xfer      = RT_NULL,
    .i2c_bus_control = s31_i2c_bus_control,
};

static rt_err_t s31_i2c_init_port(rt_uint32_t port, rt_base_t scl, rt_base_t sda,
                                  rt_uint32_t freq)
{
    struct s31_i2c *bus = &s_i2c[port];
    char name[8];

    bus->hw   = s31_i2c_hw(port);
    bus->port = port;

    if (s31_i2c_set_pins(port, scl, sda) != RT_EOK)
    {
        return -RT_EINVAL;
    }

    /* Bus clock, register reset, controller clock. The S31 gates all three
     * through HP_SYS_CLKRST. */
    i2c_ll_enable_bus_clock((int)port, true);
    i2c_ll_reset_register((int)port);

    i2c_ll_enable_fifo_mode(bus->hw, true);
    i2c_ll_set_data_mode(bus->hw, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
    i2c_ll_set_txfifo_empty_thr(bus->hw, 4);
    i2c_ll_set_rxfifo_full_thr(bus->hw, 8);

    s31_i2c_apply_freq(bus, freq ? freq : 100000u);

    /* Master mode: ctr.ms_mode = 1. */
    bus->hw->ctr.ms_mode = 1;
    i2c_ll_update(bus->hw);

    bus->parent.ops    = &s31_i2c_ops;
    bus->parent.timeout = 100;

    name[0] = 'i';
    name[1] = '2';
    name[2] = 'c';
    name[3] = (char)('0' + port);
    name[4] = '\0';

    return rt_i2c_bus_device_register(&bus->parent, name);
}

/* ------------------------------------------------------------------ msh -- */

static int cmd_i2c(int argc, char **argv)
{
    rt_uint32_t port = 0;
    struct rt_i2c_bus_device *bus;
    char name[8];
    int i;

    if (argc >= 2)
    {
        port = (rt_uint32_t)atoi(argv[1]);
    }

    if (port >= S31_I2C_HP_NUM)
    {
        rt_kprintf("usage: i2cscan [port 0..%d]\n", S31_I2C_HP_NUM - 1);
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

    rt_kprintf("i2cscan: probing %s (%u Hz)\n", name, (unsigned)s_i2c[port].freq);

    {
        int found = 0;

        for (i = 0x08; i <= 0x77; i++)
        {
            struct rt_i2c_msg msg;
            rt_uint8_t dummy = 0;

            /* A zero-length write is exactly "does this address ACK", which is
             * what every bus scanner does; the hardware still sends the address
             * byte and checks the ACK. */
            msg.addr  = (rt_uint16_t)i;
            msg.flags = RT_I2C_WR;
            msg.buf   = &dummy;
            msg.len   = 0;

            if (rt_i2c_transfer(bus, &msg, 1) == 1)
            {
                rt_kprintf("  0x%02x ACK\n", i);
                found++;
            }
        }

        rt_kprintf("i2cscan: %d device(s) on %s\n", found, name);
    }

    return 0;
}
MSH_CMD_EXPORT_ALIAS(cmd_i2c, i2cscan, scan an I2C bus for ACKing addresses);

rt_err_t rt_hw_i2c_init(void)
{
    /* S31 default IO_MUX pins are not used here on purpose: the pads are
     * configurable and a wrong guess drives somebody else's peripheral. These
     * two are free GPIOs on the devkit and are documented in port_i2c.h. */
    rt_err_t rc = s31_i2c_init_port(0, S31_I2C0_SCL_PIN, S31_I2C0_SDA_PIN, 100000u);

    if (rc != RT_EOK)
    {
        rt_kprintf("i2c0: init failed %d\n", rc);
    }

    return rc;
}
INIT_BOARD_EXPORT(rt_hw_i2c_init);

#endif /* SOC_I2C_SUPPORTED */

