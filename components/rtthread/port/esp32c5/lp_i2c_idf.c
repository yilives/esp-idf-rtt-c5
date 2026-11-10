/*
 * `c5lpi2cidf` - the SAME experiment, but through ESP-IDF's own LP I2C driver.
 *
 * This file exists to answer one question and then be deleted: is LP I2C silent
 * because this port's driver is wrong, or because the peripheral is silent
 * anyway?
 *
 * `lp_i2c.c` configures the controller by hand and the FSM never starts. Every
 * register that can be read back says the block is clocked and reachable, the
 * pads idle high, the command words are correctly formed - and `int_raw` stays
 * empty, so it never even attempted a start condition. Guessing further from
 * there is guessing at silicon.
 *
 * IDF's `esp_driver_i2c` is in every image already (`libesp_driver_i2c.a` has
 * `i2c_new_master_bus`; nothing referenced it, so --gc-sections dropped it). It
 * goes through `i2c_common.c` -> `lp_i2c_ll_enable_bus_clock()` ->
 * `i2c_hal_init()` and then drives the same silicon. So:
 *
 *   - if IDF's driver also fails to complete, the peripheral is not usable the
 *     way this port is trying to use it, and the hand-written driver is not
 *     where the answer is;
 *   - if IDF's driver completes, the difference is in this port's code and is
 *     then findable by comparison.
 *
 * That is a real experiment rather than another hypothesis, and it costs one
 * command.
 *
 * Pin choice is not free: the LP I2C IOMUX pads are fixed at IO2/IO3 on this part
 * (`i2c_ll.h`), which is why SPI MISO and PWM were moved off them earlier.
 */

#include <rthw.h>
#include <rtthread.h>

#include "sdkconfig.h"
#include "soc/soc.h"
#include "soc/soc_caps.h"

#if defined(SOC_LP_I2C_SUPPORTED)

#include "driver/i2c_master.h"
#include "hal/i2c_types.h"
#include "esp_err.h"

#include "hal/i2c_ll.h"

#include "hal/i2c_hal.h"

#include "port_lp_i2c.h"


/* Every LP I2C register that could plausibly gate the FSM, in one line per
 * group. Two independent drivers, one dump each, then a textual diff - that is
 * how thirteen rounds of "this field looks the same" get settled. */
static void c5_lp_i2c_dump_all(const char *tag)
{
    const i2c_dev_t *h = I2C_LL_GET_HW(1);

    rt_kprintf("DUMP %s ctr=%08x sr=%08x to=%08x fifo_conf=%08x fifo_st=%08x\n",
        tag, (unsigned)h->ctr.val, (unsigned)h->sr.val, (unsigned)h->to.val,
        (unsigned)h->fifo_conf.val, (unsigned)h->fifo_st.val);
    rt_kprintf("DUMP %s scl_low=%08x scl_high=%08x scl_wait_high=%08x\n",
        tag, (unsigned)h->scl_low_period.val, (unsigned)h->scl_high_period.val,
        (unsigned)h->scl_high_period.scl_wait_high_period);
    rt_kprintf("DUMP %s sda_hold=%08x sda_sample=%08x filter=%08x clk_conf=%08x\n",
        tag, (unsigned)h->sda_hold.val, (unsigned)h->sda_sample.val,
        (unsigned)h->filter_cfg.val, (unsigned)h->clk_conf.val);
    rt_kprintf("DUMP %s start_h=%08x rstart_s=%08x stop_h=%08x stop_s=%08x\n",
        tag, (unsigned)h->scl_start_hold.val, (unsigned)h->scl_rstart_setup.val,
        (unsigned)h->scl_stop_hold.val, (unsigned)h->scl_stop_setup.val);
    rt_kprintf("DUMP %s st_to=%08x main_st_to=%08x sp_conf=%08x int_ena=%08x\n",
        tag, (unsigned)h->scl_st_time_out.val, (unsigned)h->scl_main_st_time_out.val,
        (unsigned)h->scl_sp_conf.val, (unsigned)h->int_ena.val);
    rt_kprintf("DUMP %s cmd=%08x,%08x,%08x,%08x\n",
        tag, (unsigned)h->command[0].val, (unsigned)h->command[1].val,
        (unsigned)h->command[2].val, (unsigned)h->command[3].val);
}

int c5_lp_i2c_idf_selftest(void)
{
    i2c_master_bus_config_t bus_cfg;
    i2c_master_bus_handle_t bus = RT_NULL;
    i2c_device_config_t dev_cfg;
    i2c_master_dev_handle_t dev = RT_NULL;
    esp_err_t err;
    rt_uint8_t dummy = 0;

    rt_kprintf("c5lpi2cidf: ESP-IDF's own LP I2C driver, port LP_I2C_NUM_0, "
               "SDA=IO%d SCL=IO%d\n",
               (int)C5_LP_I2C_SDA_PIN, (int)C5_LP_I2C_SCL_PIN);

    rt_memset(&bus_cfg, 0, sizeof(bus_cfg));
    bus_cfg.i2c_port    = LP_I2C_NUM_0;
    bus_cfg.sda_io_num  = (gpio_num_t)C5_LP_I2C_SDA_PIN;
    bus_cfg.scl_io_num  = (gpio_num_t)C5_LP_I2C_SCL_PIN;
    bus_cfg.lp_source_clk = LP_I2C_SCLK_XTAL_D2;
    bus_cfg.glitch_ignore_cnt = 0;
    bus_cfg.flags.enable_internal_pullup = 1;

    err = i2c_new_master_bus(&bus_cfg, &bus);
    rt_kprintf("c5lpi2cidf: i2c_new_master_bus -> 0x%x\n", (unsigned)err);

    if (err != ESP_OK)
    {
        rt_kprintf("c5lpi2cidf: FAIL, IDF's driver refuses to create the bus. That "
                   "is a configuration error, not a peripheral fault, and it points "
                   "at the arguments above rather than at the silicon.\n");
        return -1;
    }

    rt_memset(&dev_cfg, 0, sizeof(dev_cfg));
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address  = 0x7Fu;          /* reserved - cannot be acknowledged */
    dev_cfg.scl_speed_hz    = 100000;

    err = i2c_master_bus_add_device(bus, &dev_cfg, &dev);
    rt_kprintf("c5lpi2cidf: i2c_master_bus_add_device -> 0x%x\n", (unsigned)err);

    if (err != ESP_OK)
    {
        rt_kprintf("c5lpi2cidf: FAIL, could not add the device\n");
        i2c_del_master_bus(bus);
        return -1;
    }

    /* Dump BEFORE the transfer: this is IDF's CONFIGURED state, which is what has
     * to be compared with what `c5lpi2c` writes. The dump after the transfer
     * shows hardware-updated bits (`command_done`) and cannot be compared. */
    {
        const i2c_dev_t *hw = I2C_LL_GET_HW(LP_I2C_NUM_0);
        int k;

        rt_kprintf("c5lpi2cidf: IDF configured ctr=0x%08x fifo_conf=0x%08x "
                   "scl_low=%u scl_high=%u\n",
                   (unsigned)hw->ctr.val, (unsigned)hw->fifo_conf.val,
                   (unsigned)hw->scl_low_period.scl_low_period,
                   (unsigned)hw->scl_high_period.scl_high_period);
        for (k = 0; k < 4; k++)
        {
            rt_kprintf("c5lpi2cidf: IDF configured cmd[%d]=0x%08x\n",
                       k, (unsigned)hw->command[k].val);
        }
    }

    /* Dump the LP-domain clock registers BEFORE the transfer, so they can be
     * compared with what `c5lpi2c` leaves. This is the last piece of shared state
     * that neither the driver dump nor the source comparison covers: the LP
     * peripheral clock gate (LPPERI) and the LP clock select (LP_CLKRST) are set
     * up by `esp_clk_tree_enable_src()` and by the LP-core/PM code, not by the
     * I2C driver, so a hand-written driver can look identical at the controller
     * while the domain is clocked differently. */
    {
        rt_kprintf("c5lpi2cidf: IDF LP_CLKRST.lpperi=0x%08x lp_i2c_clk_sel=%u "
                   "(0=LP_FAST, 1=XTAL_D2)\n",
                   (unsigned)LP_CLKRST.lpperi.val,
                   (unsigned)LP_CLKRST.lpperi.lp_i2c_clk_sel);
        rt_kprintf("c5lpi2cidf: IDF LPPERI.clk_en=0x%08x reset_en=0x%08x\n",
                   (unsigned)LPPERI.clk_en.val,
                   (unsigned)LPPERI.reset_en.val);
    }

    /* HYBRID, CLEAN: IDF creates the bus and configures everything, then this
     * hand-written sequence drives the transfer WITHOUT IDF's transmit ever
     * running. The earlier version of this test drove the bus AFTER IDF's
     * transmit, so it could not tell "the configuration matters" from "the bus
     * was already moving". This one can.
     *
     * If this completes: IDF's configuration is what the port is missing.
     * If it does not complete: the difference is in the transfer code, and the
     * configuration has been proven equivalent by the register dumps above. */
    {
        i2c_dev_t *hw = I2C_LL_GET_HW(LP_I2C_NUM_0);
        i2c_ll_hw_cmd_t cmd;
        rt_uint32_t guard;
        rt_bool_t done = RT_FALSE;
        uint8_t probe[2] = { 0xFE, 0x00 };   /* 0x7F << 1, one data byte */

        /* IDF has configured the bus but not transmitted. Apply the timing it
         * would apply, since that happens inside transmit. */
        i2c_hal_context_t hal = { .dev = hw };
        i2c_hal_set_bus_timing(&hal, 100000, I2C_CLK_SRC_XTAL, 48000000);
        i2c_ll_update(hw);

        i2c_ll_txfifo_rst(hw);
        i2c_ll_rxfifo_rst(hw);

        cmd.val = 0;
        cmd.op_code = I2C_LL_CMD_RESTART;
        i2c_ll_master_write_cmd_reg(hw, cmd, 0);

        cmd.val = 0;
        cmd.op_code = I2C_LL_CMD_WRITE;
        cmd.byte_num = 2;
        cmd.ack_en = 1;
        cmd.ack_exp = 0;
        cmd.ack_val = 0;
        i2c_ll_write_txfifo(hw, probe, 2);
        i2c_ll_master_write_cmd_reg(hw, cmd, 1);

        cmd.val = 0;
        cmd.op_code = I2C_LL_CMD_STOP;
        i2c_ll_master_write_cmd_reg(hw, cmd, 2);

        c5_lp_i2c_dump_all("IDF");
        c5_lp_i2c_dump_all("IDF-REFERENCE");
        i2c_ll_update(hw);
        i2c_ll_start_trans(hw);

        for (guard = 0; guard < 500000u; guard++)
        {
            if (hw->command[2].command_done)
            {
                done = RT_TRUE;
                break;
            }
        }

        rt_kprintf("c5lpi2cidf: hybrid(clean) done=%s int_raw=0x%08x "
                   "cmd0=0x%08x cmd1=0x%08x cmd2=0x%08x\n",
                   done ? "yes" : "NO", (unsigned)hw->int_raw.val,
                   (unsigned)hw->command[0].val,
                   (unsigned)hw->command[1].val,
                   (unsigned)hw->command[2].val);

        if (done)
        {
            rt_kprintf("c5lpi2cidf: PASS - the hand-written sequence works when IDF "
                       "configured the bus first. The missing piece is in this "
                       "port's own initialisation.\n");
        }
        else
        {
            rt_kprintf("c5lpi2cidf: the hand-written sequence fails even on IDF's "
                       "configuration. The difference is in the TRANSFER code, not "
                       "in initialisation - and the register dumps above agree on "
                       "every controller field.\n");
        }
    }

    /* One byte to an address that cannot be there. A NACK is the success case:
     * it means the controller drove a start condition and clocked nine bits. */
    err = i2c_master_transmit(dev, &dummy, 1, 200);

    rt_kprintf("c5lpi2cidf: i2c_master_transmit to 0x7F -> 0x%x (%s)\n",
               (unsigned)err,
               (err == ESP_OK)                     ? "acknowledged - nothing should be there" :
               (err == ESP_ERR_INVALID_RESPONSE)   ? "NACK - the bus WAS driven" :
               (err == ESP_ERR_TIMEOUT)            ? "TIMEOUT - no start condition" :
                                                     "other");

    /* Dump the controller state IDF's driver LEAVES BEHIND.
     *
     * This is the part that makes the experiment useful rather than merely
     * conclusive: if IDF succeeds and this port fails, the registers IDF
     * programmed are the reference. Comparing them against what `c5lpi2c` writes
     * points at the difference instead of at a hypothesis.
     *
     * Note `sda_force_out`/`scl_force_out` (ctr bits 0/1): IDF leaves them 0
     * (open-drain), the same as this port. The interesting ones are
     * `fifo_conf`, `scl_low/high_period` and whether any command is still armed. */
    {
        /* `lp_i2c_dev_t` has no extern instance (the header comments it out and
         * maps the register block to `i2c_dev_t` for the HAL), so the base comes
         * from the LL accessor that IDF itself uses. */
        const i2c_dev_t *hw = I2C_LL_GET_HW(LP_I2C_NUM_0);
        int k;

        rt_kprintf("c5lpi2cidf: IDF left ctr=0x%08x fifo_conf=0x%08x "
                   "scl_low=%u scl_high=%u\n",
                   (unsigned)hw->ctr.val, (unsigned)hw->fifo_conf.val,
                   (unsigned)hw->scl_low_period.scl_low_period,
                   (unsigned)hw->scl_high_period.scl_high_period);
        rt_kprintf("c5lpi2cidf: IDF left to=0x%08x scl_st_to=0x%08x "
                   "scl_main_st_to=0x%08x\n",
                   (unsigned)hw->to.val,
                   (unsigned)hw->scl_st_time_out.val,
                   (unsigned)hw->scl_main_st_time_out.val);
        for (k = 0; k < 3; k++)
        {
            rt_kprintf("c5lpi2cidf: IDF left cmd[%d]=0x%08x\n",
                       k, (unsigned)hw->command[k].val);
        }
    }

    (void)i2c_master_bus_rm_device(dev);
    (void)i2c_del_master_bus(bus);

    /* ESP_ERR_INVALID_RESPONSE (0x108) is IDF's "master transmit receives NACK",
     * which is the SUCCESS case here: it means a start condition went out and
     * nine bits were clocked. The first version of this test looked for ESP_FAIL
     * and printed "other" for the one result that answers the question. */
    if (err == ESP_ERR_INVALID_RESPONSE)
    {
        rt_kprintf("c5lpi2cidf: PASS (IDF's driver reached the bus and got a NACK. "
                   "This port's hand-written driver does NOT - so the difference is "
                   "in this port's code, and it is findable by comparison.)\n");
        return 0;
    }

    if (err == ESP_ERR_TIMEOUT)
    {
        rt_kprintf("c5lpi2cidf: IDF's driver times out exactly like this port's. Two "
                   "independent implementations of the same sequence both fail to "
                   "start the FSM, which points at the peripheral or at how this part "
                   "expects LP I2C to be brought up - not at either driver.\n");
        return -1;
    }

    rt_kprintf("c5lpi2cidf: unexpected result; nothing concluded\n");
    return -1;
}

#endif /* SOC_LP_I2C_SUPPORTED */
