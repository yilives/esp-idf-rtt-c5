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

#include "port_lp_i2c.h"

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
