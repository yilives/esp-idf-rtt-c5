/*
 * ESP32-C5 LP I2C 鈥?the low-power-domain I2C controller, as `lp_i2c0`.
 *
 * This is the LP peripheral work the user asked for. The important discovery is
 * that LP I2C is not a foreign register block bolted onto the side: on this part
 * `I2C_LL_GET_HW(1)` is `&LP_I2C`, and the same `i2c_ll.h` drives it. What differs
 * is the pads and the clock domain.
 *
 * ---------------------------------------------------------------------------
 * What had to be true before a single register could be written, and how each
 * one was established rather than assumed:
 *
 * 1. **The pads are fixed.** `i2c_ll.h` says so outright:
 *
 *        #define LP_I2C_SDA_IOMUX_PAD 2
 *        #define LP_I2C_SCL_IOMUX_PAD 3
 *
 *    and `i2c_common.c:376` refuses anything else with
 *    "SCL pin can only be configured as GPIO#3". So LP I2C can only ever be on
 *    IO2/IO3 鈥?this is a hardware constraint, not a choice.
 *
 * 2. **IO2 and IO3 had to be freed.** This port had SPI MISO on IO2 and the PWM
 *    test on IO3. Both are re-routable (GPIO matrix, all pads) and LP I2C is not,
 *    so SPI MISO moved to IO23 and PWM to IO24. `c5probe` was written to confirm
 *    IO23-28 actually respond on this package before relying on them: all six do.
 *
 * 3. **The pads go through `rtc_io`, not the GPIO matrix.** `SOC_LP_GPIO_MATRIX_
 *    SUPPORTED` is not defined for the C5, so IDF takes the `!SUPPORTED` branch
 *    and calls `rtc_gpio_iomux_input(pin, iomux_func, signal)`. The IOMUX function
 *    for LP I2C is 3 (`LP_I2C_MUX_FUNC` in i2c_periph.c).
 *
 * 4. **The clock is a different one.** Not PCR but LPPERI:
 *    `lp_i2c_ll_enable_bus_clock()` and `lp_i2c_ll_set_source_clk()` 鈥?the latter
 *    selects XTAL/2 or the LP fast clock, and the LP fast clock is the one that
 *    keeps running when the HP domain is powered down, which is the whole point.
 *
 * ---------------------------------------------------------------------------
 * Verification. There is no LP I2C slave on this board, so a bus scan reports
 * zero devices and proves almost nothing 鈥?a dead bus and an empty bus look
 * identical that way. What IS provable without a slave:
 *
 *   - the controller comes out of reset and its clock is running;
 *   - the pads are released by the peripheral and read high through their
 *     pull-ups, which means the LP IO mux actually handed them over;
 *   - a transfer to an absent address NACKs, which requires the controller to
 *     have driven a start condition and clocked nine bits onto a real bus.
 *
 * That third point is the one a dead controller cannot fake: "NACK" is an
 * observation about the wire, while "no device" is not.
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#include "sdkconfig.h"
#include "soc/soc.h"
#include "soc/soc_caps.h"
#include "soc/lp_i2c_struct.h"
#include "soc/lpperi_struct.h"
#include "soc/lp_clkrst_struct.h"
#include "soc/rtc_io_channel.h"
#include "hal/i2c_ll.h"
#include "hal/rtc_io_ll.h"
#include "hal/gpio_ll.h"
#include "soc/gpio_sig_map.h"
#include "esp_private/periph_ctrl.h"
#include "esp_private/esp_clk.h"
#include "esp_attr.h"

#if defined(SOC_LP_I2C_SUPPORTED)

#include "port_lp_i2c.h"

/* The controller is instance 1; `I2C_LL_GET_HW(1)` is `&LP_I2C`. */
#define C5_LP_I2C_HW_ID     1
#define C5_LP_I2C_MUX_FUNC  3   /* LP_I2C_MUX_FUNC, i2c_periph.c:10 */

struct c5_lp_i2c
{
    struct rt_i2c_bus_device parent;
    i2c_dev_t               *hw;
    rt_uint32_t              freq;
};

static struct c5_lp_i2c s_lp_i2c;
static rt_bool_t        s_lp_i2c_ready;

/*
 * Hand IO2/IO3 to the LP I2C IOMUX function.
 *
 * The order matters and it is the same order IDF uses: enable the LP IO clock,
 * put the pad on the digital side, set the IOMUX function, then enable input and
 * the open-drain output. Doing the function select before the clock leaves the
 * write dropped.
 */
static void c5_lp_i2c_set_pins(void)
{
    /* LP_GPIO's clock is in LPPERI and has to be on before any LP_IO_MUX write
     * lands; the register reads back its old value otherwise. */
    PERIPH_RCC_ATOMIC() { rtcio_ll_enable_io_clock(true); }

    /* SDA = IO2, SCL = IO3, both open-drain with the pull-up doing the high
     * level. `RTCIO_LL_FUNC_DIGITAL` puts the pad on the digital side - the LP
     * peripheral is a digital function even though it lives in the LP domain. */
    rtcio_ll_function_select(C5_LP_I2C_SDA_PIN, RTCIO_LL_FUNC_DIGITAL);
    rtcio_ll_function_select(C5_LP_I2C_SCL_PIN, RTCIO_LL_FUNC_DIGITAL);

    rtcio_ll_iomux_func_sel(C5_LP_I2C_SDA_PIN, C5_LP_I2C_MUX_FUNC);
    rtcio_ll_iomux_func_sel(C5_LP_I2C_SCL_PIN, C5_LP_I2C_MUX_FUNC);

    /* Drive the idle level HIGH before touching the output enable.
     *
     * This is the bug that made both lines read 0. `rtcio_ll_output_enable()`
     * turns on the RTC GPIO's own driver, and in open-drain mode that driver pulls
     * the pad LOW when its data register is 0 - which is the reset value. The bus
     * was therefore held low by the pad itself, and the pull-up had nothing to
     * pull against.
     *
     * Once the IOMUX function is 3 (not RTC), the output enable is handed to the
     * peripheral - `rtcio_hal_iomux_output()` says so outright: "as long as the
     * func sel is not RTC IO, the oe can only be controlled by the peripheral".
     * So the level is set high first and the RTC driver's enable is left alone.
     */
    rtcio_ll_set_level(C5_LP_I2C_SDA_PIN, 1);
    rtcio_ll_set_level(C5_LP_I2C_SCL_PIN, 1);

    rtcio_ll_input_enable(C5_LP_I2C_SDA_PIN);
    rtcio_ll_input_enable(C5_LP_I2C_SCL_PIN);
    rtcio_ll_output_mode_set(C5_LP_I2C_SDA_PIN, RTCIO_LL_OUTPUT_OD);
    rtcio_ll_output_mode_set(C5_LP_I2C_SCL_PIN, RTCIO_LL_OUTPUT_OD);

    /* Pull-ups on BOTH sides.
     *
     * `rtcio_ll_pullup_enable()` goes through the LP_AON block, and on the C5 the
     * pad's physical pull-up for IO2/IO3 is in the HP IO_MUX - the LP register
     * alone did not produce a high level, measured as SDA=0 SCL=0 with every LP
     * pull-up bit set. Enabling both costs two writes and removes the question. */
    rtcio_ll_pullup_enable(C5_LP_I2C_SDA_PIN);
    rtcio_ll_pullup_enable(C5_LP_I2C_SCL_PIN);
    rtcio_ll_pulldown_disable(C5_LP_I2C_SDA_PIN);
    rtcio_ll_pulldown_disable(C5_LP_I2C_SCL_PIN);

    gpio_ll_pullup_en(&GPIO, (uint8_t)C5_LP_I2C_SDA_PIN);
    gpio_ll_pullup_en(&GPIO, (uint8_t)C5_LP_I2C_SCL_PIN);
    gpio_ll_pulldown_dis(&GPIO, (uint8_t)C5_LP_I2C_SDA_PIN);
    gpio_ll_pulldown_dis(&GPIO, (uint8_t)C5_LP_I2C_SCL_PIN);

    /* The HP matrix must NOT also be driving these pads. This port's own pin
     * registry had SPI MISO on IO2 and PWM on IO3 until this round; leaving
     * either enabled would fight the LP peripheral for the pad and the bus would
     * read stuck. */
    gpio_ll_output_disable(&GPIO, (uint8_t)C5_LP_I2C_SDA_PIN);
    gpio_ll_output_disable(&GPIO, (uint8_t)C5_LP_I2C_SCL_PIN);
    gpio_ll_input_enable(&GPIO, (uint8_t)C5_LP_I2C_SDA_PIN);
    gpio_ll_input_enable(&GPIO, (uint8_t)C5_LP_I2C_SCL_PIN);
}

static void c5_lp_i2c_apply_freq(rt_uint32_t freq)
{
    i2c_hal_clk_config_t clk = {0};
    i2c_dev_t *hw = s_lp_i2c.hw;

    if (freq == 0u || freq > 1000000u)
    {
        freq = 100000u;
    }

    /* LP I2C has no fractional divider on this part and its source is XTAL/2.
     * Using esp_clk_xtal_freq() here would program twice the divisor and halve
     * the bus rate - a wrong clock that still produces a plausible-looking bus. */
    i2c_ll_master_cal_bus_clk((rt_uint32_t)esp_clk_xtal_freq() / 2u, freq, &clk);
    i2c_ll_master_set_bus_timing(hw, &clk);
    i2c_ll_set_scl_clk_timing(hw, clk.scl_high, clk.scl_low, clk.scl_wait_high);
    i2c_ll_master_set_start_timing(hw, clk.setup, clk.hold);
    i2c_ll_master_set_stop_timing(hw, clk.setup, clk.hold);
    i2c_ll_set_sda_timing(hw, clk.sda_sample, clk.sda_hold);
    i2c_ll_set_tout(hw, clk.tout);
    i2c_ll_update(hw);

    s_lp_i2c.freq = freq;
}

rt_err_t rt_hw_lp_i2c_init(void)
{
    if (s_lp_i2c_ready)
    {
        return RT_EOK;
    }

    /* LPPERI, not PCR: the LP I2C bus clock lives in the LP peripheral block and
     * survives the HP domain being powered down. */
    /* Reset FIRST, then enable, then pick the source.
     *
     * `lp_i2c_ll_reset_register()` pulses `LPPERI.reset_en.lp_ext_i2c_reset_en`,
     * and doing that after the bus clock is on leaves the block held in reset for
     * as long as the pulse lasts - which is fine - but doing it after
     * `ctr.clk_en` is set clears that bit again, because reset returns the whole
     * register file. The order here is reset, enable, source, then the
     * configuration writes below. */
    PERIPH_RCC_ATOMIC()
    {
        lp_i2c_ll_reset_register(C5_LP_I2C_HW_ID);
        lp_i2c_ll_enable_bus_clock(C5_LP_I2C_HW_ID, true);
        lp_i2c_ll_set_source_clk(&LP_I2C, LP_I2C_SCLK_XTAL_D2);
    }

    s_lp_i2c.hw = I2C_LL_GET_HW(C5_LP_I2C_HW_ID);

    c5_lp_i2c_set_pins();

    /* THE CONTROLLER CLOCK IS SEPARATE FROM THE BUS CLOCK, and skipping it is
     * invisible in exactly the way the HP driver records at i2c.c:453-456: every
     * register read-back looks correct while no transfer ever retires.
     *
     * Measured here: `ctr` read 0x00000208 with `clk_en` (bit 8) clear, all five
     * commands at `done=0`, `trans_start` never latching, and both pads idling
     * high - a completely plausible-looking bus that had never been clocked. The
     * HP driver had the same bug first and this is its fix.
     *
     * The rest mirrors i2c_hal_master_init(): master mode, open-drain pads,
     * arbitration off (single master here), receive-ack level. */
    /* DO NOT touch `ctr.clk_en`.
     *
     * Earlier rounds of this file wrote it on the reasoning that
     * `i2c_ll_enable_controller_clock()` is a no-op for LP_I2C (its body returns
     * early for that instance) and that `ctr.clk_en` was therefore the missing
     * gate. The register header says otherwise:
     *
     * (the header comment for that bit reads, verbatim: "clk_en : R/W; bitpos:
     * [8]; default: 0" followed by the single word "Reserved")
     *
     *
     * It is a RESERVED bit. IDF never writes it, and writing a reserved bit to
     * "make the clock work" is guessing at silicon, not programming it. If the LP
     * I2C is silent, the cause is somewhere the LL does expose - and the honest
     * move is to find it, not to poke at reserved bits until something changes.
     *
     * The LL still enables what IDF enables for this instance:
     * `lp_i2c_ll_enable_bus_clock()` (LPPERI) above, and `i2c_hal_init()`'s
     * `i2c_ll_enable_controller_clock()` below, which is a deliberate no-op here.
     */
    i2c_ll_enable_controller_clock(s_lp_i2c.hw, true);
    i2c_ll_set_mode(s_lp_i2c.hw, I2C_BUS_MODE_MASTER);
    i2c_ll_enable_pins_open_drain(s_lp_i2c.hw, true);
    i2c_ll_enable_arbitration(s_lp_i2c.hw, false);
    i2c_ll_master_rx_full_ack_level(s_lp_i2c.hw, false);

    /* FIFO thresholds left at their RESET VALUES (rxfifo_wm=6, txfifo_wm=2).
     *
     * IDF's driver never sets them for LP I2C - measured `fifo_conf=0x4046` both
     * before and after its transfer, which is exactly the reset value. This port
     * had been writing 4 and 8, giving `0x4088`. The watermark is not obviously
     * a reason for the FSM to refuse to start, but it is a difference, and
     * removing differences is cheaper than reasoning about which one matters. */
    i2c_ll_enable_fifo_mode(s_lp_i2c.hw, true);
    i2c_ll_set_data_mode(s_lp_i2c.hw, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);

    i2c_ll_set_source_clk(s_lp_i2c.hw, I2C_CLK_SRC_XTAL);

    /* Two calls IDF makes that this driver did not (i2c_master.c:133,136).
     *
     * `i2c_ll_disable_intr_mask()` matters more than it looks: the interrupt
     * mask register gates events into `int_st`, and IDF clears it before use.
     * This driver polls `int_raw`, so the mask should not matter - but a stale
     * mask from the HP driver's own init (this image runs `i2c0` too) is
     * exactly the kind of cross-driver state that produces "the FSM never ran"
     * with an empty status.
     *
     * `i2c_ll_master_set_filter()` with 0 is "no glitch filter", which is the
     * reset value; setting it explicitly removes the question. */
    i2c_ll_disable_intr_mask(s_lp_i2c.hw, 0xFFFFFFFFu);
    i2c_ll_clear_intr_mask(s_lp_i2c.hw, 0xFFFFFFFFu);
    i2c_ll_master_set_filter(s_lp_i2c.hw, 0);

    /* Timing is programmed per transfer, not here.
     *
     * Measured on IDF's own driver: before `i2c_master_transmit()` returns, the
     * bus timing registers read ZERO (`scl_low=0 scl_high=0`), and after it they
     * read 119/62. So IDF applies the timing inside the transfer, on the same
     * path that arms the commands. Doing it once at init - which is what the HP
     * driver does and what this port copied - is a real difference, and worth
     * removing before reasoning about the FSM. */
    c5_lp_i2c_apply_freq(C5_LP_I2C_FREQ);
    i2c_ll_update(s_lp_i2c.hw);

    /* `ctr` right after init, before any transfer touches it. The transfer's own
     * fsm_rst clears it, so reading only in the timeout dump cannot tell "the
     * configuration never took" from "the reset undid it". */
    rt_kprintf("c5lpi2c: after init ctr=0x%08x (clk_en=%u) LPPERI.clk_en=0x%08x "
               "(lp_ext_i2c_ck_en=%u)\n",
               (unsigned)LP_I2C.ctr.val, (unsigned)LP_I2C.ctr.clk_en,
               (unsigned)LPPERI.clk_en.val,
               (unsigned)LPPERI.clk_en.lp_ext_i2c_ck_en);
    rt_kprintf("c5lpi2c: LP_CLKRST.lpperi=0x%08x lp_i2c_clk_sel=%u\n",
               (unsigned)LP_CLKRST.lpperi.val,
               (unsigned)LP_CLKRST.lpperi.lp_i2c_clk_sel);

    s_lp_i2c_ready = RT_TRUE;
    return RT_EOK;
}

/* One message, same command sequence as the HP driver but on this controller. */
static rt_err_t c5_lp_i2c_xfer_msg(struct rt_i2c_msg *msg)
{
    i2c_dev_t *hw = s_lp_i2c.hw;
    rt_uint32_t idx = 0;
    rt_uint8_t addr = (rt_uint8_t)(msg->addr << 1);
    i2c_ll_hw_cmd_t cmd;
    rt_uint32_t guard;
    rt_bool_t nack = RT_FALSE;

    /* DO NOT reset the FIFOs or the FSM here.
     *
     * `i2c_ll_txfifo_rst()` and `i2c_ll_master_fsm_rst()` write self-clearing bits
     * in `fifo_conf`, and this controller is in the LP domain where those bits can
     * stay set. The HP driver does this per transfer and gets away with it; here it
     * would be the first thing to blame when the FSM never starts.
     *
     * Measured with them: ctr = 0x00000100, every `command` at done=0, int_raw
     * showing only the txfifo watermark default. Measured without them: identical.
     * So they are not the cause, and removing them costs nothing. */
    i2c_ll_clear_intr_mask(hw, 0xFFFFFFFFu);

    /* Same place IDF applies it: immediately before arming the transfer. */
    c5_lp_i2c_apply_freq(s_lp_i2c.freq ? s_lp_i2c.freq : C5_LP_I2C_FREQ);

    cmd.val = 0;
    cmd.op_code = I2C_LL_CMD_RESTART;
    i2c_ll_master_write_cmd_reg(hw, cmd, (int)idx++);

    /* THE ADDRESS AND THE FIRST DATA BYTE GO IN ONE WRITE COMMAND.
     *
     * This is the difference that made the peripheral look dead. The first
     * version emitted RESTART, WRITE(1 byte = address), WRITE(n bytes = data),
     * STOP, END - five commands, which is how the HP controller behaves and how
     * the S31 driver is written. On this LP controller the FSM never starts:
     * every command sits with `command_done` clear and `int_raw` empty.
     *
     * IDF's own driver (`i2c_master.c:194`) does:
     *
     *     hw_cmd.byte_num = data_fill + *address_fill;
     *     i2c_ll_write_txfifo(hal->dev, write_pr, data_fill);
     *     i2c_ll_master_write_cmd_reg(hal->dev, hw_cmd, i2c_master->cmd_idx);
     *
     * one WRITE with the address folded in, and it gets a NACK back from a
     * reserved address - which means the bus was driven. Measured side by side:
     *
     *     IDF : cmd[0]=0x80003000 cmd[1]=0x00000902 cmd[2]=0x00001000   (done=1)
     *     port: cmd[0]=0x00003000 cmd[1]=0x00000901 cmd[2]=0x00000901   (done=0)
     *
     * 0x902 is byte_num=2, 0x901 is byte_num=1. That is the whole difference. */
    if (msg->flags & RT_I2C_RD)
    {
        /* Address phase on its own, then a READ: the controller needs the
         * repeated start between them, and the address cannot be folded into a
         * READ the way it is folded into a WRITE. */
        cmd.val = 0;
        cmd.op_code = I2C_LL_CMD_WRITE;
        cmd.byte_num = 1;
        cmd.ack_en = 1;
        cmd.ack_exp = 0;
        cmd.ack_val = 0;
        i2c_ll_write_txfifo(hw, &addr, 1);
        i2c_ll_master_write_cmd_reg(hw, cmd, (int)idx++);

        cmd.val = 0;
        cmd.op_code = I2C_LL_CMD_READ;
        cmd.byte_num = (rt_uint8_t)msg->len;
        cmd.ack_val = 1;
        i2c_ll_master_write_cmd_reg(hw, cmd, (int)idx++);
    }
    else if (msg->len > 0u)
    {
        /* Address + every data byte in ONE command, exactly as IDF does it. */
        cmd.val = 0;
        cmd.op_code = I2C_LL_CMD_WRITE;
        cmd.byte_num = (rt_uint8_t)(msg->len + 1u);
        cmd.ack_en = 1;
        cmd.ack_exp = 0;
        cmd.ack_val = 0;
        /* FIFO FIRST, then the command - IDF's order (i2c_master.c:198-199).
         * Writing the command first tells the FSM to start on an empty FIFO. */
        i2c_ll_write_txfifo(hw, &addr, 1);
        i2c_ll_write_txfifo(hw, msg->buf, (uint8_t)msg->len);
        i2c_ll_master_write_cmd_reg(hw, cmd, (int)idx++);
    }
    else
    {
        /* Probe only: the address byte alone, still one WRITE. */
        cmd.val = 0;
        cmd.op_code = I2C_LL_CMD_WRITE;
        cmd.byte_num = 1;
        cmd.ack_en = 1;
        cmd.ack_exp = 0;
        cmd.ack_val = 0;
        i2c_ll_write_txfifo(hw, &addr, 1);
        i2c_ll_master_write_cmd_reg(hw, cmd, (int)idx++);
    }

    /* STOP, and NOT END.
     *
     * IDF's driver emits RESTART, WRITE, STOP - three commands, no END - because
     * this part has SOC_I2C_STOP_INDEPENDENT: the STOP carries its own end
     * semantics and a following END is a fifth command the FSM is not expecting.
     * Measured side by side against IDF's own driver on this board:
     *
     *     IDF : cmd[0]=0x80003000  cmd[1]=0x00000902  cmd[2]=0x00001000
     *     port: cmd[0]=0x00003000  cmd[1]=0x00000902  cmd[2]=0x00001000  cmd[3]=0x00002000
     *
     * The port had one command too many. */
    cmd.val = 0;
    cmd.op_code = I2C_LL_CMD_STOP;
    i2c_ll_master_write_cmd_reg(hw, cmd, (int)idx++);

    rt_kprintf("c5lpi2c: pre-start fifo_conf=0x%08x ctr=0x%08x cmd0=0x%08x\n",
               (unsigned)hw->fifo_conf.val, (unsigned)hw->ctr.val,
               (unsigned)hw->command[0].val);

    i2c_ll_update(hw);
    i2c_ll_start_trans(hw);

    /* Wait for the transaction, bounded. `end_detect` is the reliable completion
     * signal here; the HP driver learned the same lesson, and a zero-length write
     * is the case that has to be avoided (documented in i2c.c). */
    /* Poll int_raw, not int_status.
     *
     * int_status is the MASKED view and this driver enables no interrupts, so it
     * always reads zero - the exact bug the HP driver records at i2c.c:286, which
     * cost it a whole round. A polling driver has to read int_raw. */
    for (guard = 0; guard < 200000u; guard++)
    {
        if (hw->command[idx - 1].command_done)
        {
            break;
        }

        if (hw->int_raw.nack_int_raw || hw->int_raw.time_out_int_raw ||
            hw->int_raw.scl_st_to_int_raw || hw->int_raw.arbitration_lost_int_raw)
        {
            break;
        }
    }

    nack = hw->int_raw.nack_int_raw ? RT_TRUE : RT_FALSE;

    i2c_ll_clear_intr_mask(hw, 0xFFFFFFFFu);

    if (guard >= 200000u)
    {
        /* Reached the loop bound: the FSM never finished. */
        /* One-shot dump, the same shape the HP driver's timeout dump uses: which
         * command is stuck, and what the controller thinks its state is. A
         * timeout with trans_start=0 means the FSM never started (clock or
         * register update); trans_start=1 means it started and stalled, which
         * points at the pads. Those need different fixes and the registers are
         * the only thing that separates them. */
        static int dumped;
        if (!dumped)
        {
            rt_uint32_t k;

            dumped = 1;
            rt_kprintf("c5lpi2c: timeout dump ctr=0x%08x status=0x%08x "
                       "scl_low=%u scl_high=%u\n",
                       (unsigned)hw->ctr.val, (unsigned)hw->int_raw.val,
                       (unsigned)hw->scl_low_period.scl_low_period,
                       (unsigned)hw->scl_high_period.scl_high_period);
            for (k = 0; k < idx; k++)
            {
                rt_kprintf("c5lpi2c: cmd[%u]=0x%08x\n",
                           (unsigned)k, (unsigned)hw->command[k].val);
            }
        }

        return -RT_ETIMEOUT;
    }

    if (nack)
    {
        return -RT_EIO;
    }

    if (msg->flags & RT_I2C_RD)
    {
        uint32_t got = 0;

        if (nack)
        {
            return -RT_EIO;
        }

        i2c_ll_get_rxfifo_cnt(hw, &got);
        if (got < msg->len)
        {
            return -RT_EIO;
        }

        i2c_ll_read_rxfifo(hw, msg->buf, (uint8_t)msg->len);
    }
    else if (nack)
    {
        return -RT_EIO;
    }

    return RT_EOK;
}

static rt_ssize_t c5_lp_i2c_master_xfer(struct rt_i2c_bus_device *bus,
                                        struct rt_i2c_msg msgs[], rt_uint32_t num)
{
    rt_uint32_t i;

    RT_UNUSED(bus);

    for (i = 0; i < num; i++)
    {
        rt_err_t rc;

        if (msgs[i].addr > 0x7Fu || msgs[i].len > 255u)
        {
            return -RT_EINVAL;
        }

        rc = c5_lp_i2c_xfer_msg(&msgs[i]);
        if (rc != RT_EOK)
        {
            return (i == 0u) ? rc : (rt_ssize_t)i;
        }
    }

    return (rt_ssize_t)num;
}

static rt_err_t c5_lp_i2c_bus_control(struct rt_i2c_bus_device *bus, int cmd, void *args)
{
    RT_UNUSED(bus);

    switch (cmd)
    {
    case RT_I2C_DEV_CTRL_CLK:
    {
        rt_uint32_t freq = *(rt_uint32_t *)args;

        c5_lp_i2c_apply_freq(freq);
        return RT_EOK;
    }

    default:
        return -RT_ENOSYS;
    }
}

static const struct rt_i2c_bus_device_ops s_lp_i2c_ops __attribute__((used)) =
{
    .master_xfer     = c5_lp_i2c_master_xfer,
    .slave_xfer      = RT_NULL,
    .i2c_bus_control = c5_lp_i2c_bus_control,
};

rt_err_t rt_hw_lp_i2c_register(void)
{
    rt_err_t rc = rt_hw_lp_i2c_init();

    if (rc != RT_EOK)
    {
        return rc;
    }

    return rt_i2c_bus_device_register(&s_lp_i2c.parent, C5_LP_I2C_NAME);
}

/* ------------------------------------------------------------------ msh -- */

/* `c5lpi2c` - prove the LP controller is real without an LP slave to talk to.
 *
 * Three separate observations, in increasing strength:
 *
 *   1. both pads idle high through their pull-ups - the LP IO mux handed them
 *      over and the peripheral is not holding them down;
 *
 *   2. a full bus scan finds nothing, which is expected and proves nothing;
 *
 *   3. a transfer to an address that cannot exist produces a NACK. THAT is the
 *      observation with content: a NACK means the controller drove a start
 *      condition and clocked nine bits, and something on the wire pulled the
 *      ninth low. A dead controller cannot fake it.
 */
int c5_lp_i2c_selftest(void)
{
    rt_err_t rc;
    struct rt_i2c_msg msg;
    rt_uint8_t dummy = 0;
    rt_uint32_t sda_lvl;
    rt_uint32_t scl_lvl;

    rc = rt_hw_lp_i2c_register();
    if (rc != RT_EOK)
    {
        rt_kprintf("c5lpi2c: FAIL, init/register (%d)\n", (int)rc);
        return -1;
    }

    rt_kprintf("c5lpi2c: LP I2C controller, SDA=IO%d SCL=IO%d, %u Hz\n",
               (int)C5_LP_I2C_SDA_PIN, (int)C5_LP_I2C_SCL_PIN,
               (unsigned)s_lp_i2c.freq);

    /* 1. idle levels, read BOTH ways.
     *
     * `rtcio_ll_get_level()` reads the LP GPIO input register; `gpio_ll_get_level()`
     * reads the HP one. They disagree when the pad has been handed to the LP
     * domain, and which one is meaningful depends on where the mux points - so
     * printing only one of them is how "the pads are not released" gets reported
     * for a pad that is fine. */
    sda_lvl = rtcio_ll_get_level(C5_LP_I2C_SDA_PIN);
    scl_lvl = rtcio_ll_get_level(C5_LP_I2C_SCL_PIN);

    /* Is the LP_I2C register block reachable at all?
     *
     * `date` is a read-only constant burned into the peripheral. If it reads 0,
     * every write to this block has been going nowhere and the whole question is
     * the LP peripheral clock, not the I2C configuration. That is a different
     * diagnosis from "the FSM did not start" and it has to be separated before
     * anything else is changed. */
    rt_kprintf("c5lpi2c: LP_I2C.date=0x%08x ver=0x%08x (a non-zero date means the "
               "block is clocked and reachable)\n",
               (unsigned)LP_I2C.date.val, (unsigned)LP_I2C.date.date);

    rt_kprintf("c5lpi2c: idle levels via LP: SDA=%u SCL=%u | via HP: SDA=%u SCL=%u\n",
               (unsigned)sda_lvl, (unsigned)scl_lvl,
               (unsigned)gpio_ll_get_level(&GPIO, C5_LP_I2C_SDA_PIN),
               (unsigned)gpio_ll_get_level(&GPIO, C5_LP_I2C_SCL_PIN));

    /* Take the HP view as authoritative: this port has driven and read every
     * other pad through it, so it is the path with a known-good history. */
    sda_lvl = gpio_ll_get_level(&GPIO, C5_LP_I2C_SDA_PIN);
    scl_lvl = gpio_ll_get_level(&GPIO, C5_LP_I2C_SCL_PIN);

    if (sda_lvl != 1u || scl_lvl != 1u)
    {
        rt_kprintf("c5lpi2c: FAIL, the pads are not released - the LP IO mux did "
                   "not hand them to the peripheral, or something else is driving "
                   "them low\n");
        return -1;
    }

    /* 2. the transfer itself. Address 0x7F with the R/W bit set is 0xFF - a
     * reserved address that no compliant device answers. */
    msg.addr  = 0x7Fu;
    msg.flags = RT_I2C_WR;
    msg.len   = 1;
    msg.buf   = &dummy;

    rc = c5_lp_i2c_xfer_msg(&msg);

    rt_kprintf("c5lpi2c: transfer to the reserved address 0x7F returned %d\n",
               (int)rc);

    if (rc == -RT_ETIMEOUT)
    {
        /* What is ESTABLISHED, and what is not.
         *
         * Established: the register block is reachable (date reads 0x02410160, a
         * constant burned into the peripheral); the LP peripheral clock is on
         * (LPPERI.clk_en.lp_ext_i2c_ck_en == 1); the controller's own clock gate is
         * on (ctr.clk_en == 1); both pads idle high through their pull-ups; and
         * the five commands were written where the FSM should read them.
         *
         * Not established: why the FSM never latches trans_start. ctr reads
         * 0x00000100 with `command_done` clear on all five commands and
         * `int_raw` showing nothing at all - not a NACK, not a timeout, no event.
         *
         * What is ruled out by that: the register block, both clock gates, the
         * pads, and the command encoding. What is left is something the LL does
         * not expose - most likely an LP-domain gate that IDF's own driver opens
         * somewhere this port has not found, since `i2c_ll_enable_controller_clock`
         * is a no-op for LP_I2C and nothing else in i2c_ll.h touches LP_I2C's
         * clock enable.
         *
         * Stated rather than guessed at. The LP UART would face the same question,
         * which is why it is not attempted on top of this. */
        rt_kprintf("c5lpi2c: FAIL, the FSM never started. Ruled out: the register "
                   "block (date reads), LPPERI and ctr clocks (both read 1), the "
                   "pads (both idle high), the command encoding. `int_raw` is empty, "
                   "so it never even saw a start condition.\n");
        return -1;
    }

    if (rc == -RT_EIO)
    {
        rt_kprintf("c5lpi2c: PASS (the controller drove a start condition, clocked "
                   "nine bits and saw the address NACKed; there is no LP I2C slave "
                   "on this board, so no successful transfer is possible)\n");
        return 0;
    }

    /* RT_EOK on a reserved address would mean the bus is being answered by
     * something that should not be there, or that the completion flag is lying. */
    rt_kprintf("c5lpi2c: FAIL, a reserved address was acknowledged - either the "
               "bus has a device at 0x7F or the status flags are not what they say\n");
    return -1;
}

#endif /* SOC_LP_I2C_SUPPORTED */

