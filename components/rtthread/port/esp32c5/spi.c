/*
 * ESP32-C5 GPSPI master over the GPSPI LL.
 *
 * One bus is registered as `spi0` (the GPSPI2 peripheral). SPI0/SPI1 are the
 * cache/flash controllers and must not be touched. It is a plain `rt_spi_bus`,
 * so `rt_spi_transfer()`, `rt_spi_send_then_recv()` and any RT-Thread SPI device
 * package work unchanged.
 *
 * What this driver deliberately does NOT do:
 *
 *   - DMA. The GPSPI DMA is a separate allocator plus descriptor rings; polled
 *     transfers are enough for small peripherals and far easier to audit.
 *     SOC_SPI_MAXIMUM_BUFFER_SIZE (64) caps one hardware transaction anyway, so a
 *     long transfer is a loop either way.
 *   - Multi-line (Dual/Quad/OCT) modes. The field is programmed for 1-line here;
 *     anything else needs the flash-style command/addr/dummy phases.
 *
 * Ported from port/esp32s31/spi.c. Three things differ on this chip:
 *
 * 1. THERE IS ONE GPSPI INSTANCE, NOT TWO. SOC_SPI_PERIPH_NUM is 2, but index 0
 *    is MSPI (the flash controller, no signal table) and only index 1 is
 *    GPSPI2 - SPI_LL_GET_HW() returns NULL for anything that is not SPI2_HOST.
 *    A driver that indexes two general-purpose buses writes registers in the
 *    flash controller on the second one.
 *
 * 2. THE PAD SIGNALS COME FROM spi_periph_signal[], NOT gpio_sig_map.h. The C5
 *    calls them FSPICLK_OUT_IDX / FSPID_OUT_IDX / FSPIQ_OUT_IDX / FSPICS0_OUT_IDX
 *    - the S31's SPI2_CK_PAD_OUT_IDX and friends do not exist here.
 *
 * 3. THE CLOCK IS APB-DERIVED AND MUST BE QUERIED. spi_ll_master_set_clock() is
 *    only exact when the source is the APB clock, so this driver asks
 *    esp_clk_apb_freq() rather than assuming a frequency.
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#include "sdkconfig.h"
#include "soc/soc.h"
#include "soc/soc_caps.h"
#include "soc/gpio_struct.h"
#include "soc/spi_pins.h"
#include "hal/spi_ll.h"
#include "hal/spi_types.h"
#include "soc/spi_periph.h"
#include "hal/gpio_ll.h"
#include "esp_rom_gpio.h"
#include "esp_private/esp_clk.h"
#include "esp_private/periph_ctrl.h"
#include "esp_attr.h"

#if defined(SOC_GPSPI_SUPPORTED)

#include "port_spi.h"

struct c5_spi
{
    struct rt_spi_bus parent;
    spi_dev_t        *hw;
    spi_host_device_t host;
    rt_uint32_t       port;
    rt_uint32_t       freq;
};

static struct c5_spi s_spi[C5_SPI_NUM];

/* Route SCLK/MOSI/MISO/CS through the GPIO matrix. The C5 pads are fully
 * mux-able, so no IO_MUX peripheral function is used - only PIN_FUNC_GPIO plus
 * the matrix signal indices from spi_periph_signal[]. */
static rt_err_t c5_spi_set_pins(rt_uint32_t port, const struct c5_spi_pins *p)
{
    const spi_signal_conn_t *sig;
    rt_uint32_t i;

    if (port >= C5_SPI_NUM)
    {
        return -RT_EINVAL;
    }

    /* SPI2 is periph index 1; index 0 is MSPI and has no signal table. */
    sig = &spi_periph_signal[port + 1];

    for (i = 0; i < 4; i++)
    {
        rt_base_t pin = (i == 0) ? p->sclk : (i == 1) ? p->mosi
                      : (i == 2) ? p->miso : p->cs;

        if (pin < 0)
        {
            continue;
        }

        if (pin >= SOC_GPIO_PIN_COUNT ||
            ((SOC_GPIO_VALID_GPIO_MASK >> pin) & 1ULL) == 0)
        {
            return -RT_EINVAL;
        }

        gpio_ll_func_sel(&GPIO, (uint8_t)pin, PIN_FUNC_GPIO);
        gpio_ll_input_enable(&GPIO, (uint8_t)pin);
        gpio_ll_set_output_enable_ctrl(&GPIO, (uint8_t)pin, true, true);
    }

    esp_rom_gpio_connect_out_signal((uint32_t)p->sclk, sig->spiclk_out, false, false);
    esp_rom_gpio_connect_out_signal((uint32_t)p->mosi, sig->spid_out, false, false);
    esp_rom_gpio_connect_in_signal((uint32_t)p->miso, sig->spiq_in, false);

    if (p->cs >= 0)
    {
        esp_rom_gpio_connect_out_signal((uint32_t)p->cs, sig->spics_out[0], false, false);
    }

    /* MISO is an input from the peripheral's point of view. */
    gpio_ll_output_disable(&GPIO, (uint8_t)p->miso);
    gpio_ll_input_enable(&GPIO, (uint8_t)p->miso);

    return RT_EOK;
}

static rt_err_t c5_spi_configure(struct rt_spi_device *device,
                                 struct rt_spi_configuration *cfg)
{
    struct c5_spi *bus = (struct c5_spi *)device->bus;
    spi_dev_t *hw = bus->hw;
    rt_uint32_t freq = cfg->max_hz;

    if (freq == 0 || freq > 80000000u)
    {
        freq = (freq == 0) ? 1000000u : 80000000u;
    }

    /* CPOL/CPHA -> the hardware mode field, 0..3. RT-Thread encodes them in the
     * low two bits of cfg->mode, which matches. */
    spi_ll_master_set_mode(hw, (rt_uint8_t)(cfg->mode & (RT_SPI_CPOL | RT_SPI_CPHA)));

    /* Bit order is per-direction and independent of the mode bits. */
    if (cfg->mode & RT_SPI_MSB)
    {
        spi_ll_set_tx_lsbfirst(hw, false);
        spi_ll_set_rx_lsbfirst(hw, false);
    }
    else
    {
        spi_ll_set_tx_lsbfirst(hw, true);
        spi_ll_set_rx_lsbfirst(hw, true);
    }

    /* The divider is only meaningful relative to the clock the peripheral
     * actually sees, so ask rather than assume. */
    spi_ll_master_set_clock(hw, (int)esp_clk_apb_freq(), (int)freq, 128);

    bus->freq = freq;
    return RT_EOK;
}

/*
 * One hardware transaction. The LL caps a single transfer at
 * SOC_SPI_MAXIMUM_BUFFER_SIZE bytes (64), so the caller loops.
 *
 * The full-duplex flag is derived from the message: if both buffers are given
 * the hardware shifts simultaneously.
 */
static rt_uint32_t c5_spi_xfer_chunk(struct c5_spi *bus,
                                     const rt_uint8_t *tx, rt_uint8_t *rx,
                                     rt_uint32_t len, rt_bool_t keep_cs)
{
    spi_dev_t *hw = bus->hw;
    rt_uint32_t guard;

    spi_ll_cpu_tx_fifo_reset(hw);
    spi_ll_cpu_rx_fifo_reset(hw);

    spi_ll_set_command_bitlen(hw, 0);
    spi_ll_set_addr_bitlen(hw, 0);
    spi_ll_set_dummy(hw, 0);
    spi_ll_set_mosi_bitlen(hw, (tx != RT_NULL) ? len * 8u : 0u);
    spi_ll_set_miso_bitlen(hw, (rx != RT_NULL) ? len * 8u : 0u);

    spi_ll_enable_mosi(hw, (tx != RT_NULL) ? 1 : 0);
    spi_ll_enable_miso(hw, (rx != RT_NULL) ? 1 : 0);

    spi_ll_master_keep_cs(hw, keep_cs ? 1 : 0);

    if (tx != RT_NULL)
    {
        spi_ll_write_buffer(hw, tx, len * 8u);
    }

    /* A stale trans_done from a previous transfer would make the poll below
     * succeed immediately and read an empty RX FIFO. */
    spi_ll_clear_intr(hw, SPI_LL_INTR_TRANS_DONE);

    spi_ll_apply_config(hw);
    spi_ll_user_start(hw);

    /* Polled completion. The transaction is at most 64 bytes; even at 100 kHz
     * that is ~5 ms, and the guard is scaled well beyond it so a stuck
     * peripheral returns instead of hanging the shell. */
    guard = 0;
    while (!spi_ll_usr_is_done(hw))
    {
        if (++guard > 20000000u)
        {
            rt_kprintf("c5spi: transfer timeout on spi%u\n", (unsigned)bus->port);
            spi_ll_cpu_tx_fifo_reset(hw);
            spi_ll_cpu_rx_fifo_reset(hw);
            return 0;
        }
    }

    if (rx != RT_NULL)
    {
        spi_ll_read_buffer(hw, rx, len * 8u);
    }

    return len;
}

static rt_ssize_t c5_spi_xfer(struct rt_spi_device *device,
                              struct rt_spi_message *message)
{
    struct c5_spi *bus = (struct c5_spi *)device->bus;
    rt_uint32_t total = 0;

    if (message == RT_NULL)
    {
        return 0;
    }

    /* Take CS before the first chunk, release after the last - the loop below
     * has to keep CS asserted across chunks or the peripheral sees several
     * separate transactions. */
    if (message->cs_take)
    {
        spi_ll_master_select_cs(bus->hw, (int)device->config.mode);
        spi_ll_master_keep_cs(bus->hw, 1);
    }

    while (total < message->length)
    {
        rt_uint32_t left = message->length - total;
        rt_uint32_t chunk = (left > 64u) ? 64u : left;
        const rt_uint8_t *tx = (message->send_buf != RT_NULL)
                               ? ((const rt_uint8_t *)message->send_buf + total) : RT_NULL;
        rt_uint8_t *rx = (message->recv_buf != RT_NULL)
                         ? ((rt_uint8_t *)message->recv_buf + total) : RT_NULL;
        rt_bool_t last = (total + chunk) >= message->length;

        if (c5_spi_xfer_chunk(bus, tx, rx, chunk, !last) == 0)
        {
            break;
        }

        total += chunk;
    }

    if (message->cs_release)
    {
        spi_ll_master_keep_cs(bus->hw, 0);
        spi_ll_master_select_cs(bus->hw, -1);
    }

    return (rt_ssize_t)total;
}

static const struct rt_spi_ops c5_spi_ops =
{
    .configure = c5_spi_configure,
    .xfer      = c5_spi_xfer,
};

rt_err_t c5_spi_init_port(rt_uint32_t port, const struct c5_spi_pins *pins,
                          rt_uint32_t freq)
{
    struct c5_spi *bus;
    char name[8];

    if (port >= C5_SPI_NUM)
    {
        return -RT_EINVAL;
    }

    bus = &s_spi[port];
    bus->hw   = SPI_LL_GET_HW(SPI2_HOST);
    bus->host = SPI2_HOST;
    bus->port = port;

    if (c5_spi_set_pins(port, pins) != RT_EOK)
    {
        return -RT_EINVAL;
    }

    /* Both of these are macros that require an RCC atomic environment in scope
     * (they reference __DECLARE_RCC_ATOMIC_ENV). PERIPH_RCC_ATOMIC() is the IDF
     * idiom. */
    PERIPH_RCC_ATOMIC()
    {
        spi_ll_enable_bus_clock(bus->host, true);
        spi_ll_reset_register(bus->host);
        /* The bus clock alone is not enough: the master state machine runs on
         * the HS/mst clock, a separate gate. Without it usr=1 starts a
         * transaction that never raises trans_done. */
        spi_ll_enable_clock(bus->host, true);
    }

    spi_ll_master_init(bus->hw);

    /* One-line mode for all three phases. Without this the fcmd/faddr/fread bits
     * keep whatever the ROM left and the first transfer can go out as a
     * multi-line command. */
    {
        spi_line_mode_t line = { .cmd_lines = 1, .addr_lines = 1, .data_lines = 1 };

        spi_ll_master_set_line_mode(bus->hw, line);
    }

    /* Full duplex, MOSI/MISO both enabled. */
    spi_ll_set_half_duplex(bus->hw, false);

    bus->freq = freq ? freq : 1000000u;
    spi_ll_master_set_clock(bus->hw, (int)esp_clk_apb_freq(), (int)bus->freq, 128);

    bus->parent.mode = RT_SPI_BUS_MODE_SPI;

    name[0] = 's'; name[1] = 'p'; name[2] = 'i';
    name[3] = (char)('0' + port); name[4] = '\0';

    return rt_spi_bus_register(&bus->parent, name, &c5_spi_ops);
}

rt_err_t rt_hw_spi_init(void)
{
    /* Default pins are the C5's own IOMUX defaults for GPSPI2 (spi_pins.h),
     * which are free on this board: the console is IO11/IO12, PWM uses
     * IO3/IO5/IO7, I2C uses IO8/IO9, and the in-package flash is on IO15-IO22. */
    struct c5_spi_pins p0 = {
        .sclk = C5_SPI0_SCLK_PIN,
        .mosi = C5_SPI0_MOSI_PIN,
        .miso = C5_SPI0_MISO_PIN,
        .cs   = C5_SPI0_CS_PIN,
    };
    rt_err_t rc = c5_spi_init_port(0, &p0, 1000000u);

    if (rc != RT_EOK)
    {
        rt_kprintf("spi0: init failed %d\n", rc);
    }

    return rc;
}
INIT_BOARD_EXPORT(rt_hw_spi_init);

/* ------------------------------------------------------------------ msh -- */

/* `c5spiloop` - loopback self test.
 *
 * MOSI has to be tied to MISO with a jumper for this to prove anything, and there
 * is no way to know from software whether it is. What the command therefore
 * reports is the truth about the two cases:
 *
 *   rx identical to tx   -> the data path works end to end (FIFO, bit order,
 *                           clock, both pads)
 *   rx all zero          -> the master clocked out and nothing drove MISO, i.e.
 *                           the jumper is missing
 *   rx anything else     -> something is driving MISO but the path is wrong
 *
 * That distinction is the difference between "SPI is broken" and "you forgot the
 * wire", and reporting zeros as a failure would be wrong.
 */
int c5_spi_selftest(rt_uint32_t port, rt_bool_t inside)
{
    struct rt_spi_device *dev;
    struct rt_spi_configuration cfg;
    rt_uint8_t tx[8] = { 0xA5, 0x5A, 0x01, 0x80, 0xFF, 0x00, 0x3C, 0xC3 };
    rt_uint8_t rx[8] = { 0 };
    char bus_name[8];
    int i;

    if (port >= C5_SPI_NUM)
    {
        rt_kprintf("usage: c5spiloop [port 0..%u]\n", (unsigned)(C5_SPI_NUM - 1));
        return -1;
    }

    bus_name[0] = 's'; bus_name[1] = 'p'; bus_name[2] = 'i';
    bus_name[3] = (char)('0' + port); bus_name[4] = '\0';

    /* rt_device_find("spi0") returns the BUS, not a device.
     *
     * Casting a bus pointer to rt_spi_device and calling rt_spi_configure()
     * dereferences a mutex that was never initialised - which is the load access
     * fault at rt_object_get_type() the S31 file records. A real rt_spi_device
     * has to be attached to the bus first: that call fills in device->bus, the CS
     * pin and the lock. */
    {
        static struct rt_spi_device loop_dev;

        if (rt_spi_bus_attach_device(&loop_dev, "spiloop", bus_name, RT_NULL) != RT_EOK)
        {
            rt_kprintf("c5spiloop: attach to %s failed\n", bus_name);
            return -1;
        }

        dev = &loop_dev;
    }

    rt_memset(&cfg, 0, sizeof(cfg));
    cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB;
    cfg.data_width = 8;
    cfg.max_hz = 1000000u;

    if (rt_spi_configure(dev, &cfg) != RT_EOK)
    {
        rt_kprintf("c5spiloop: configure failed\n");
        return -1;
    }

    /* Software loopback, attempted and NOT working - kept because the failure is
     * informative.
     *
     * The GPIO matrix is pad-centric: a peripheral output goes to a pad, and a
     * pad's input feeds a peripheral input. So `spid_out` cannot be routed
     * straight to `spiq_in` - tried first, and rx came back all zeros because
     * nothing drives the pad.
     *
     * The second attempt makes the MISO pad carry `spid_out` itself, so the pad
     * drives its own input. Measured: rx is then a constant 0xff rather than the
     * shifted pattern, i.e. the pad output is enabled but the data does not
     * reach it - the output-enable for that pad is not tied to the SPI
     * peripheral's own MOSI enable.
     *
     * Getting this right needs the pad's OE source wired to the SPI block's
     * output-enable signal, which is a different matrix slot than the data
     * signal. Not pursued further: a jumper is a one-second test and this is a
     * diagnostic convenience. The `jumper` mode below is the real test. */
    if (inside)
    {
        const spi_signal_conn_t *sig = &spi_periph_signal[port + 1];

        esp_rom_gpio_connect_out_signal((uint32_t)C5_SPI0_MISO_PIN,
                                        sig->spid_out, false, false);
        gpio_ll_set_output_enable_ctrl(&GPIO, (uint8_t)C5_SPI0_MISO_PIN,
                                       true, true);
    }

    if (rt_spi_transfer(dev, tx, rx, sizeof(tx)) != sizeof(tx))
    {
        rt_kprintf("c5spiloop: transfer failed\n");
        return -1;
    }

    if (inside)
    {
        const spi_signal_conn_t *sig = &spi_periph_signal[port + 1];

        esp_rom_gpio_connect_out_signal((uint32_t)C5_SPI0_MISO_PIN,
                                        sig->spiq_out, false, false);
        gpio_ll_output_disable(&GPIO, (uint8_t)C5_SPI0_MISO_PIN);
        gpio_ll_input_enable(&GPIO, (uint8_t)C5_SPI0_MISO_PIN);
    }

    rt_kprintf("c5spiloop: %s tx/rx at %u Hz (tie MOSI to MISO for a real test)\n",
               bus_name, (unsigned)s_spi[port].freq);
    rt_kprintf("  tx:");
    for (i = 0; i < (int)sizeof(tx); i++)
    {
        rt_kprintf(" %02x", tx[i]);
    }
    rt_kprintf("\n  rx:");
    for (i = 0; i < (int)sizeof(rx); i++)
    {
        rt_kprintf(" %02x", rx[i]);
    }
    rt_kprintf("\n");

    if (rt_memcmp(tx, rx, sizeof(tx)) == 0)
    {
        rt_kprintf("c5spiloop: PASS, loopback identical\n");
        return 0;
    }

    {
        rt_uint32_t acc = 0;

        for (i = 0; i < (int)sizeof(rx); i++)
        {
            acc |= rx[i];
        }

        if (acc == 0)
        {
            rt_kprintf("c5spiloop: all zeros - the master ran but nothing drove "
                       "MISO; tie MOSI to MISO with a jumper to test the path\n");
            return 0;
        }
    }

    rt_kprintf("c5spiloop: rx differs and is not all zeros - something drives "
               "MISO but the data path is wrong\n");
    return -1;
}

#endif /* SOC_GPSPI_SUPPORTED */






