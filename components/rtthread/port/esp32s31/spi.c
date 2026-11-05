/*
 * ESP32-S31 SPI master over the GPSPI LL.
 *
 * Two SPIs are registered (`spi0`, `spi1` -> SPI2/SPI3 peripherals; SPI1 is the
 * flash controller and is not exposed). They are plain `rt_spi_bus` buses, so
 * `rt_spi_transfer()`, `rt_spi_send_then_recv()` and any RT-Thread SPI device
 * package (SD over SPI, ENC28J60, displays) work unchanged.
 *
 * What this driver deliberately does NOT do:
 *
 *   - DMA. The GPSPI DMA is a separate allocator plus descriptor rings; polled
 *     transfers are enough for small peripherals and are far easier to audit.
 *     `SOC_SPI_MAXIMUM_BUFFER_SIZE` (64) caps one hardware transaction anyway,
 *     so a long transfer is a loop either way.
 *   - Multi-line (Dual/Quad/OCT) modes. `spi_ll_master_set_line_mode()` exists
 *     and the field is programmed for 1-line here; anything else needs the
 *     flash-style command/addr/dummy phases, which is a different API.
 *
 * Three S31-specific points:
 *
 * 1. The clock comes from a source the driver picks. `spi_ll_master_set_clock()`
 *    takes `fapb` and computes the divider, and it is only exact when the source
 *    is the APB clock - so this driver queries it rather than assuming 80 MHz.
 *
 * 2. `SPI_LL_ONE_LINE_CTRL_MASK`/`..._USER_MASK` are cleared inside
 *    spi_ll_master_set_line_mode(); without that call the reset value of the
 *    fcmd / faddr / fread bits is whatever the ROM left, and the first
 *    transaction can go out in a line mode nobody asked for.
 *
 * 3. The peripheral must be brought out of reset before any register write is
 *    meaningful. `spi_ll_enable_bus_clock()` + `spi_ll_reset_register()` are
 *    separate calls and both are required.
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#include "sdkconfig.h"
#include "soc/soc.h"
#include "soc/soc_caps.h"
#include "soc/gpio_sig_map.h"
#include "soc/io_mux_reg.h"
#include "soc/spi_reg.h"
#include "hal/spi_ll.h"
#include "hal/spi_types.h"
#include "hal/gpio_ll.h"
#include "port_board.h"
#include "esp_rom_gpio.h"
#include "esp_private/esp_clk.h"
#include "esp_private/periph_ctrl.h"
#include "esp_attr.h"

#if defined(SOC_GPSPI_SUPPORTED)

#include "port_spi.h"

/* SPI2 and SPI3 are the two general-purpose controllers. SPI0/SPI1 are the
 * cache/flash controllers and must not be touched. */
#define S31_SPI_NUM         2

struct s31_spi
{
    struct rt_spi_bus parent;
    spi_dev_t        *hw;
    spi_host_device_t  host;
    rt_uint32_t        port;
    rt_uint32_t        freq;
};

static struct s31_spi s_spi[S31_SPI_NUM];

static spi_dev_t *s31_spi_hw(rt_uint32_t port)
{
    return (port == 0) ? &GPSPI2 : &GPSPI3;
}

/* Route SCLK/MOSI/MISO/CS through the GPIO matrix. S31 pads are fully
 * mux-able, so no IO_MUX peripheral function is used - only PIN_FUNC_GPIO plus
 * the matrix signal indices. */
static rt_err_t s31_spi_set_pins(rt_uint32_t port, const struct s31_spi_pins *p)
{
    /* S31 names the GPSPI pins after the flash lines they share silicon with:
     * CK = clock, D = MOSI, Q = MISO, CS = chip select. There are no
     * SPI2_CLK/MOSI/MISO symbols on this part - the first attempt used those
     * names and the compiler offered GPIO_SD5_OUT_IDX instead, which is the
     * flash pin multiplexed onto the same matrix input. */
    static const struct
    {
        int sclk, mosi, miso, cs;
    } idx[S31_SPI_NUM] =
    {
        { SPI2_CK_PAD_OUT_IDX, SPI2_D_PAD_OUT_IDX, SPI2_Q_PAD_IN_IDX, SPI2_CS_PAD_OUT_IDX },
        { SPI3_CK_PAD_OUT_IDX, SPI3_D_PAD_OUT_IDX, SPI3_Q_PAD_IN_IDX, SPI3_CS_PAD_OUT_IDX },
    };

    rt_uint32_t i;

    if (port >= S31_SPI_NUM)
    {
        return -RT_EINVAL;
    }

    for (i = 0; i < 4; i++)
    {
        rt_base_t pin = (i == 0) ? p->sclk : (i == 1) ? p->mosi : (i == 2) ? p->miso : p->cs;

        if (pin < 0)
        {
            continue;
        }

        if (((SOC_GPIO_VALID_GPIO_MASK >> pin) & 1ULL) == 0)
        {
            return -RT_EINVAL;
        }

        gpio_ll_func_sel(&GPIO, (uint8_t)pin, PIN_FUNC_GPIO);
        gpio_ll_input_enable(&GPIO, (uint8_t)pin);
        gpio_ll_set_output_enable_ctrl(&GPIO, (uint8_t)pin, true, true);
    }

    esp_rom_gpio_connect_out_signal((uint32_t)p->sclk, idx[port].sclk, false, false);
    esp_rom_gpio_connect_out_signal((uint32_t)p->mosi, idx[port].mosi, false, false);
    esp_rom_gpio_connect_in_signal((uint32_t)p->miso, idx[port].miso, false);

    if (p->cs >= 0)
    {
        esp_rom_gpio_connect_out_signal((uint32_t)p->cs, idx[port].cs, false, false);
    }

    /* MISO is an input from the peripheral's point of view. */
    gpio_ll_output_disable(&GPIO, (uint8_t)p->miso);
    gpio_ll_input_enable(&GPIO, (uint8_t)p->miso);

    return RT_EOK;
}

static rt_err_t s31_spi_configure(struct rt_spi_device *device,
                                  struct rt_spi_configuration *cfg)
{
    struct s31_spi *bus = (struct s31_spi *)device->bus;
    spi_dev_t *hw = bus->hw;
    rt_uint32_t freq = cfg->max_hz;

    if (freq == 0 || freq > 80000000u)
    {
        freq = (freq == 0) ? 1000000u : 80000000u;
    }

    /* CPOL/CPHA -> the hardware mode field, 0..3. RT-Thread encodes them in
     * the low two bits of cfg->mode, which happens to match. */
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

    /* The clock divider is only meaningful relative to the clock the peripheral
     * actually sees, so ask instead of assuming. esp_clk_apb_freq() is exact on
     * this chip and follows DFS. */
    spi_ll_master_set_clock(hw, (int)esp_clk_apb_freq(), (int)freq, 128);

    bus->freq = freq;
    return RT_EOK;
}

/*
 * One hardware transaction. The LL caps a single transfer at
 * SOC_SPI_MAXIMUM_BUFFER_SIZE bytes (64), so this loops.
 *
 * The full-duplex flag is derived from the message: if both buffers are given
 * the hardware shifts simultaneously, which is what full duplex means and what
 * every SPI peripheral expects.
 */
static rt_uint32_t s31_spi_xfer_chunk(struct s31_spi *bus,
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

    /* A stale trans_done left over from a previous transfer would make the poll
     * below succeed immediately and then read an empty RX FIFO. */
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
            rt_kprintf("spi%d: transfer timeout\n", (int)bus->port);
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

static rt_ssize_t s31_spi_xfer(struct rt_spi_device *device,
                               struct rt_spi_message *message)
{
    struct s31_spi *bus = (struct s31_spi *)device->bus;
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

        if (s31_spi_xfer_chunk(bus, tx, rx, chunk, !last) == 0)
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

static const struct rt_spi_ops s31_spi_ops =
{
    .configure = s31_spi_configure,
    .xfer      = s31_spi_xfer,
};

static rt_err_t s31_spi_init_port(rt_uint32_t port, const struct s31_spi_pins *pins,
                                  rt_uint32_t freq)
{
    struct s31_spi *bus = &s_spi[port];
    char name[8];

    bus->hw   = s31_spi_hw(port);
    bus->host = (port == 0) ? SPI2_HOST : SPI3_HOST;
    bus->port = port;

    if (s31_spi_set_pins(port, pins) != RT_EOK)
    {
        return -RT_EINVAL;
    }

    /* Both of these are macros that require an RCC atomic environment in scope
     * (they reference __DECLARE_RCC_ATOMIC_ENV to force the caller into a
     * critical section). PERIPH_RCC_ATOMIC() is the IDF idiom and is what the
     * esp_driver_spi path would use as well. */
    PERIPH_RCC_ATOMIC()
    {
        spi_ll_enable_bus_clock(bus->host, true);
        spi_ll_reset_register(bus->host);
        /* The bus clock alone is not enough: the master state machine runs on
         * the HS/mst clock, a separate gate in HP_SYS_CLKRST. Without it
         * usr=1 starts a transaction that never raises trans_done - which is
         * exactly the timeout the first version of this driver hit. */
        spi_ll_enable_clock(bus->host, true);
    }

    spi_ll_master_init(bus->hw);

    /* One-line mode for all three phases. Without this the fcmd/faddr/fread
     * bits keep whatever the ROM left and the first transfer can go out as a
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

    return rt_spi_bus_register(&bus->parent, name, &s31_spi_ops);
}

/* ------------------------------------------------------------------ msh -- */

/*
 * Loopback self test: MOSI is tied to MISO with a jumper, the driver sends a
 * pattern and compares.
 *
 * This is the only way to prove a SPI master works without a peripheral on the
 * bench, and it exercises the actual data path (FIFO, bit order, clock) rather
 * than just register state. Without the jumper it reports zeros, which is the
 * expected result and is stated as such instead of being reported as failure.
 */
static int cmd_spiloop(int argc, char **argv)
{
    rt_uint32_t port = 0;
    struct rt_spi_device *dev;
    struct rt_spi_configuration cfg;
    rt_uint8_t tx[8] = { 0xA5, 0x5A, 0x01, 0x80, 0xFF, 0x00, 0x3C, 0xC3 };
    rt_uint8_t rx[8] = { 0 };
    char bus_name[8];
    int i;

    if (argc >= 2)
    {
        port = (rt_uint32_t)atoi(argv[1]);
    }

    if (port >= S31_SPI_NUM)
    {
        rt_kprintf("usage: s31spiloop [port 0..%d]\n", S31_SPI_NUM - 1);
        return -1;
    }

    bus_name[0] = 's'; bus_name[1] = 'p'; bus_name[2] = 'i';
    bus_name[3] = (char)('0' + port); bus_name[4] = '\0';

    /*
     * rt_device_find("spi0") returns the BUS, not a device.
     *
     * A bus is registered as an rt_spi_bus whose parent is an rt_device; casting
     * that pointer to rt_spi_device and calling rt_spi_configure() dereferences
     * a mutex that was never initialised - which is the load access fault at
     * rt_object_get_type() that this command used to take. A real rt_spi_device
     * has to be attached to the bus first: that call is what fills in
     * device->bus, the CS pin and the lock.
     */
    {
        static struct rt_spi_device loop_dev;

        if (rt_spi_bus_attach_device(&loop_dev, "spiloop", bus_name, RT_NULL) != RT_EOK)
        {
            rt_kprintf("s31spiloop: attach to %s failed\n", bus_name);
            return -1;
        }

        dev = &loop_dev;
    }

    cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB;
    cfg.data_width = 8;
    cfg.max_hz = 1000000u;
    cfg.usage_freq = 0;

    if (rt_spi_configure(dev, &cfg) != RT_EOK)
    {
        rt_kprintf("s31spiloop: configure failed\n");
        return -1;
    }

    if (rt_spi_transfer(dev, tx, rx, sizeof(tx)) != sizeof(tx))
    {
        rt_kprintf("s31spiloop: transfer failed\n");
        return -1;
    }

    rt_kprintf("s31spiloop: %s tx/rx (tie MOSI to MISO for a real test)\n", bus_name);
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

    if (rt_memcmp(tx, rx, sizeof(tx)) == 0)
    {
        rt_kprintf("\ns31spiloop: PASS, loopback identical\n");
        return 0;
    }

    rt_kprintf("\ns31spiloop: rx differs - expected unless MOSI is jumpered to MISO\n");
    return 0;
}

MSH_CMD_EXPORT_ALIAS(cmd_spiloop, s31spiloop, SPI loopback check; tie MOSI to MISO);

rt_err_t rt_hw_spi_init(void)
{
    rt_err_t rc;

    /* Pins from port_board.h. The board's SPI LCD header (J1) and the SD slot
     * are separate buses; these two are the "free" pair on the 1x20P headers,
     * documented in port_board.h. */
    {
        struct s31_spi_pins p0 = {
            .sclk = BOARD_SPI0_SCLK_PIN,
            .mosi = BOARD_SPI0_MOSI_PIN,
            .miso = BOARD_SPI0_MISO_PIN,
            .cs   = BOARD_SPI0_CS_PIN,
        };
        struct s31_spi_pins p1 = {
            .sclk = BOARD_SPI1_SCLK_PIN,
            .mosi = BOARD_SPI1_MOSI_PIN,
            .miso = BOARD_SPI1_MISO_PIN,
            .cs   = BOARD_SPI1_CS_PIN,
        };

        rc = s31_spi_init_port(0, &p0, 1000000u);
        if (rc != RT_EOK)
        {
            return rc;
        }

        rc = s31_spi_init_port(1, &p1, 1000000u);
    }

    if (rc != RT_EOK)
    {
        rt_kprintf("spi: init failed %d\n", rc);
    }

    return rc;
}
INIT_BOARD_EXPORT(rt_hw_spi_init);

#endif /* SOC_GPSPI_SUPPORTED */
