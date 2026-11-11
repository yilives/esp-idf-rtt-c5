# ESP32-C5 + RT-Thread - full buildable fork

This is a fork of [RT-Thread-packages/esp-idf](https://github.com/RT-Thread-packages/esp-idf)
whose default branch is the **ESP32-C5 port**. Cloning it gives you a tree that
builds as-is: ESP-IDF v6.1 with RT-Thread 5.4.0 as the kernel, plus the C5 port.

If you only want the port and the project - without 350 MB of ESP-IDF history -
use [yilives/esp32c5-rtthread](https://github.com/yilives/esp32c5-rtthread)
instead.

**The port itself lives in `components/rtthread/port/esp32c5/`; the project that
exercises it is `c5-rtt-full/`.**

---

# ESP32-C5 + RT-Thread

A port of **RT-Thread 5.4.0** to the **ESP32-C5**, running inside ESP-IDF in place
of the FreeRTOS kernel.

The chip is a single 240 MHz RISC-V core (plus an LP core), 16 MB in-package
flash, 8 MB in-package PSRAM, dual-band Wi-Fi 6 and BLE 5. It has **no FPU**.
Everything below was measured on real hardware, and the report in
[docs/PORTING-REPORT.md](docs/PORTING-REPORT.md) records what failed as well as
what worked.

```
RT-Thread 5.4.0 on ESP32-C5 (Wi-Fi + BLE)
tick=1000 Hz, cpus=1, msh on UART0
msh >
```

---

## What works

Every row is a self test you can run from the shell, and each reports a
measurement rather than "OK".

| | command | what it actually proves |
|---|---|---|
| Kernel | `list thread` | RT-Thread scheduler, 1 ms tick, one core |
| Console | — | `uart0` at 115200, msh |
| Wi-Fi | `c5join5`, `c5iperf`, `c5rx` | 5 GHz, WPA2, 23.7 Mbit/s TCP measured on the wire |
| BLE | `c5all` | advertising as `c5-rtt`, seen by another device at -43 dBm |
| SoftAP | `c5ap` | beaconing, visible, AP+STA round trip |
| GPIO / PIN | `c5pins`, `c5probe` | every pad driven and read back; ownership conflicts reported |
| PWM | `c5pwm` | frequency and duty measured at the pad, 1 Hz – 40 kHz |
| I2C | `i2cscan` | NACK and timeout counters, not a device count |
| SPI | `c5spiloop` | byte-identical loopback through the real data path |
| ADC | `c5adc` | all 6 channels converted; **millivolts from eFuse curve fitting** |
| CAN (TWAI) | `c5can`, `c5twai` | loopback at 125k/250k/500k/1M **and** as an `rt_can` device |
| RMT | `c5rmt` | TX waveform measured back by RX: 19 symbols, mean half 4761 ticks |
| MCPWM | `c5mcpwm` | complementary pair + dead time, measured at the pad |
| PCNT | `c5pcnt` | pulse counting as an `rt_pulse_encoder` device |
| Temperature | `c5tsens` | two-band cross-check, uncalibrated (no eFuse deltaT) |
| AHB GDMA | `c5gdma` | mem-to-mem byte-exact at 1/16/128/256 B, no overrun |
| PARLIO | `c5parlio` | GDMA feeds it, unit consumes every bit, PCNT sees the clock |
| PSRAM | `c5psram test 7000` | 7 MB written and read back byte-exact |
| `rt_malloc()` | `c5heap` | small blocks stay in SRAM, 4 MB lands in PSRAM, contents verified |
| DMA → PSRAM | `c5dma` | byte-exact both directions; **fails without cache flush/invalidate** |
| Kernel soak | `c5soak 2000 1` | 2000 rounds: both heaps flat, tick vs hardware clock within 1 ms |
| LP core | `c5ulp` | LP program runs, shares variables, checksum verified |
| Deep sleep | `c5sleep 3` | timer wake works; **PSRAM does not survive it** |
| USB Serial/JTAG | `c5usj` | device side only; no host, so no enumeration |

`list device`:

```
pcnt0        Miscellaneous Device 0
can0         CAN Device           0
spi0         SPI Bus              0
adc1         ADC Device           0
i2c0         I2C Bus              0
pwm0         PWM Device           0
pin          Pin Device           0
uart0        Character Device     1
```

---

## What does not work

Stated here rather than buried, because a port that only lists successes is not
useful to the next person.

| | status |
|---|---|
| ~~**LP I2C**~~ | **works since §17ab.** The FSM never started because a correct `rtc_gpio_*` pad setup was undone by four leftover raw `rtcio_ll_*` calls below it. |
| **LP UART / LP peripherals** | not attempted — blocked on the same unknown as LP I2C |
| **SDIO slave** | needs a host on the other end |
| ~~`rt_malloc()` → PSRAM~~ | **done**: a `psram` memheap with auto-binding; 4 MB allocated, written and read back |
| **PARLIO RX** | TX only |
| **Analog layer** | no transceiver for CAN, no scope for MCPWM dead time, no host for USB |
| **ADC accuracy** | millivolts are computed, but accuracy against a reference is not measured |
| **Streaming into PSRAM** | a transfer the CPU starts works (`c5dma`); a peripheral streaming in while the CPU reads behind it is not exercised |

---

## Getting started

### 1. Prerequisites

* ESP-IDF **v6.1** with the RISC-V toolchain (`riscv32-esp-elf`)
* **RT-Thread 5.4.0** source tree
* Python 3.11+ with `pyserial` (for the test harness)

### 2. Lay the port over an IDF + RT-Thread tree

This repository contains the **parts you add**, not a fork of ESP-IDF. You need
a tree where IDF already knows how to build RT-Thread as its kernel — that
arrangement lives upstream in
[RT-Thread-packages/esp-idf](https://github.com/RT-Thread-packages/esp-idf)
(the `rtt-dev` branch). If you would rather not assemble it by hand, use the
full fork instead:
**[github.com/yilives/esp-idf-rtt-c5](https://github.com/yilives/esp-idf-rtt-c5)**
— that one is ready to build as-is.

Assuming `$IDF` is your `esp-idf` checkout and `$RTT` your RT-Thread tree:

```bash
cp -r port/esp32c5        $IDF/components/rtthread/port/
cp -r project/main        <your-project>/main/
cp    project/CMakeLists.txt project/partitions.csv \
      project/sdkconfig.defaults      <your-project>/
```

Then apply the C5 wiring to `$IDF/components/rtthread/CMakeLists.txt`. It is five
edits, and `docs/cmake-c5-sections.md` lists them with the surrounding context:

1. add the 21 port `.c` files to the C5 `target_srcs`
2. add `dev_can.c` and `ringbuffer.c` to the C5 `serial_srcs`
3. add the `-u` anchors (see below)
4. link `idf::esp_hal_*` components
5. suppress two warnings for `dev_can.c` only

> **The `-u` flags are not optional.** `INIT_BOARD_EXPORT` and friends put a
> pointer in a data section, and `--gc-sections` drops archive members nothing
> references *by symbol*. Measured eleven times in this port: the object compiles,
> sits in the build tree, and is simply absent from the linked image — no error
> anywhere, and `list device` just shows fewer devices.

### 3. Configure

`sdkconfig.defaults` is the known-good configuration. The parts that matter:

```
CONFIG_IDF_RTOS_RTTHREAD=y
CONFIG_IDF_RTOS_RTTHREAD_FREERTOS_SHIM=y
CONFIG_FREERTOS_UNICORE=y
CONFIG_FREERTOS_HZ=1000

CONFIG_SPIRAM=y                     # 8 MB in-package
CONFIG_SPIRAM_MODE_QUAD=y
CONFIG_SPIRAM_SPEED_40M=y
CONFIG_SPIRAM_BOOT_INIT=y
CONFIG_SPIRAM_USE_MALLOC=y
# CONFIG_SPIRAM_IGNORE_NOTFOUND is not set   <- keep it this way, see below

CONFIG_ULP_COPROC_ENABLED=y         # LP core
CONFIG_ULP_COPROC_TYPE_LP_CORE=y
CONFIG_ULP_COPROC_RESERVE_MEM=4096

CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y
CONFIG_PARTITION_TABLE_CUSTOM=y
CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="partitions.csv"
```

> **`CONFIG_SPIRAM_IGNORE_NOTFOUND` must stay off.** Turning it on lets the image
> boot with no PSRAM at all, which hides the exact failure you are debugging.

> **A PSRAM change needs the bootloader rebuilt.** `CONFIG_SPIRAM` does not
> invalidate the bootloader stamp, so remove **both** `build/bootloader` and
> `build/bootloader-prefix` — removing only the first gives `could not load
> cache`.

### 4. Build and flash

```bash
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

On Windows with the helper in this repo:

```powershell
.\tools\build.ps1 -Project <your-project> build
```

`build.ps1` runs `tools/check_xtensa_guards.py` first — see below.

### 5. Credentials

`project/main/wifi_creds.h` is **not** in this repository. Create it:

```c
#define WIFI_TEST_SSID  "your-ap"
#define WIFI_TEST_PASS  "your-password"
#define WIFI_B24_SSID   "your-ap_2.4G"
#define WIFI_B24_PASS   "your-password"
#define WIFI_B5_SSID    "your-ap_5G"
#define WIFI_B5_PASS    "your-password"
```

If your SSID is not pure ASCII, write it as hex byte escapes with the text in a
comment — the console cannot print UTF-8 well enough to tell `_5G` from a missing
suffix, and that mistake cost a round here (`reason=201`, NO_AP_FOUND).

---

## Running the tests

```bash
# one command, resets the board first
python tools/c5cmd.py /dev/ttyUSB0 "c5pins"

# several stateful commands in one boot (needed for anything that requires the
# Wi-Fi association, because a reset wipes it)
python tools/c5seq.py /dev/ttyUSB0 "c5join5" "c5iperf 192.168.1.10 4444 5"
```

Throughput needs a listener on the host:

```bash
python tools/tcpsink.py 4444          # terminal 1
python tools/c5seq.py COM21 "c5join5" "c5iperf <host-ip> 4444 5"
```

---

## The pin map

All of it is enforced by `port/esp32c5/resources.c`; run `c5pins` to print it and
to fail on a conflict.

| pad | use | note |
|---|---|---|
| IO0 | RMT | TX and RX on one pad |
| IO1 | MCPWM A | |
| IO2 | **LP I2C SDA** | fixed by hardware; used to be SPI MISO |
| IO3 | **LP I2C SCL** | fixed by hardware; used to be PWM |
| IO4 | CAN TX/RX | one pad, single-pad loopback |
| IO5 | PCNT | |
| IO6 / IO7 / IO10 | SPI CLK / MOSI / CS | |
| IO8 / IO9 | I2C0 SCL / SDA | |
| IO11 / IO12 | console TX / RX | UART0 default |
| IO13 / IO14 | USB D- / D+ | do not reuse |
| IO15 – IO22 | in-package flash | do not touch |
| IO23 | SPI MISO | moved here for LP I2C |
| IO24 | PWM | moved here for LP I2C |

**`rtc_io_periph.c` maps only GPIO0..GPIO6 to the LP/RTC domain.** IO7 and above
are `-1`. LP I2C is fixed to IO2/IO3 by `i2c_ll.h`, which is why SPI and PWM had
to move — they are re-routable, LP I2C is not.

---

## Two heaps, and why not one

`rt_malloc()` reaches the 8 MB through a second memheap named `psram`, with
`RT_USING_MEMHEAP_AUTO_BINDING` so a request the 65 KB system heap cannot satisfy
falls through to it.

```
msh > free
memheap       pool size  max used size available size
psram        8126464    64            8126400
heap         65536      33504         32032
```

**This requires `RT_USING_MEMHEAP_AS_HEAP`, not `RT_USING_SMALL_MEM_AS_HEAP`.**
`kservice.c` selects the allocator with `#if SMALL_MEM_AS_HEAP / #elif
MEMHEAP_AS_HEAP`, and the walk over "every other memheap" lives inside
`_memheap_alloc()`, which only exists in the MEMHEAP branch. With small-mem as
the heap, registering a second memheap does nothing and nothing warns.

The cost is a ~40-byte header per allocation in the 65 KB region. The alternative
- one merged pool - has to pick one allocator for both regions, and both choices
are wrong: small-mem over PSRAM pays the SPI-cache penalty on every access;
memheap over 65 KB spends the tightest resource on headers.

`c5heap` asserts both directions and checks contents with a position-dependent
pattern.

### DMA to PSRAM is not free

```
c5dma   : SRAM -> SRAM                 err= 0   0/256 differ
c5dma   : SRAM -> PSRAM                err=-1  256/256 differ
c5dma   : PSRAM -> SRAM                err=-1  256/256 differ
c5dma   : PSRAM -> PSRAM with cache sync: byte-exact
```

Every PSRAM direction fails without `esp_cache_msync()`, and succeeds with it.
Two rules:

* **flush** (`DIR_C2M`) before the DMA reads the CPU's source;
* **invalidate** (`DIR_M2C`) before **and** after the DMA writes the destination —
  before so a dirty line cannot be written back over the output, after so the
  CPU's next read comes from memory.

`rt_hw_dma_memcpy_cached()` does this around the cache-agnostic
`rt_hw_gdma_memcpy()`. Three constants matter: `DMA_DESCRIPTOR_BUFFER_MAX_SIZE`
is 4095 (not 4096), memheap aligns to 8 bytes while `esp_cache_msync()` needs 32,
and a request small enough to fit the 65 KB system heap will land in SRAM — so a
"PSRAM" test has to ask for far more than 65 KB and assert where it landed.

**Not proven:** a peripheral streaming into PSRAM while the CPU reads behind it.
That needs the lines around the read pointer invalidated as the transfer
advances, and it cannot be exercised without a real producer.

## Soak testing

```
msh > c5soak 2000 1
c5soak  : tick advanced 145999 ms, hardware clock advanced 145999 ms, difference 0 ms
c5soak  : internal heap used 17088 -> 17088 (worst 17088), tolerance 512
c5soak  : PSRAM free 8126400 -> 8126400 (worst 8126400), tolerance 65536
c5soak  : PASS
```

Four metrics, because one hides three failures: `used` at the end of each round,
`used` **worst** over the run, the tick against **`esp_timer_get_time()`** (an
independent hardware clock), and thread/semaphore churn.

**The clock matters.** The first version compared the tick against
`rounds * delay_ms` and reported 1439 ms of drift on a 1479 ms run — but each
round also creates threads and memsets 512 KB, none of which is sleep. It was
measuring the test's own work and calling it a scheduler fault. Comparing against
the systimer instead reads 0 ms.

**Not proven:** the absence of a leak (a few bytes per hour is invisible to a two
minute run), any driver or radio soak, and thermal drift.

## The LP I2C question, and how it was answered

For thirty rounds `c5lpi2c` was the one thing that did not work: registers
readable, both clocks enabled, pads idling high, command words well formed, and
`int_raw` empty. Then ESP-IDF's own LP I2C driver was run on the same board and
worked, which proved the hardware was fine and the fault was in this port.

Thirteen candidate differences were eliminated on evidence — command packing, the
extra `END` command, FIFO/command order, watermark values, when timing is applied,
two missing LL calls, the clock-source enable, the pad-configuration API, the
interrupt mask, the per-transfer preamble, emptying the init, clock-before-reset,
and the LP-domain clock registers (which turned out to be byte-identical).

What found it was running IDF's driver and diffing **every** register: identical
field for field apart from timing values, both correct for 100 kHz. A register
comparison cannot see a pad that is no longer wired to the peripheral — and that
is what narrowed the search to outside the controller.

The cause is in the section below. The comparison also found a real defect in this
port: `i2c_del_master_bus()` panicked with `assert failed:
vSemaphoreDeleteWithCaps`, because the FreeRTOS shim's static-buffer accessor
rejected a NULL output pointer that upstream FreeRTOS accepts.

## Two peripherals, one GDMA channel

`c5parlio` passed on its own and failed after `c5dma` in the same boot — 88 of 128
bits, `dma_txint=0x1` (TX_DONE) while PARLIO's own status stayed empty. Both
drove TX channel 0.

Five cleanups were tried and **none worked**: disconnect on exit, disconnect and
reset before configuring, explicit `owner_check(false)`, reset both halves on
exit, and sizing the burst to the frame. The failure moved (88 → 97 bits) without
ever passing.

The fix is not more cleanup. The C5 has **three** channel pairs
(`GDMA_LL_AHB_PAIRS_PER_GROUP` is 3), so they did not have to share:

```c
#define C5_GDMA_TX_CH          0    /* the M2M self test */
#define C5_PARLIO_GDMA_TX_CH   1    /* PARLIO, its own */
```

A shared resource with per-caller configuration is a bug that cleanup hides
rather than fixes. The give-away was that the number moved and never passed:
each attempt was covering *some* of the relevant fields and never all of them.

## LP I2C: a fix undone by the code beneath it

Thirteen rounds of "the FSM never starts", thirteen eliminated candidates, and the
cause was a fix this port had already made:

```c
/* Use rtc_gpio_*, not the raw rtcio_ll_* calls. */
rtc_gpio_iomux_input(pin, C5_LP_I2C_MUX_FUNC, 0);   /* pad -> LP peripheral */
...
rtcio_ll_function_select(pin, RTCIO_LL_FUNC_DIGITAL);  /* -> back to digital */
rtcio_ll_iomux_func_sel(pin, C5_LP_I2C_MUX_FUNC);      /* -> IOMUX overwritten */
```

The pad was disconnected from the LP peripheral's input. Every symptom follows:
registers readable, both clocks on, pads idling high **on the digital side**,
command words correct, `int_raw` empty because no start condition was ever seen.

```
msh > c5lpi2c
c5lpi2c: transfer to the reserved address 0x7F returned -8
c5lpi2c: PASS (the controller drove a start condition, clocked nine bits
         and saw the address NACKed)
```

**What finally found it** was running ESP-IDF's own driver and diffing every
register: identical field for field apart from timing values, both correct for
100 kHz. A register comparison cannot see a pad that is no longer wired to the
peripheral — which is exactly what narrowed the search to outside the controller.

**Not proven:** a successful data transfer. There is no I2C slave on this board, so
a NACK from a reserved address is the strongest result available.

## LP UART, and the bug that produces no warning at all

Written in one round, after LP I2C took thirteen — because that failure named the
three things to get right first: use `rtc_gpio_iomux_output/input()` and then
touch nothing else on the pad, enable the bus clock before the reset pulse, and
verify the capability macro.

```
msh > c5lpuart 115200
c5lpuart: 16 byte(s) looped back byte-exact
c5lpuart: 20 byte(s) looped back byte-exact (1 FIFO(s))
c5lpuart: 40 byte(s) looped back byte-exact (2 FIFO(s))
c5lpuart: PASS at 115200 baud
```

### `#if` on a macro that does not exist is not an error

```c
#if defined(SOC_LP_UART_NUM) && (SOC_LP_UART_NUM >= 1)
```

The real names on this part are `SOC_UART_LP_NUM` and `SOC_UART_HAS_LP_UART`. A
misspelled macro in `#if` evaluates to 0, so **the whole file compiled to nothing**
and the only symptom was `undefined reference to c5_lp_uart_selftest` three
translation units away.

### Two bugs in the test, both found by trying more than one case

**Write-all-then-read loses bytes.** The LP receive FIFO is 16 bytes, so a 20-byte
message overflows it while the CPU is still pushing the transmit side. 16 bytes
passed at 115200; **20 bytes failed at 921600 with all 20 wrong.** The 115200 case
was luck. Reading and writing are now interleaved.

**Changing a baud divisor is not a plain register write.** `c5lpuart 921600`
passed as the first call after boot and failed when it followed
`c5lpuart 115200` — the init returns early once ready, so the second call rewrote
the divisor with the loopback still on and both FIFOs holding data clocked at the
old rate.

### What it does not prove

The loopback is **internal to the peripheral**. This proves the baud generator,
shifter, both FIFOs and the LP clock — **not the pads and not a wire.** The RXD pad
level is read back and printed separately so the pad state is asserted rather than
inferred: a silently failed pad setup would leave the loopback passing while
proving nothing, which is exactly the LP I2C failure shape.

## The CAN device class worked exactly once per boot

Found by running `c5can` twice instead of once. Three stacked causes, each of which
alone produces the same symptom — `rt_device_write()` returning 0.

| layer | cause | why it hid |
|---|---|---|
| 1 | a finished TX buffer is never returned to Empty (state 8); it sits in TX_OK (4) | `c5twai` bypasses the device class entirely |
| 2 | framework box numbers and hardware TX buffer indices are **two different spaces**, and `sendmsg` discarded the box number it was given | only shows up once layer 1 makes the driver stop using buffer 0 |
| 3 | the interrupt is armed on the **first** open only, while this driver's exit path disables it on purpose | `c5can` ran once per boot, so it never re-opened |

Measured on the second run:

```
c5twai: tx_status=0x00008884 buffer(s)=4: [0]=4 [1]=8 [2]=8 [3]=8
```

State 4 on buffer 0 — **the frame had gone out and completed**, while the device
class waited for a completion no ISR would deliver.

**Each test covered the other's blind spot, and together they looked like full
coverage.** That is the general lesson: two tests that both pass can still leave a
whole path untested when they take different routes through it.

Verified now with five consecutive runs interleaved with `c5twai`:

```
c5can PASS, c5can PASS, c5twai PASS (4 rates), c5can PASS, c5twai PASS, c5can PASS
```

### A stale pin table certified a conflict

The LP UART's pads are fixed by the LP IOMUX (this part does not define
`SOC_LP_GPIO_MATRIX_SUPPORTED`), so IO4/IO5 belong to it. CAN and PCNT moved to
IO26/IO27 — both go through the GPIO matrix.

Measured before the move: `c5can` PASS, `c5lpuart` PASS, `c5can` **FAIL**.
`c5pins` reported PASS throughout, because its table still listed IO2 as SPI MISO
and IO3 as the PWM test — both had already moved to IO23/IO24 — and did not know
the LP peripherals existed. **A pin table that is not maintained is worse than
none: it certifies a conflict.**

## Every command run twice

The CAN bug was a **repeatability** defect: the driver worked once per boot and
nothing noticed, because every self test ran exactly once. Running everything
twice found one more.

```
c5spiloop -> PASS, loopback identical
c5spiloop -> attach to spi0 failed
```

`rt_spi_bus_attach_device()` ends in `rt_device_register(device, "spiloop", ...)`
(`dev_spi.c:164`), and the object manager refuses a duplicate name. The
`struct rt_spi_device` is `static`, so it survives between invocations and is
already the registered device — the second attach was being asked to create
something that existed and was correct. Fixed with `rt_device_find()` first.

**A test that can only pass once is not testing the driver, it is testing the
first call.**

Everything else survives a second run: `c5rmt`, `c5pcnt`, `c5mcpwm`, `c5tsens`,
`c5adc`, `c5dma`, `c5parlio`, `c5usj`, `c5heap`, `c5psram`, `c5ulp`, `c5lpi2c`,
`c5lpuart` — all PASS twice in one boot, and `c5can` five times interleaved with
`c5twai`.

### What the sweep does not cover

Defects that need a **reboot** to appear, and defects that need a **specific other
command** to have run first. The interleaved `c5can`/`c5twai` run covers one such
pair; the rest are not covered, and saying so is more useful than implying the
sweep was exhaustive.

## The Wi-Fi re-join crash: four layers

`c5join5` worked once per boot and crashed on the second call. Four independent
causes, each of which alone produced a failure, and each of which had to be fixed
before the next became visible.

| layer | cause | symptom |
|---|---|---|
| 1 | `esp_netif_create_default_wifi_sta()` called again | **assert panic**, reboot |
| 2 | `esp_wifi_init()` guarded the documented `ESP_ERR_WIFI_INIT_STATE` (0x3013), but the observed code is `ESP_ERR_INVALID_STATE` (0x103) | re-join refused |
| 3 | `esp_wifi_connect()` returns 0 and does nothing when already associated | 30 s timeout |
| 4 | **`FT_IRAM` expanded to nothing on RISC-V** | `panic'ed (Cache error)` |

### Layer 4

```c
#if defined(__XTENSA__)
#  define FT_IRAM    __attribute__((section(".iram1")))
#else
#  define FT_IRAM                              /* empty */
#endif
```

The comment justifying the `#else` said the cache "is not shared with flash the
same way" on non-Xtensa targets. **The ESP32-C5 is RISC-V and has exactly the same
flash cache.** The guard did not prevent the resets it was written for — it
excluded every RISC-V target from the protection.

```
MEPC = 0x4206f21e  ft_ipc_resolve            (flash)
RA   = 0x408095bc  xQueueGenericSendFromISR  (IRAM)
```

One side of the same call in IRAM, the other in flash. Wi-Fi writes NVS while
associating, which disables the flash cache; an interrupt in that window jumps into
flash and dies — intermittently, which is why it took so long.

Both halves needed fixing: `FT_IRAM` unconditional (and on `ft_ipc_ptr_plausible`,
which the previous round's diagnostics had added *inside* the ISR path), and the
kernel side via `linker.lf` — `object`, `rt_thread_resume`, `rt_schedule`, the
scheduler helpers. `linker.lf` already placed the **trap** path in IRAM for the
same reason; the FreeRTOS shim's ISR path was a different way in that it did not
cover.

Cost: **3.8 KB** of IRAM, with 79.3 KB still free.

### Two rounds on the wrong cause

The panic's `MEPC` was inside `ft_ipc_resolve`'s prologue — a `void` function that
dereferences nothing. The value `0x40860000` in the argument registers was read as
"a bad handle caused this", and two rounds went into a range check, with two wrong
sets of bounds: an 80 MB window that **admitted** the address it was written to
reject, then `__heap_end__` as the ceiling that **rejected a legitimate** IDF handle
at `0x4085ce9c`.

Register contents after a cache fault carry no meaning. **The bad handle was a
symptom.** What settled it was cheap: `nm` showed one side of the call in IRAM and
the other in flash.

## Testing TCP: three ways to make it look broken

`c5tcp` appeared to fail intermittently with `assert failed:
tcpip_send_msg_wait_sem (Invalid mbox)`. That assertion is real but it is **not a
port defect** - three mistakes in how the test was invoked produced it, and ten
rounds were spent chasing it.

| mistake | what it looks like |
|---|---|
| `c5tcp` run **without** `c5join5` in the same boot | the assert, then a reboot; the next run asserts again |
| a server that accepts **one** connection (`tcpecho.py`) | `errno=104` on runs 2 and 3 |
| a reply other than `"pong-from-pc"` | `received 4 byte(s): pong` then `FAIL, reply is not the expected one` |

`tcpip_mbox` is created by `tcpip_init()` inside `esp_netif_init()`; a socket
call before that is a NULL mailbox and asserts. So:

```
msh > c5join5                                  # required, same boot
msh > c5tcp <host-ip> 3334                     # with tcpecho_loop.py running
tcp     : connected to 10.0.0.159:3334
tcp     : sent 12 byte(s)
tcp     : received 12 byte(s): pong-from-pc
```

Use `tools/tcpecho_loop.py <port> pong-from-pc` - it accepts repeatedly, unlike
`tcpecho.py`, and answers with the string `c5_tcp` checks for.

**Lessons, both about measurement:** when a fault is intermittent, check whether
the test is; and a reading taken after a reboot is not a reading of the same
state.

## IRAM: the same fault six times

`Invalid mbox` after repeated Wi-Fi joins was **IRAM code calling flash code**.
It took six layers to reach the bottom because each fix moved the crash:

| # | `MEPC` (flash) | fixed by |
|---|---|---|
| 1 | `ft_ipc_resolve` | `FT_IRAM` unconditional |
| 2 | `rt_sem_trytake` | `ipc (noflash)` |
| 3 | `rt_object_get_type` | `object (noflash)` |
| 4 | `ft_ipc_push` | `ft_ipc (noflash)` |
| 5 | `rt_memcpy` | `kstring (noflash)` |
| 6 | `ft_qset_signal` | the remaining shim files |

**That is why it looked intermittent: the crash site moved every time.** What
identified it was a coordinate, not a hypothesis — `MEPC` in `0x420xxxxx` (flash)
while `RA` was in `0x4080xxxx` (IRAM). One side of the same call in each region.
It was visible in every panic dump that had already been captured.

The entry points had `FT_IRAM` from the first fix; their **static helpers in the
same file did not**. A file whose entry points are ISR-callable must be placed as
a unit.

```
_iram_end      = 0x40820b00   (130.8 KB)
__heap_start__ = 0x40834b70   (79.2 KB still free)
```

Six consecutive joins, all `GOT_IP`.

**Not proof that a seventh layer does not exist.** The headroom is finite and the
approach does not scale — the kernel is ~105 KB of text. The structural fix is an
ISR path that does not call the kernel (FreeRTOS's `FromISR` contract is
lightweight by design; this shim forwards to a locking kernel).

## One open issue

~~`Invalid mbox`~~ **Resolved above** - it was IRAM/flash mixing, six layers deep.
The section is kept for the diagnostic advice, which still applies.

Ruled out with evidence: `tcpip_init()` not running (it does; `mbox` reads
non-NULL), `c5_ble_init()` touching lwIP (it does not), `sys_mbox_new()` failing
(never observed), `nimble_port_init()` failing (returns 0), a stale host server
and buffered output hiding lines.

The same sequence also **succeeded** in a later full run:

```
c5join5    -> GOT_IP
c5iperf 3s -> 21418 kbit/s
c5all      -> step1/2/3 all 0, PASS, alive after 60 s
```

If you hit it, start from: capture **unfiltered** output to a file, confirm the
board did not reboot since setup, use `tcpecho_loop.py` with the reply
`pong-from-pc`, and record the value in RTC-noinit memory at the assert site -
that survives the panic, which `.bss` does not.

## A known limit: repeated Wi-Fi re-join

Four `c5join5` in one boot aborts with

```
RT-Thread scheduler entry while flash cache is disabled
```

`rt_schedule()` is reached while the Wi-Fi driver writes NVS with the flash cache
off. Six attempts to fix it by moving code into IRAM each moved the crash to the
next function (`ft_ipc_resolve`, `rt_sem_trytake`, `rt_object_get_type`,
`ft_ipc_push`, `rt_memcpy`, `ft_qset_signal`) and never eliminated it. A seventh -
`rt_interrupt_enter/leave` around the shim ISR - changed nothing, because
`rt_schedule()` calls `rt_thread_self()` *before* it looks at any nest counter.

The real constraint:

```
IRAM in use      130.8 KB
IRAM still free   79.2 KB
rt-thread text   ~105 KB
```

The kernel cannot be moved in, and moving enough to make one call safe leaves the
next one unsafe.

**The correct fix is a redesign**: `ft_ipc_send_isr` must call no kernel API at
all - spinlock, append, set `*woken`, return - with the wake performed where the
cache is guaranteed on. That changes `ft_ipc.c`, `ft_qset.c` and `ft_event.c`
together.

What works: single `c5join5`, TCP round trips, 13-21 Mbit/s throughput, and ten
joins one call each across fresh boots. What does not: four joins inside one boot.

## The pre-build guard check

Nine separate times, an Xtensa debugging probe added to shared code broke the
C5 build — and always at **link time**, minutes in, as a wall of undefined
references. `rt_xt_br`, `rt_xt_nf`, `rt_xt_ip3`, `xt_rt_flash_op` and friends live
only in `libcpu/xtensa/` or the S3 BSP.

`tools/check_xtensa_guards.py` walks the tree, tracks `#if` nesting, and reports
any reference that is not inside an `#if defined(__XTENSA__)` region:

```
components/rtthread/port/freertos/ft_notify.c:179: rt_xt_nf -> { extern volatile unsigned int rt_xt_nf[8];
```

`build.ps1` runs it as a gate (~1 s). `-SkipGuardCheck` bypasses it. **It was
verified both ways** — a deliberately inserted reference makes it fail and makes
the build refuse to run.

---

## Repository layout

```
port/esp32c5/          the port itself: 21 sources + 21 headers
project/               the ESP-IDF project that exercises them
  main/                shell commands, Wi-Fi, BLE, SoftAP, LP core test
  main/ulp/            the LP-core-side program (separate toolchain)
  sdkconfig.defaults   the known-good configuration
  partitions.csv       16 MB flash, 2 MB app slot
tools/                 serial harness, build wrapper, guard check
docs/
  PORTING-REPORT.md    ~2700 lines: what worked, what failed, and why
  cmake-c5-sections.md the CMake edits you have to apply
```

---

## Things that cost the most, in case they save you the same time

Each of these is one line in the report and hours on the board.

* **Output-enable polarity.** `gpio_ll_set_output_enable_ctrl(hw, pin, ctrl_by_periph, oen_inv)` — the second bool is **invert**, not enable. Passing `true` disables the pad exactly when the peripheral drives it. Hit this in RMT, SPI, I2C and PARLIO.
* **Carrier modulators default ON.** Both `chnconf1.carrier_en_chn` (TX) and `chmconf0.carrier_en_chm` (RX) reset to 1. The RX one made RMT receive exactly zero symbols for three rounds.
* **Clock sources default to a PLL nobody enabled.** ADC, TWAI, RMT, LEDC and PARLIO all had to be pointed at XTAL explicitly.
* **`int_st` is masked.** `int_ena` reads 0 out of reset, so a polling driver that waits on `int_st` waits forever. Read `int_raw`. This cost a complete round on PARLIO.
* **A lost edit is invisible until the path it fixes runs again.** The `locks.c` ISR fix was silently reverted and stayed invisible for three rounds.
* **`--gc-sections` also drops code nothing references.** The ADC calibration component was compiled and linked into every image and then discarded, because nothing called it - `nm` on the final ELF found no `adc_cali` symbols while `libesp_adc.a` had them. `-u` in `target_link_libraries` is the fix, for a whole feature and not just for drivers.
* **Never rewrite an important file with `open(..., "w")`.** The report was truncated to zero bytes once that way.

---

## License

The port itself (`port/`, `project/`) is Apache-2.0, matching ESP-IDF.
RT-Thread is Apache-2.0. See [LICENSE](LICENSE).

---

## Status

Kernel, radios, eight RT-Thread devices and eleven self-tested peripherals work.
LP peripherals, SDIO slave and `rt_malloc()`-into-PSRAM do not. The report's
§21 lists the rest.
