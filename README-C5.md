# ESP32-C5 + RT-Thread — full buildable fork

This is a fork of [RT-Thread-packages/esp-idf](https://github.com/RT-Thread-packages/esp-idf)
whose default branch is the **ESP32-C5 port**. Cloning it gives you a tree that
builds as-is: ESP-IDF v6.1 with RT-Thread 5.4.0 as the kernel, plus the C5 port.

If you only want the port and the project — without 350 MB of ESP-IDF history —
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
| **LP I2C** | controller reachable, both clocks enabled, pads handed over, **the FSM never starts**. **IDF's own driver works on the same board** (§17y), so the fault is in this port's driver; six candidate differences are eliminated. |
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

## The LP I2C question, and how it was narrowed

`c5lpi2c` drives the low-power I2C controller by hand and the FSM never starts:
registers readable, both clocks enabled, pads idling high, command words well
formed, and `int_raw` empty. Thirty rounds of that looked like hardware.

It is not. **ESP-IDF's own LP I2C driver works on the same board**, same pads,
same registers:

```
c5lpi2cidf: i2c_master_transmit to 0x7F -> 0x108 (NACK - the bus WAS driven)
```

`0x108` is `ESP_ERR_INVALID_RESPONSE` = "master transmit receives NACK", and a NACK
from a reserved address is the success case: a start condition went out and nine
bits were clocked.

Six differences were then found and removed — command packing, the extra `END`
command, FIFO/command order, watermark values, when timing is applied, and two
missing LL calls — and **none of them fixed it**. That is recorded rather than
hidden: the next candidate is the LP-domain clock select, and it is named as a
hypothesis, not a finding.

The comparison also found a real defect in this port: `i2c_del_master_bus()`
panics with `assert failed: vSemaphoreDeleteWithCaps` because the FreeRTOS shim's
semaphore wrapper refuses a delete it did not hand out.

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
