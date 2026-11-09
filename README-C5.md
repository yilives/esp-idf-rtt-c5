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
| **LP I2C** | controller is reachable, both clocks read enabled, pads handed over, **the FSM never starts**. See [the report §17t](docs/PORTING-REPORT.md). |
| **LP UART / LP peripherals** | not attempted — blocked on the same unknown as LP I2C |
| **SDIO slave** | needs a host on the other end |
| **`rt_malloc()` → PSRAM** | the 8 MB is in IDF's `heap_caps` heap only; RT-Thread's allocator is still 65 KB. Bridging them is a design decision, not a bug. |
| **PARLIO RX** | TX only |
| **Analog layer** | no transceiver for CAN, no scope for MCPWM dead time, no host for USB |
| **ADC accuracy** | millivolts are computed, but accuracy against a reference is not measured |

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
