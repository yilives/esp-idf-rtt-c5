/*
 * Board pin map for the Maple ESP32-S31 Alef (XPU Labs).
 *
 * Source: the board's own user guide and schematic, checked in
 *   docs/开发板资料/docs/maple_esp32/src/maple_esp32s31/ug0001_..._user_guide.md
 *   docs/开发板资料/docs/maple_esp32/src/maple_esp32s31/static/images/
 *   SCH_Schematic1_2026-10-30.pdf
 *
 * This file exists because the port was written before the board was known, and
 * two of its guesses were wrong in a way that only shows up as "nothing on the
 * bus": I2C is on IO50/IO51 on this board, not on the low-numbered pads the
 * generic examples use, and the LCD header shares IO3..IO7 with the I2S/DVP
 * header.
 *
 * Verified against soc_caps.h: none of these pads is GPIO29 or GPIO41, so all of
 * them pass SOC_GPIO_VALID_GPIO_MASK.
 */

#ifndef __PORT_BOARD_H__
#define __PORT_BOARD_H__

/* ---------------------------------------------------------------- I2C -- */
/*
 * The board labels these "I2C_SCL" / "I2C_SDA" on the audio/video header
 * (IO50 / IO51). They are the *only* I2C pads the board brings out, and the
 * on-board camera header reuses IO0/IO1 for its own I2C (CAM_I2C_SDA/SCL), so
 * the generic i2c0 must not take those.
 *
 * This is the correction of the previous guess (IO4/IO5): those pads are
 * I2S_LRCK and I2S_DSDIN on this board, and driving them from the I2C
 * peripheral silently produced "0 devices" on a bus nothing was connected to.
 */
#define BOARD_I2C0_SCL_PIN      50
#define BOARD_I2C0_SDA_PIN      51

/* ------------------------------------------------------------ RGB LED -- */
/*
 * Single-wire addressable LED, SK6805-2427 (WS2812-compatible), data in on
 * IO60. It is driven by RMT in the IDF examples; a bit-banged driver would work
 * at 320 MHz but lose interrupts for the whole transfer.
 */
#define BOARD_RGB_LED_PIN       60

/* ------------------------------------------------------------- buttons -- */
/*
 * IO61 is BOOT1, wired to GND with the button - so it reads 1 when released
 * and 0 when pressed. (The ROM download-mode BOOT strap is the same pin; the
 * board's auto-download circuit drives it from the CH343P.)
 */
#define BOARD_BOOT_BUTTON_PIN   61

/* ------------------------------------------------------------ SPI LCD -- */
/*
 * The 12-pin LCD header (J1). Net names come from the schematic:
 *   LCD_DC  FP_IO3     LCD_SPI1_CS   FP_IO4
 *   LCD_SPI1_SCK  FP_IO5   LCD_SPI1_MOSI  FP_IO6
 *   LCD_RST FP_IO7     backlight cathode LED_K (through R10 10R)
 *
 * NOTE: these pads are shared with the I2S/DVP header (IO3 = I2S_SCLK,
 * IO4 = I2S_LRCK, IO5 = I2S_DSDIN, IO6 = I2S_SDOUT, IO7 = PA_CTRL), so an LCD
 * and I2S cannot both be used at once. That is a board limitation, not a driver
 * one - do not "fix" it by remapping.
 */
#define BOARD_LCD_DC_PIN        3
#define BOARD_LCD_CS_PIN        4
#define BOARD_LCD_SCLK_PIN      5
#define BOARD_LCD_MOSI_PIN      6
#define BOARD_LCD_RST_PIN       7

/* -------------------------------------------------------------- SPI buses -- */
/*
 * spi0: free pins on the 1x20P header, for a breadboarded peripheral.
 * spi1: a second, independent bus. Deliberately NOT the LCD header (IO3..IO7) -
 *       those are shared with I2S and are claimed on demand by the LCD driver,
 *       not at boot.
 *
 * None of these overlap the I2C pair (IO50/IO51), the RGB LED (IO60), BOOT1
 * (IO61) or the in-package flash pads (29/41).
 */
#define BOARD_SPI0_SCLK_PIN     8
#define BOARD_SPI0_MOSI_PIN     9
#define BOARD_SPI0_MISO_PIN     10
#define BOARD_SPI0_CS_PIN       11

#define BOARD_SPI1_SCLK_PIN     12
#define BOARD_SPI1_MOSI_PIN     13
#define BOARD_SPI1_MISO_PIN     14
#define BOARD_SPI1_CS_PIN       15

/* --------------------------------------------------------------- MCPWM -- */
/*
 * One complementary pair (A / B) for the MCPWM self test. The pads are chosen
 * so that A can also be read back by PCNT (edge counting) and by mcycle polling
 * (duty), which is how `s31mcpwm` verifies itself without a scope.
 *
 * GPIO20/21 are free on this board: not the I2C pair (50/51), not the RGB LED
 * (60), not BOOT1 (61), not the in-package flash pads (29/41), and not the
 * LCD/I2S header (3..7).
 */
#define BOARD_MCPWM_A_PIN       20
#define BOARD_MCPWM_B_PIN       21

/* ---------------------------------------------------------------- RMT -- */
/*
 * RMT TX channel 0 for the square-wave self test.
 *
 * Deliberately NOT IO60 (the on-board SK6805): `s31led` owns that pad, and the
 * square wave's whole purpose is to be measurable - which the LED data line is
 * not. GPIO22 is free on this board.
 */
#define BOARD_RMT_TX_PIN        22

/* ---------------------------------------------------------------- TWAI -- */
/*
 * CAN TX / RX for the self-test.
 *
 * GPIO23/24 are free on this board. In self-test mode the controller loops TX
 * back to RX internally, so the two pads need not be joined - but routing both
 * is what makes the test meaningful once a transceiver is fitted.
 */
#define BOARD_TWAI_TX_PIN       23
#define BOARD_TWAI_RX_PIN       24

/* ----------------------------------------------------------------- SD -- */
/* SDIO 4-bit slot. Module pads 27..32; the GPIO numbers behind them are in the
 * schematic's net names (SD_D0..SD_D3, SD_CLK, SD_CMD) and are resolved by the
 * SDMMC driver from the slot, not from this map. */

/* ----------------------------------------------------------------- I2S -- */
/* Shared with the LCD header - see the note above. */
#define BOARD_I2S_MCLK_PIN      2
#define BOARD_I2S_SCLK_PIN      3
#define BOARD_I2S_LRCK_PIN      4
#define BOARD_I2S_DSDIN_PIN     5
#define BOARD_I2S_SDOUT_PIN     6
#define BOARD_I2S_PA_CTRL_PIN   7

/* ----------------------------------------------------------- DVP camera -- */
/* LCD_CAM DVP, on the same header: Y2..Y9 = IO46..IO53, plus the syncs. */
#define BOARD_CAM_XCLK_PIN      55
#define BOARD_CAM_PCLK_PIN      54
#define BOARD_CAM_VSYNC_PIN     56
#define BOARD_CAM_HREF_PIN      57
#define BOARD_CAM_SDA_PIN       0
#define BOARD_CAM_SCL_PIN       1

#endif /* __PORT_BOARD_H__ */
