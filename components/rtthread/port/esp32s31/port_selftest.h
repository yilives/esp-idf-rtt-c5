/*
 * ESP32-S31 bring-up self tests.
 */

#ifndef __PORT_SELFTEST_H__
#define __PORT_SELFTEST_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Compile-time tick sanity check, run from rt_hw_board_init().
 *
 * RT_TICK_PER_SECOND must equal CONFIG_FREERTOS_HZ: the tick comes from the
 * IDF SYSTIMER alarm, so a mismatch scales every delay and timeout silently.
 * The check is done at compile time; this function only exists so the linker
 * keeps the object that also carries the msh commands.
 *
 * Renamed from s31_selftest_init(): the function does no initialisation, it exists
 * only so the linker has a reference to an object whose commands live in a data
 * section. The declaration here was left behind by that rename, which meant this
 * header promised a symbol that no longer existed.
 */
void s31_selftest_link_anchor(void);

#ifdef __cplusplus
}
#endif

#endif /* __PORT_SELFTEST_H__ */
