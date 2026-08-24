/*-
 * Copyright (c) 2013
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
/*
 * QEMU stm32f2xx RTC emulation
 * Ported to QEMU 10.x APIs.
 */
#include "qemu/osdep.h"
#include <sys/time.h>
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "qemu/bcd.h"
#include "qemu/cutils.h"
#include "system/rtc.h"
#include "qemu/timer.h"
#include "hw/arm/stm32_common.h"
#include "qemu/log.h"
#include "qapi/error.h"

// Define this to add extra BKUP registers past the normal ones implemented by the STM.
// For Pebble emulation, we use these to pass settings flags to the emulated target
#define STM32F2XX_RTC_NUM_EXTRA_BKUP_REG  1

//#define DEBUG_STM32F2XX_RTC
#ifdef DEBUG_STM32F2XX_RTC
// NOTE: The usleep() helps the MacOS stdout from freezing when we have a lot of print out
#define DPRINTF(fmt, ...)                                       \
    do { printf("DEBUG_STM32F2XX_RTC: " fmt , ## __VA_ARGS__); \
         usleep(100); \
    } while (0)
#else
#define DPRINTF(fmt, ...)
#endif



#define R_RTC_TR     (0x00 / 4)
#define R_RTC_DR     (0x04 / 4)
#define R_RTC_CR     (0x08 / 4)
#define R_RTC_CR_ALRAE_BIT 8
#define R_RTC_CR_WUTE   0x00000400
#define R_RTC_CR_WUTIE  0x00004000

#define R_RTC_ISR    (0x0c / 4)
#define R_RTC_ISR_RESET 0x00000007
#define R_RTC_ISR_RSF   0x00000020
#define R_RTC_ISR_WUT   0x00000400

#define R_RTC_PRER   (0x10 / 4)
#define R_RTC_PRER_PREDIV_A_MASK 0x7f
#define R_RTC_PRER_PREDIV_A_SHIFT 16
#define R_RTC_PRER_PREDIV_S_MASK 0x1fff
#define R_RTC_PRER_PREDIV_S_SHIFT 0
#define R_RTC_PRER_RESET 0x007f00ff
#define R_RTC_WUTR   (0x14 / 4)
#define R_RTC_WUTR_RESET 0x0000ffff
#define R_RTC_CALIBR (0x18 / 4)
#define R_RTC_ALRMAR (0x1c / 4)
#define R_RTC_ALRMBR (0x20 / 4)
#define R_RTC_WPR    (0x24 / 4)
#define R_RTC_SSR    (0x28 / 4)
#define R_RTC_TSTR   (0x30 / 4)
#define R_RTC_TSDR   (0x34 / 4)
#define R_RTC_TAFCR  (0x40 / 4)
#define R_RTC_BKPxR  (0x50 / 4)
#define R_RTC_BKPxR_LAST (0x9c / 4)
#define R_RTC_BKPxR_INC_EXTRA_LAST (R_RTC_BKPxR_LAST + STM32F2XX_RTC_NUM_EXTRA_BKUP_REG)
#define R_RTC_MAX    (R_RTC_BKPxR_INC_EXTRA_LAST + 1)

#define R_RTC_CR_FMT_MASK (0x01 << 6)

#define DEBUG_ALARM(x...)

typedef struct f2xx_rtc {
    SysBusDevice  parent_obj;
    MemoryRegion  iomem;
    QEMUTimer     *timer;
    QEMUTimer     *wu_timer;
    qemu_irq      irq[2];
    qemu_irq      wut_irq;

    // target_us = host_us + host_to_target_offset_us
    int64_t       host_to_target_offset_us;

    // target time in ticks (seconds according to the RTC registers)
    time_t        ticks;

    uint32_t      regs[R_RTC_MAX];
    int           wp_count; /* Number of correct writes to WP reg */

    /* Last level driven onto each outgoing IRQ line. The EXTI model that these
     * lines feed edge-detects every qemu_set_irq() call, so we must only signal
     * an IRQ on an actual level change -- otherwise re-asserting an already-high
     * alarm line (e.g. while ALRxF is still set) looks like a fresh edge and
     * produces spurious extra interrupts. */
    bool          alarm_irq_level[2];
    bool          wut_irq_level;
} f2xx_rtc;


// Update target date and time from the host
static void f2xx_update_current_date_and_time(void *arg);

// Side-effect-free refresh of the TR/DR registers (used by the read path)
static void f2xx_rtc_refresh_time_and_date(f2xx_rtc *s);


// Drive an alarm IRQ line, but only emit a signal when the level actually
// changes. The downstream EXTI controller treats every qemu_set_irq() call as a
// potential edge, so re-asserting an already-high line would spuriously re-pend
// the interrupt once the guest has cleared the EXTI pending bit.
static void
f2xx_rtc_set_alarm_irq(f2xx_rtc *s, int unit, int level)
{
    level = !!level;
    if (s->alarm_irq_level[unit] == level) {
        return;
    }
    s->alarm_irq_level[unit] = level;
    qemu_set_irq(s->irq[unit], level);
}

// Same edge-change filtering for the wake-up timer IRQ line.
static void
f2xx_rtc_set_wut_irq(f2xx_rtc *s, int level)
{
    level = !!level;
    if (s->wut_irq_level == level) {
        return;
    }
    s->wut_irq_level = level;
    qemu_set_irq(s->wut_irq, level);
}


// Compute the period for the clock (seconds increments) in nanoseconds
static uint64_t
f2xx_clock_period_ns(f2xx_rtc *s)
{
    uint32_t prer = s->regs[R_RTC_PRER];
    unsigned int prescale;

    // NOTE: We are making the assumption here, as in f2xx_wut_period_ns, that RTC_CLK
    // is the 32768 LSE clock
    prescale = (((prer >> R_RTC_PRER_PREDIV_A_SHIFT) & R_RTC_PRER_PREDIV_A_MASK) + 1) *
               (((prer >> R_RTC_PRER_PREDIV_S_SHIFT) & R_RTC_PRER_PREDIV_S_MASK) + 1);
    uint64_t result = 1000000000LL * prescale / 32768;

    //DPRINTF("%s: period = %lldns\n", __func__, result);
    return result;
}


// Compute the period for the wakeup timer in nanoseconds if the WUT counter is set to the
// given value
static uint64_t
f2xx_wut_period_ns(f2xx_rtc *s, uint32_t counter)
{
    uint32_t clock_sel = s->regs[R_RTC_CR] & 0x07;

    // NOTE: We are making the assumption here, as in f2xx_clock_period_ns, that RTC_CLK
    // is the 32768 LSE clock
    uint64_t rtc_clk_period_ns = 1000000000LL / 32768;

    // Using the synchronous clock (clk_spre)
    if (clock_sel & 0x04) {
        if (clock_sel & 0x02) {
            return counter * f2xx_clock_period_ns(s);
        } else {
            return (counter + 65536) * f2xx_clock_period_ns(s);
        }
    } else if (clock_sel == 0) {
        return counter * rtc_clk_period_ns * 16;
    } else if (clock_sel == 1) {
        return counter * rtc_clk_period_ns * 8;
    } else if (clock_sel == 2) {
        return counter * rtc_clk_period_ns * 4;
    } else if (clock_sel == 3) {
        return counter * rtc_clk_period_ns * 2;
    } else {
        abort();
    }
}


// Set the time and registers based on the content of the passed in tm struct
static void
f2xx_rtc_set_time_and_date_registers(f2xx_rtc *s, struct tm *tm)
{
    uint8_t wday;

    wday = tm->tm_wday == 0 ? tm->tm_wday : 7;
    s->regs[R_RTC_TR] = to_bcd(tm->tm_sec) |
                        to_bcd(tm->tm_min) << 8 |
                        to_bcd(tm->tm_hour) << 16;
    s->regs[R_RTC_DR] = to_bcd(tm->tm_mday) |
                        to_bcd(tm->tm_mon + 1) << 8 |
                        wday << 13 |
                        to_bcd(tm->tm_year % 100) << 16;
}


// Compute what time we want in the target based on the host's current date and time.
// This takes into consideration the host_to_target_offset_us we have computed and captured
// previously.
// The rtc_period_ns passed in is the number of nanoseconds in host time that corresponds to 1
// "tick" (i.e. a second increment in the RTC register). Usually, you would expect this to
// be 1e9, but on Pebble plastic for example, the firmware sets up the RTC so that the
// seconds increment 1024 times/sec.
static time_t
f2xx_rtc_compute_target_time_from_host_time(f2xx_rtc *s, uint64_t rtc_period_ns,
                                            struct tm *target_tm)
{
    // Get the host time in microseconds
    struct timeval tv;
    gettimeofday(&tv, NULL);
    int64_t host_time_us = tv.tv_sec * 1000000LL + (tv.tv_usec);

    // Apply local timezone offset if provided via environment (e.g. from browser JS).
    // TZ_OFFSET_SEC is seconds east of UTC (e.g. -28800 for PST).
    static int64_t tz_offset_us = 0;
    static int tz_checked = 0;
    if (!tz_checked) {
        const char *tz_str = getenv("TZ_OFFSET_SEC");
        if (tz_str) {
            tz_offset_us = (int64_t)atoi(tz_str) * 1000000LL;
        }
        tz_checked = 1;
    }

    // Compute the target time by adding the offset
    int64_t target_time_us = host_time_us + s->host_to_target_offset_us + tz_offset_us;

    // Convert to target ticks according period set in the RTC
    time_t target_time_ticks = (target_time_us * 1000) / rtc_period_ns;

    // Convert to date, hour, min, sec components (gmtime since we already applied tz offset)
    gmtime_r(&target_time_ticks, target_tm);


#ifdef DEBUG_STM32F2XX_RTC
    char new_date_time[256];
    strftime(new_date_time, sizeof(new_date_time), "%c", target_tm);
    //DPRINTF("%s: setting new date & time to: %s\n", __func__, new_date_time);
#endif
    return target_time_ticks;
}


// Return the current date and time as stored in the RTC TR and DR registers in two forms:
// By filling in the passed in tm struct and by returning the UTC time in seconds.
static time_t
f2xx_rtc_get_current_target_time(f2xx_rtc *s, struct tm *target_tm)
{
    memset(target_tm, 0, sizeof(*target_tm));

    // Fill in the target_tm from the contents of the time register and date registers
    uint32_t time_reg = s->regs[R_RTC_TR];

    target_tm->tm_hour = from_bcd((time_reg & 0x003F0000) >> 16);
    bool pm = (time_reg & 0x00800000) != 0;
    if (pm && (s->regs[R_RTC_CR] & R_RTC_CR_FMT_MASK)) {
        target_tm->tm_hour += 12;
    }
    target_tm->tm_min = from_bcd((time_reg & 0x00007F00) >> 8);
    target_tm->tm_sec = from_bcd(time_reg & 0x0000007F);

    uint32_t date_reg = s->regs[R_RTC_DR];
    target_tm->tm_mday = from_bcd(date_reg & 0x3f);
    target_tm->tm_mon = from_bcd((date_reg & 0x00001F00) >> 8) - 1;
    target_tm->tm_year = from_bcd((date_reg & 0x00FF0000) >> 16) + 100;

    // Have mktime fill in the remaining fields and return the UTC seconds as well
    return mktimegm(target_tm);
}



// Compute the host to target offset based on the passed in target ticks and the current
//  host time.
static int64_t
f2xx_rtc_compute_host_to_target_offset(f2xx_rtc *s, int64_t period_ns, time_t target_ticks)
{
    // Convert target ticks to real clock microseconds
    int64_t target_time_us = target_ticks * period_ns / 1000;

    // Get the host time in microseconds
    struct timeval tv;
    gettimeofday(&tv, NULL);
    int64_t host_time_us = tv.tv_sec * 1000000LL + (tv.tv_usec);

    // Get the host to target offset in micro seconds
    return target_time_us - host_time_us;
}


static uint64_t
f2xx_rtc_read(void *arg, hwaddr addr, unsigned int size)
{
    f2xx_rtc *s = arg;
    uint32_t r;
    int offset = addr & 0x3;

    addr >>= 2;
    if (addr >= R_RTC_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "invalid read f2xx rtc register 0x%x\n",
          (unsigned int)addr << 2);
        DPRINTF("  %s: result: 0\n", __func__);
        return 0;
    }

    // If reading the time or date register, make sure they are brought up to
    // date first. Use the side-effect-free refresh: a read must not check
    // alarms or raise IRQs, otherwise reading RTC_TR from inside the firmware's
    // tick ISR can re-raise the alarm and produce a duplicate tick event.
    if (addr == R_RTC_TR || addr == R_RTC_DR) {
        f2xx_rtc_refresh_time_and_date(s);
    }

    uint32_t value = s->regs[addr];
    if (addr == R_RTC_ISR) {
        value |= R_RTC_ISR_RSF;
    }

    // HACK for Pebble. Clear the "entered standby" bit. If this bit is set, the Pebble
    // will only continue booting if one of the buttons is held down. Because the button GPIOs
    // are setup AFTER QEMU gets the key press event, it is difficult to set the GPIOs according
    // to the buttons held down before the reset.
    if (addr == R_RTC_BKPxR) {
        value &= ~0x10000;
    }

    // If reading the sub-second register, determine what it should be from the current
    //  host time
    if (addr == R_RTC_SSR) {
        // How many ns are in in a full cycle of the subsecond counter?
        uint32_t prer = s->regs[R_RTC_PRER];;
        uint32_t prer_s = (prer >> R_RTC_PRER_PREDIV_S_SHIFT) & R_RTC_PRER_PREDIV_S_MASK;

        uint64_t full_cycle_us = f2xx_clock_period_ns(s) / 1000;

        // What fraction of a full cycle are we in?
        struct timeval tv;
        gettimeofday(&tv, NULL);
        int64_t host_time_us = tv.tv_sec * 1000000LL + (tv.tv_usec);
        host_time_us += s->host_to_target_offset_us;

        int64_t host_mod = host_time_us % full_cycle_us;
        double fract = (double)host_mod / full_cycle_us;

        // Compute the prescaler value
        value = prer_s - (fract * prer_s);
        DPRINTF("%s: SSR value = %d\n", __func__, value);
    }

    r = (value >> offset * 8) & ((1ull << (8 * size)) - 1);

#ifdef DEBUG_STM32F2XX_RTC
    // Debug printing
    if (addr == R_RTC_TR || addr == R_RTC_DR) {
        struct tm target_tm;
        f2xx_rtc_get_current_target_time(s, &target_tm);

        char date_time_str[256];
        strftime(date_time_str, sizeof(date_time_str), "%x %X", &target_tm);
        DPRINTF("%s: current date/time: %s\n", __func__, date_time_str);
    } else {
        DPRINTF("%s: addr: 0x%llx, size: %d, value: 0x%x\n", __func__, addr << 2, size, r);
    }
#endif

    return r;
}

static void
f2xx_rtc_write(void *arg, hwaddr addr, uint64_t data, unsigned int size)
{
    f2xx_rtc *s = arg;
    int offset = addr & 0x3;
    bool    compute_new_target_offset = false;
    bool    update_wut = false;

    DPRINTF("%s: addr: 0x%llx, data: 0x%llx, size: %d\n", __func__, addr, data, size);

    addr >>= 2;
    if (addr >= R_RTC_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "invalid write f2xx rtc register 0x%x\n",
          (unsigned int)addr << 2);
        return;
    }

    /* Special case for write protect state machine. */
    if (addr == R_RTC_WPR) {
        if (offset > 0) {
            return;
        }
        data &= 0xff;
        if ((s->wp_count == 0 && data == 0xca) ||
          (s->wp_count == 1 && data == 0x53)) {
            s->wp_count++;
        } else {
            s->wp_count = 0;
        }
        s->regs[addr] = data;
        return;
    }

    switch(size) {
    case 1:
        data = (s->regs[addr] & ~(0xff << (offset * 8))) | data << (offset * 8);
        break;
    case 2:
        data = (s->regs[addr] & ~(0xffff << (offset * 8))) | data << (offset * 8);
        break;
    case 4:
        break;
    default:
        abort();
    }
    if (addr >= R_RTC_BKPxR && addr <= R_RTC_BKPxR_INC_EXTRA_LAST) {
        s->regs[addr] = data;
        return;
    }
    /* Write protect */
    if (s->wp_count < 2 && addr != R_RTC_TAFCR && addr != R_RTC_ISR
            && addr != R_RTC_WPR) {
        qemu_log_mask(LOG_GUEST_ERROR, "f2xx rtc write reg 0x%x+%u without wp disable\n",
                      (unsigned int)addr << 2, offset);
        return;
    }
    switch(addr) {
    case R_RTC_TR:
    case R_RTC_DR:
        compute_new_target_offset = true;
        break;
    case R_RTC_CR:
        if ((data & R_RTC_CR_WUTE) != (s->regs[R_RTC_CR] & R_RTC_CR_WUTE)) {
            update_wut = true;
        }
        break;
    case R_RTC_ISR:
        if ((data & 1<<8) == 0 && (s->regs[R_RTC_ISR] & 1<<8) != 0) {
            DPRINTF("f2xx rtc isr lowered\n");
            f2xx_rtc_set_alarm_irq(s, 0, 0);
        }
        if ((data & 1<<9) == 0 && (s->regs[R_RTC_ISR] & 1<<9) != 0) {
            DPRINTF("f2xx rtc isr B lowered\n");
            f2xx_rtc_set_alarm_irq(s, 1, 0);
        }
        if ((data & 1<<10) == 0 && (s->regs[R_RTC_ISR] & 1<<10) != 0) {
            DPRINTF("f2xx rtc WUT isr lowered\n");
            f2xx_rtc_set_wut_irq(s, 0);
        }
        break;
    case R_RTC_PRER:
        /*
         * XXX currently updates upon next clock tick.  To do this properly we
         * would need to account for the time already elapsed, and then update
         * the timer for the remaining period.
         */
        break;
    case R_RTC_WUTR:
        update_wut = true;
        break;
    case R_RTC_ALRMAR:
    case R_RTC_ALRMBR:
        break;
    case R_RTC_TAFCR:
        if (data) {
            qemu_log_mask(LOG_UNIMP,
              "f2xx rtc unimplemented write TAFCR+%u size %u val %u\n",
              offset, size, (unsigned int)data);
        }
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "f2xx rtc unimplemented write 0x%x+%u size %u val 0x%x\n",
          (unsigned int)addr << 2, offset, size, (unsigned int)data);
    }
    s->regs[addr] = data;


    // Do we need to recompute the host to target offset?
    if (compute_new_target_offset) {
        struct tm target_tm;
        // Recompute ticks based on the modified contents of the TR and DR registers
        s->ticks = f2xx_rtc_get_current_target_time(s, &target_tm);
        // Update the host to target offset as well
        s->host_to_target_offset_us = f2xx_rtc_compute_host_to_target_offset(s,
        								f2xx_clock_period_ns(s), s->ticks);
    }

    // Do we need to update the timer for the wake-up-timer?
    if (update_wut) {
        if (s->regs[R_RTC_CR] & R_RTC_CR_WUTE) {
            int64_t elapsed = f2xx_wut_period_ns(s, s->regs[R_RTC_WUTR]);
            DPRINTF("%s: scheduling WUT to fire in %f ms\n", __func__, (float)elapsed/1000000.0);
            timer_mod(s->wu_timer, qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + elapsed);
        } else {
            DPRINTF("%s: Cancelling WUT\n", __func__);
            f2xx_rtc_set_wut_irq(s, 0);
            timer_del(s->wu_timer);
        }
    }
}


static bool
f2xx_alarm_match(f2xx_rtc *s, uint32_t alarm_reg)
{
    uint32_t tr = s->regs[R_RTC_TR];

    if ((alarm_reg & (1<<7)) == 0 && (tr & 0x7f) != (alarm_reg & 0x7f)) {
        /* Seconds match requested, but do not match. */
        return false;
    }
    if ((alarm_reg & (1<<15)) == 0 && (tr & 0x7f00) != (alarm_reg & 0x7f00)) {
        /* Minutes match requested, but do not match. */
        return false;
    }
    if ((alarm_reg & (1<<23)) == 0 && (tr & 0x7f0000) != (alarm_reg & 0x7f0000)) {
        /* Hours match requested, but do not match. */
        return false;
    }
    if ((alarm_reg & (1<<31)) == 0) { /* Day match. */
        uint32_t dr = s->regs[R_RTC_DR];
        if (alarm_reg & (1<<30)) { /* Day is week day. */
            if (((alarm_reg>>24) & 0xf) != ((dr>>13) & 0xf)) {
                return false;
            }
        } else { /* Day is day of month. */
            if (((alarm_reg>>24) & 0x3f) != (dr & 0x3f)) {
                return false;
            }
        }
    }
    return true;
}

static void
f2xx_alarm_check(f2xx_rtc *s, int unit)
{
    uint32_t cr = s->regs[R_RTC_CR];
    uint32_t isr = s->regs[R_RTC_ISR];

    if ((cr & 1<<(R_RTC_CR_ALRAE_BIT + unit)) == 0) {
        return; /* Not enabled. */
    }

    if ((isr & 1<<(R_RTC_CR_ALRAE_BIT + unit)) == 0) {
        if (f2xx_alarm_match(s, s->regs[R_RTC_ALRMAR + unit])) {
            isr |= 1<<(8 + unit);
            s->regs[R_RTC_ISR] = isr;
            DPRINTF("f2xx rtc alarm activated 0x%x 0x%x\n", isr, cr);
        }
    }
	f2xx_rtc_set_alarm_irq(s, unit, cr & 1<<(12 + unit) && isr & 1<<(8 + unit));
}

// This method updates the current time and date registers to match the current
// host time. While it advances the target time, it checks for alarms that need to fire.
static void
f2xx_update_current_date_and_time(void *arg)
{
    f2xx_rtc *s = arg;
    uint64_t period_ns = f2xx_clock_period_ns(s);

    struct tm new_target_tm;
    time_t new_target_ticks = f2xx_rtc_compute_target_time_from_host_time(s,
                                  period_ns, &new_target_tm);


    // Normally, we would advance the target ticks until we catch up to the host ticks and
    // check for an alarm at each tick. But, if the clocks got too far off (host or target
    // changed time), just jam in the new target time without checking alarms
    int delta = new_target_ticks - s->ticks;
    //DPRINTF("%s: advancing target by %d ticks\n", __func__, delta);
    if (delta < 0 || delta > 1000) {
        printf("DEBUG_STM32F2XX_RTC %s: detected %d mismatch between host and target ticks, "
              "jamming new host time into RTC without checking for alarms\n", __func__, delta);
        s->ticks = new_target_ticks;
        f2xx_rtc_set_time_and_date_registers(s, &new_target_tm);
    } else {
        while (s->ticks != new_target_ticks) {
            s->ticks += 1;
            gmtime_r(&s->ticks, &new_target_tm);
            f2xx_rtc_set_time_and_date_registers(s, &new_target_tm);

            f2xx_alarm_check(s, 0);
            f2xx_alarm_check(s, 1);
        }
    }

    // Reschedule tick timer to run one tick from now to check for alarms again
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_HOST) + period_ns);
}


// Refresh the TR/DR registers so they reflect the current host-derived time.
// This is side-effect free: unlike f2xx_update_current_date_and_time(), it does
// NOT advance s->ticks, check alarms, raise interrupts, or reschedule the timer.
// It is therefore safe to call from the MMIO read path. Alarm checking is left
// entirely to the periodic f2xx_timer() callback, so that a register read can
// never spuriously re-raise the alarm IRQ (which caused duplicate tick events
// when the firmware read RTC_TR from inside its own tick ISR).
static void
f2xx_rtc_refresh_time_and_date(f2xx_rtc *s)
{
    struct tm target_tm;
    uint64_t period_ns = f2xx_clock_period_ns(s);

    f2xx_rtc_compute_target_time_from_host_time(s, period_ns, &target_tm);
    f2xx_rtc_set_time_and_date_registers(s, &target_tm);
}


// This timer runs on every tick (usually second)
static void
f2xx_timer(void *arg)
{
    f2xx_rtc *s = arg;
    f2xx_update_current_date_and_time(s);
}


// This timer fires when the wake up time has expired
static void
f2xx_wu_timer(void *arg)
{
    f2xx_rtc *s = arg;

    DPRINTF("%s: fired\n", __func__);

    // Fire the interrupt?
    uint32_t cr = s->regs[R_RTC_CR];
    uint32_t isr = s->regs[R_RTC_ISR];

    // Make sure WUT is enabled
    if ( (cr & R_RTC_CR_WUTE) == 0 ) {
        return; /* Not enabled */
    }

    // If interrupt not already asserted, assert it
    if ( (isr & R_RTC_ISR_WUT) == 0 ) {
        isr |= R_RTC_ISR_WUT;
        s->regs[R_RTC_ISR] = isr;
        DPRINTF("f2xx wakeup timer ISR activated 0x%x 0x%x\n", isr, cr);
    }

    f2xx_rtc_set_wut_irq(s, (cr & R_RTC_CR_WUTIE) && (isr & R_RTC_ISR_WUT));

    // Reschedule again
    int64_t elapsed = f2xx_wut_period_ns(s, s->regs[R_RTC_WUTR]);
    timer_mod(s->wu_timer, qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + elapsed);
}


// External interface to set the value of an "extra" backup register. This can be used to
// communicate emulator specific settings to the target
void f2xx_rtc_set_extra_bkup_reg(void *opaque, uint32_t idx, uint32_t value)
{
    f2xx_rtc *s = (f2xx_rtc *)opaque;

     // Copy in the extra backup registers that were specified via properties
    assert(idx < STM32F2XX_RTC_NUM_EXTRA_BKUP_REG);
    s->regs[R_RTC_BKPxR_LAST + 1 + idx] = value;
}



static const MemoryRegionOps f2xx_rtc_ops = {
    .read = f2xx_rtc_read,
    .write = f2xx_rtc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    }
};

static void f2xx_rtc_reset(DeviceState *dev)
{
    f2xx_rtc *s = (f2xx_rtc *)dev;
    s->regs[R_RTC_CR] = 0;
}

static void
f2xx_rtc_realize(DeviceState *dev, Error **errp)
{
    f2xx_rtc *s = (f2xx_rtc *)dev;
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &f2xx_rtc_ops, s, "rtc", 0x03ff);
    sysbus_init_mmio(sbd, &s->iomem);

    sysbus_init_irq(sbd, &s->irq[0]);
    sysbus_init_irq(sbd, &s->irq[1]);
    sysbus_init_irq(sbd, &s->wut_irq);

    s->regs[R_RTC_ISR] = R_RTC_ISR_RESET;
    s->regs[R_RTC_PRER] = R_RTC_PRER_RESET;
    s->regs[R_RTC_WUTR] = R_RTC_WUTR_RESET;

    uint32_t period_ns = f2xx_clock_period_ns(s);
    DPRINTF("%s: period: %d ns\n", __func__, period_ns);

    // Init the time and date registers from the time on the host as the default
    s->host_to_target_offset_us = 0;
    struct tm now;
    qemu_get_timedate(&now, 0);

    // Set time and date registers from the now struct
    f2xx_rtc_set_time_and_date_registers(s, &now);

    // Compute current ticks and host to target offset
    s->ticks = mktimegm(&now);
    s->host_to_target_offset_us = f2xx_rtc_compute_host_to_target_offset(s,
                                        f2xx_clock_period_ns(s), s->ticks);

    s->timer = timer_new_ns(QEMU_CLOCK_HOST, f2xx_timer, s);
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_HOST) + period_ns);

    s->wu_timer = timer_new_ns(QEMU_CLOCK_REALTIME, f2xx_wu_timer, s);
}

static void
f2xx_rtc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = f2xx_rtc_realize;
    device_class_set_legacy_reset(dc, f2xx_rtc_reset);
}

static const TypeInfo
f2xx_rtc_info = {
    .name          = "f2xx_rtc",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(f2xx_rtc),
    .class_init    = f2xx_rtc_class_init,
};

static void
f2xx_rtc_register_types(void)
{
    type_register_static(&f2xx_rtc_info);
}

type_init(f2xx_rtc_register_types)
