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
 * QEMU Sharp LS013B7DH01 Memory LCD device model.
 * Ported from QEMU 2.5.0-pebble8 ls013b7dh01.c to QEMU 10.x APIs.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "ui/console.h"
#include "ui/pixel_ops.h"
#include "hw/ssi/ssi.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "qapi/error.h"

#define NUM_ROWS 168
#define NUM_COLS 144
#define NUM_COL_BYTES (NUM_COLS / 8)

typedef enum {
    SM_LCD_COMMAND,
    SM_LCD_LINENO,
    SM_LCD_DATA,
    SM_LCD_TRAILER
} SmLcdXferState;

typedef struct {
    SSIPeripheral parent_obj;
    QemuConsole *con;
    bool redraw;
    uint8_t framebuffer[NUM_ROWS * NUM_COL_BYTES];
    int fbindex;
    SmLcdXferState state;

    bool   backlight_enabled;
    float  brightness;

    bool   vibrate_on;
    int    vibrate_offset;

    bool power_on;

    /* Tintin display was installed 'upside-down'.
     * Use the "rotate_display" property to flip it.
     */
    bool rotate_display;
} SmLcdState;

#define TYPE_SM_LCD "sm-lcd"
#define SM_LCD(obj) OBJECT_CHECK(SmLcdState, (obj), TYPE_SM_LCD)

static uint8_t
bitswap(uint8_t val)
{
    return ((val * 0x0802LU & 0x22110LU) |
            (val * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16;
}

static uint32_t
sm_lcd_transfer(SSIPeripheral *dev, uint32_t data)
{
    SmLcdState *s = SM_LCD(dev);
    /* QEMU's SPI infrastructure is implicitly MSB-first */
    data = bitswap(data);

    switch (s->state) {
    case SM_LCD_COMMAND:
        data &= 0xfd; /* Mask VCOM bit */
        switch (data) {
        case 0x01: /* Write Line */
            s->state = SM_LCD_LINENO;
            break;
        case 0x04: /* Clear Screen */
            memset(s->framebuffer, 0, sizeof(s->framebuffer));
            s->redraw = true;
            break;
        case 0x00: /* Toggle VCOM */
            break;
        default:
            /* Simulate confused display controller */
            memset(s->framebuffer, 0x55, sizeof(s->framebuffer));
            s->redraw = true;
            break;
        }
        break;
    case SM_LCD_LINENO:
        if (data == 0) {
            s->state = SM_LCD_COMMAND;
        } else {
            s->fbindex = (data - 1) * NUM_COL_BYTES;
            s->state = SM_LCD_DATA;
        }
        break;
    case SM_LCD_DATA:
        s->framebuffer[s->fbindex++] = data;
        if (s->fbindex % NUM_COL_BYTES == 0) {
            s->state = SM_LCD_TRAILER;
        }
        break;
    case SM_LCD_TRAILER:
        if (data != 0) {
            qemu_log_mask(LOG_GUEST_ERROR,
              "ls013 memory lcd received non-zero data in TRAILER\n");
        }
        s->state = SM_LCD_LINENO;
        s->redraw = true;
        break;
    }
    return 0;
}

static void sm_lcd_update_display(void *arg)
{
    SmLcdState *s = arg;

    uint8_t *d;
    uint32_t colour_on, colour_off, colour;
    int x, y, bpp;

    DisplaySurface *surface = qemu_console_surface(s->con);

    /* The browser bridge polls gfx_update on a timer, which reaches here in
     * states the native display loop never does. Bail out instead of
     * writing through NULL. */
    if (!surface || !surface_data(surface)) {
        return;
    }
    bpp = surface_bits_per_pixel(surface);
    d = surface_data(surface);

    /* If vibrate is on, jiggle the display */
    if (s->vibrate_on) {
        if (s->vibrate_offset == 0) {
            s->vibrate_offset = 2;
        }
        int bytes_per_pixel;
        switch (bpp) {
        case 8:
            bytes_per_pixel = 1;
            break;
        case 15:
        case 16:
            bytes_per_pixel = 2;
            break;
        case 32:
            bytes_per_pixel = 4;
            break;
        default:
            abort();
        }
        int total_bytes = NUM_ROWS * NUM_COLS * bytes_per_pixel
                        - abs(s->vibrate_offset) * bytes_per_pixel;
        if (s->vibrate_offset > 0) {
            memmove(d, d + s->vibrate_offset * bytes_per_pixel, total_bytes);
        } else {
            memmove(d - s->vibrate_offset * bytes_per_pixel, d, total_bytes);
        }
        s->vibrate_offset *= -1;
        dpy_gfx_update(s->con, 0, 0, NUM_COLS, NUM_ROWS);
        return;
    }

    if (!s->redraw) {
        return;
    }

    /* Adjust white level for brightness */
    float brightness = s->backlight_enabled ? s->brightness : 0.0;
    int max_val = 170 + (255 - 170) * brightness;

    switch (bpp) {
    case 8:
        colour_on = rgb_to_pixel8(max_val, max_val, max_val);
        colour_off = rgb_to_pixel8(0x00, 0x00, 0x00);
        break;
    case 15:
        colour_on = rgb_to_pixel15(max_val, max_val, max_val);
        colour_off = rgb_to_pixel15(0x00, 0x00, 0x00);
        break;
    case 16:
        colour_on = rgb_to_pixel16(max_val, max_val, max_val);
        colour_off = rgb_to_pixel16(0x00, 0x00, 0x00);
        break;
    case 32:
        colour_on = rgb_to_pixel32(max_val, max_val, max_val);
        colour_off = rgb_to_pixel32(0x00, 0x00, 0x00);
        break;
    default:
        return;
    }

    for (y = 0; y < NUM_ROWS; y++) {
        for (x = 0; x < NUM_COLS; x++) {
            int xr = (s->rotate_display) ? NUM_COLS - 1 - x : x;
            int yr = (s->rotate_display) ? NUM_ROWS - 1 - y : y;
            bool on = s->framebuffer[yr * NUM_COL_BYTES + xr / 8] &
                      (1 << (xr % 8));
            colour = on ? colour_on : colour_off;
            switch (bpp) {
            case 8:
                *((uint8_t *)d) = colour;
                d++;
                break;
            case 15:
            case 16:
                *((uint16_t *)d) = colour;
                d += 2;
                break;
            case 32:
                *((uint32_t *)d) = colour;
                d += 4;
                break;
            default:
                break;
            }
        }
    }

    dpy_gfx_update(s->con, 0, 0, NUM_COLS, NUM_ROWS);
    s->redraw = false;
}

static void sm_lcd_invalidate_display(void *arg)
{
    SmLcdState *s = arg;
    s->redraw = true;
}

static void sm_lcd_backlight_enable_cb(void *opaque, int n, int level)
{
    SmLcdState *s = (SmLcdState *)opaque;
    assert(n == 0);

    bool enable = (level != 0);
    if (s->backlight_enabled != enable) {
        s->backlight_enabled = enable;
        s->redraw = true;
    }
}

static void sm_lcd_set_backlight_level_cb(void *opaque, int n, int level)
{
    SmLcdState *s = (SmLcdState *)opaque;
    assert(n == 0);

    float bright_f = (float)level / 255;
    float new_setting = MIN(1.0, bright_f * 4);
    if (new_setting != s->brightness) {
        s->brightness = MIN(1.0, bright_f * 4);
        if (s->backlight_enabled) {
            s->redraw = true;
        }
    }
}

static void sm_lcd_vibe_ctl(void *opaque, int n, int level)
{
    SmLcdState *s = (SmLcdState *)opaque;
    assert(n == 0);

    s->vibrate_on = (level != 0);
}

static void sm_lcd_power_ctl(void *opaque, int n, int level)
{
    SmLcdState *s = (SmLcdState *)opaque;
    assert(n == 0);

    if (!level && s->power_on) {
        memset(&s->framebuffer, 0, sizeof(s->framebuffer));
        s->redraw = true;
        s->power_on = false;
    }
    s->power_on = !!level;
}

static void sm_lcd_reset(DeviceState *dev)
{
    SmLcdState *s = SM_LCD(dev);
    memset(&s->framebuffer, 0, sizeof(s->framebuffer));
    s->redraw = true;
}

static const GraphicHwOps sm_lcd_ops = {
    .gfx_update = sm_lcd_update_display,
    .invalidate = sm_lcd_invalidate_display,
};

static void sm_lcd_realize(SSIPeripheral *dev, Error **errp)
{
    SmLcdState *s = SM_LCD(dev);

    s->brightness = 0.0;

    s->con = graphic_console_init(DEVICE(dev), 0, &sm_lcd_ops, s);
    qemu_console_resize(s->con, NUM_COLS, NUM_ROWS);

    qdev_init_gpio_in_named(DEVICE(dev), sm_lcd_backlight_enable_cb,
                            "backlight_enable", 1);
    qdev_init_gpio_in_named(DEVICE(dev), sm_lcd_set_backlight_level_cb,
                            "backlight_level", 1);
    qdev_init_gpio_in_named(DEVICE(dev), sm_lcd_vibe_ctl,
                            "vibe_ctl", 1);
    qdev_init_gpio_in_named(DEVICE(dev), sm_lcd_power_ctl,
                            "power_ctl", 1);
}

static const Property sm_lcd_properties[] = {
    DEFINE_PROP_BOOL("rotate_display", SmLcdState, rotate_display, true),
};

static void sm_lcd_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);

    device_class_set_props(dc, sm_lcd_properties);
    k->realize = sm_lcd_realize;
    k->transfer = sm_lcd_transfer;
    k->cs_polarity = SSI_CS_LOW;
    device_class_set_legacy_reset(dc, sm_lcd_reset);
}

static const TypeInfo sm_lcd_info = {
    .name          = TYPE_SM_LCD,
    .parent        = TYPE_SSI_PERIPHERAL,
    .instance_size = sizeof(SmLcdState),
    .class_init    = sm_lcd_class_init,
};

static void sm_lcd_register(void)
{
    type_register_static(&sm_lcd_info);
}

type_init(sm_lcd_register);
