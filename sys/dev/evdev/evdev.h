/*-
 * Copyright (c) 2014 Jakub Wojciech Klama <jceel@FreeBSD.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#ifndef	_DEV_EVDEV_EVDEV_H
#define	_DEV_EVDEV_EVDEV_H

#include <sys/types.h>
#include <sys/kbio.h>
#include <dev/evdev/input.h>
#include <dev/kbd/kbdreg.h>

#define	NAMELEN		80

struct evdev_dev;

typedef int (evdev_open_t)(struct evdev_dev *, void *);
typedef void (evdev_close_t)(struct evdev_dev *, void *);
typedef void (evdev_event_t)(struct evdev_dev *, void *, uint16_t,
    uint16_t, int32_t);
typedef void (evdev_keycode_t)(struct evdev_dev *, void *,
    struct input_keymap_entry *);

#define	ABS_MT_FIRST	ABS_MT_TOUCH_MAJOR
#define	ABS_MT_LAST	ABS_MT_TOOL_Y
#define	ABS_IS_MT(x)	((x) >= ABS_MT_FIRST && (x) <= ABS_MT_LAST)
#define	ABS_MT_INDEX(x)	((x) - ABS_MT_FIRST)
#define	MT_CNT		(ABS_MT_INDEX(ABS_MT_LAST) + 1)
/* Multitouch protocol type A */
#define	MAX_MT_REPORTS	5
/* Multitouch protocol type B interface */
#define	MAX_MT_SLOTS	16

#define	EVDEV_FLAG_SOFTREPEAT	0x00	/* use evdev to repeat keys */
#define	EVDEV_FLAG_MT_STCOMPAT	0x01	/* autogenerate ST-compatible events
					 * for MT protocol type B reports */
#define	EVDEV_FLAG_MAX		0x1F
#define	EVDEV_FLAG_CNT		(EVDEV_FLAG_MAX + 1)

#define	PS2_KEYBOARD_VENDOR		1
#define	PS2_KEYBOARD_PRODUCT		1
#define	PS2_MOUSE_VENDOR		2
#define	PS2_MOUSE_GENERIC_PRODUCT	1

struct evdev_methods
{
	evdev_open_t		*ev_open;
	evdev_close_t		*ev_close;
	evdev_event_t		*ev_event;
	evdev_keycode_t		*ev_get_keycode;
	evdev_keycode_t		*ev_set_keycode;
};

/* Input device interface: */
struct evdev_dev *evdev_alloc(void);
void evdev_free(struct evdev_dev *);
void evdev_set_name(struct evdev_dev *, const char *);
void evdev_set_id(struct evdev_dev *, const struct input_id *);
void evdev_set_phys(struct evdev_dev *, const char *);
void evdev_set_serial(struct evdev_dev *, const char *);
void evdev_set_methods(struct evdev_dev *, void *, struct evdev_methods *);
int evdev_register(device_t, struct evdev_dev *);
int evdev_unregister(device_t, struct evdev_dev *);
int evdev_push_event(struct evdev_dev *, uint16_t, uint16_t, int32_t);
int evdev_sync(struct evdev_dev *);
int evdev_mt_sync(struct evdev_dev *);
int evdev_support_prop(struct evdev_dev *, uint16_t);
int evdev_support_event(struct evdev_dev *, uint16_t);
int evdev_support_key(struct evdev_dev *, uint16_t);
int evdev_support_rel(struct evdev_dev *, uint16_t);
int evdev_support_abs(struct evdev_dev *, uint16_t, struct input_absinfo *);
int evdev_support_msc(struct evdev_dev *, uint16_t);
int evdev_support_led(struct evdev_dev *, uint16_t);
int evdev_support_snd(struct evdev_dev *, uint16_t);
int evdev_support_sw(struct evdev_dev *, uint16_t);
void evdev_set_repeat_params(struct evdev_dev *, uint16_t, int);
int evdev_set_report_size(struct evdev_dev *, size_t);
int evdev_set_flag(struct evdev_dev *, uint16_t);

/* Multitouch related functions: */
int32_t evdev_get_mt_slot_by_tracking_id(struct evdev_dev *, int32_t);
void evdev_support_nfingers(struct evdev_dev *, int32_t);
void evdev_support_mt_compat(struct evdev_dev *);
void evdev_push_nfingers(struct evdev_dev *, int32_t);
void evdev_push_mt_compat(struct evdev_dev *);

/* Utility functions: */
uint16_t evdev_hid2key(int);
void evdev_support_all_known_keys(struct evdev_dev *);
uint16_t evdev_scancode2key(int *, int);
void evdev_push_mouse_btn(struct evdev_dev *, int);
void evdev_push_leds(struct evdev_dev *, int);
void evdev_push_repeats(struct evdev_dev *, keyboard_t *);
evdev_event_t evdev_ev_kbd_event;

#endif	/* _DEV_EVDEV_EVDEV_H */
