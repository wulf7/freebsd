/*-
 * Copyright (c) 2015 Vladimir Kondratyev <wulf@cicgroup.ru>
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
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
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

#include <sys/param.h>
#include <sys/consio.h>
#include <sys/ioctl.h>
#include <sys/kbio.h>
#include <sys/queue.h>
#include <sys/sysctl.h>

#include <dev/evdev/input.h>
#include <dev/evdev/uinput.h>

#define L2CAP_SOCKET_CHECKED
#include <bluetooth.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <syslog.h>
#include <time.h>
#include <unistd.h>
#include <usbhid.h>

#include "bthid_config.h"
#include "bthidd.h"
#include "btuinput.h"

static int16_t const mbuttons[] = {
	BTN_LEFT,
	BTN_MIDDLE,
	BTN_RIGHT,
	BTN_SIDE,
	BTN_EXTRA,
	BTN_FORWARD,
	BTN_BACK,
	BTN_TASK
};

#define	NONE	KEY_RESERVED

static uint16_t const keycodes[256] = {
	/* 0x00 - 0x27 */
	NONE,	NONE,	NONE,	NONE,	KEY_A,	KEY_B,	KEY_C,	KEY_D,
	KEY_E,	KEY_F,	KEY_G,	KEY_H,	KEY_I,	KEY_J,	KEY_K,	KEY_L,
	KEY_M,	KEY_N,	KEY_O,	KEY_P,	KEY_Q,	KEY_R,	KEY_S,	KEY_T,
	KEY_U,	KEY_V,	KEY_W,	KEY_X,	KEY_Y,	KEY_Z,	KEY_1,	KEY_2,
	KEY_3,	KEY_4,	KEY_5,	KEY_6,	KEY_7,	KEY_8,	KEY_9,	KEY_0,
	/* 0x28 - 0x3f */
	KEY_ENTER,	KEY_ESC,	KEY_BACKSPACE,	KEY_TAB,
	KEY_SPACE,	KEY_MINUS,	KEY_EQUAL,	KEY_LEFTBRACE,
	KEY_RIGHTBRACE,	KEY_BACKSLASH,	KEY_BACKSLASH,	KEY_SEMICOLON,
	KEY_APOSTROPHE,	KEY_GRAVE,	KEY_COMMA,	KEY_DOT,
	KEY_SLASH,	KEY_CAPSLOCK,	KEY_F1,		KEY_F2,
	KEY_F3,		KEY_F4,		KEY_F5,		KEY_F6,
	/* 0x40 - 0x5f */
	KEY_F7,		KEY_F8,		KEY_F9,		KEY_F10,
	KEY_F11,	KEY_F12,	KEY_SYSRQ,	KEY_SCROLLLOCK,
	KEY_PAUSE,	KEY_INSERT,	KEY_HOME,	KEY_PAGEUP,
	KEY_DELETE,	KEY_END,	KEY_PAGEDOWN,	KEY_RIGHT,
	KEY_LEFT,	KEY_DOWN,	KEY_UP,		KEY_NUMLOCK,
	KEY_SLASH,	KEY_KPASTERISK,	KEY_KPMINUS,	KEY_KPPLUS,
	KEY_KPENTER,	KEY_KP1,	KEY_KP2,	KEY_KP3,
	KEY_KP4,	KEY_KP5,	KEY_KP6,	KEY_KP7,
	/* 0x60 - 0x7f */
	KEY_KP8,	KEY_KP9,	KEY_KP0,	KEY_KPDOT,
	KEY_102ND,	KEY_COMPOSE,	KEY_POWER,	KEY_KPEQUAL,
	KEY_F13,	KEY_F14,	KEY_F15,	KEY_F16,
	KEY_F17,	KEY_F18,	KEY_F19,	KEY_F20,
	KEY_F21,	KEY_F22,	KEY_F23,	KEY_F24,
	KEY_OPEN,	KEY_HELP,	KEY_PROPS,	KEY_FRONT,
	KEY_STOP,	KEY_AGAIN,	KEY_UNDO,	KEY_CUT,
	KEY_COPY,	KEY_PASTE,	KEY_FIND,	KEY_MUTE,
	/* 0x80 - 0x9f */
	KEY_VOLUMEUP,	KEY_VOLUMEDOWN,	NONE,		NONE,
	NONE,		KEY_KPCOMMA,	NONE,		KEY_RO,
	KEY_KATAKANAHIRAGANA,	KEY_YEN,KEY_HENKAN,	KEY_MUHENKAN,
	KEY_KPJPCOMMA,	NONE,		NONE,		NONE,
	KEY_HANGEUL,	KEY_HANJA,	KEY_KATAKANA,	KEY_HIRAGANA,
	KEY_ZENKAKUHANKAKU,	NONE,	NONE,		NONE,
	NONE,		NONE,		NONE,		NONE,
	NONE,		NONE,		NONE,		NONE,
	/* 0xa0 - 0xbf */
	NONE,		NONE,		NONE,		NONE,
	NONE,		NONE,		NONE,		NONE,
	NONE,		NONE,		NONE,		NONE,
	NONE,		NONE,		NONE,		NONE,
	NONE,		NONE,		NONE,		NONE,
	NONE,		NONE,		NONE,		NONE,
	NONE,		NONE,		NONE,		NONE,
	NONE,		NONE,		NONE,		NONE,
	/* 0xc0 - 0xdf */
	NONE,		NONE,           NONE,		NONE,
	NONE,		NONE,           NONE,		NONE,
	NONE,		NONE,           NONE,		NONE,
	NONE,		NONE,           NONE,		NONE,
	NONE,		NONE,           NONE,		NONE,
	NONE,		NONE,           NONE,		NONE,
	NONE,		NONE,           NONE,		NONE,
	NONE,		NONE,           NONE,		NONE,
	/* 0xe0 - 0xff */
	KEY_LEFTCTRL,	KEY_LEFTSHIFT,	KEY_LEFTALT,	KEY_LEFTMETA,
	KEY_RIGHTCTRL,	KEY_RIGHTSHIFT,	KEY_RIGHTALT,	KEY_RIGHTMETA,
	KEY_PLAYPAUSE,	KEY_STOPCD,	KEY_PREVIOUSSONG,KEY_NEXTSONG,
	KEY_EJECTCD,	KEY_VOLUMEUP,	KEY_VOLUMEDOWN, KEY_MUTE,
	KEY_WWW,	KEY_BACK,	KEY_FORWARD,	KEY_STOP,
	KEY_FIND,	KEY_SCROLLUP,	KEY_SCROLLDOWN,	KEY_EDIT,
	KEY_SLEEP,	KEY_COFFEE,	KEY_REFRESH,	KEY_CALC,
	NONE,		NONE,		NONE,		NONE,
};

/*
 * Setup uinput device as 8button mouse with wheel
 * TODO: copypaste feature detection code from ums
 */
int
uinput_create_mouse(bthid_session_p const s)
{
	size_t			i, len;
	struct uinput_setup	uisetup;
	struct sockaddr_l2cap	local;
	socklen_t		sclen;
	char			devname[HCI_DEVNAME_SIZE];
	const char		*phys;

	/* Find bluetooth device name or take local bdaddr */
	sclen = sizeof(local);
	if (getsockname(s->ctrl, (struct sockaddr *) &local, &sclen) == 0 &&
	    bt_devname(devname, &local.l2cap_bdaddr) != 0) {
		len = strnlen(devname, HCI_DEVNAME_SIZE);
		/* cut "hci" ending */
		if (len > 4 && strncmp(devname + len - 3, "hci", 3) == 0)
			devname[len - 3] = 0;
		phys = devname;
	} else
		phys = bt_ntoa(&local.l2cap_bdaddr, NULL);

	/* Set evdev device name and bus/vendor information */
	memset(&uisetup, 0, sizeof(uisetup));
	snprintf(uisetup.name, UINPUT_MAX_NAME_SIZE, "Bluetooth Mouse (%s)",
	    bt_ntoa(&s->bdaddr, NULL));
	uisetup.id.bustype = BUS_BLUETOOTH;
	uisetup.id.vendor  = 0x0000; /* Dummy value */
	uisetup.id.product = 0x0000; /* Dummy value */
	uisetup.id.version = 0x0000; /* Dummy value */

	/* Advertise events and axes */
	if (ioctl(s->uinput, UI_SET_EVBIT, EV_KEY) < 0 ||
	    ioctl(s->uinput, UI_SET_EVBIT, EV_REL) < 0 ||
	    ioctl(s->uinput, UI_SET_EVBIT, EV_SYN) < 0 ||
	    ioctl(s->uinput, UI_SET_RELBIT, REL_X) < 0 ||
	    ioctl(s->uinput, UI_SET_RELBIT, REL_Y) < 0 ||
	    ioctl(s->uinput, UI_SET_RELBIT, REL_WHEEL) < 0 ||
	    ioctl(s->uinput, UI_SET_PHYS, phys) < 0 ||
	    ioctl(s->uinput, UI_DEV_SETUP, &uisetup) < 0)
		goto bail_out;

	/* Advertise mouse buttons */
	for (i = 0; i < nitems(mbuttons); i++)
		if (ioctl(s->uinput, UI_SET_KEYBIT, mbuttons[i]) < 0)
			goto bail_out;

	if (ioctl(s->uinput, UI_DEV_CREATE) >= 0)
		return (0); /* SUCCESS */

bail_out:
	return (-1);
}

/*
 * Setup uinput keyboard
 */
int
uinput_create_keyboard(bthid_session_p const s)
{
	size_t			i, len;
	struct uinput_setup	uisetup;
	struct sockaddr_l2cap	local;
	socklen_t		sclen;
	char			devname[HCI_DEVNAME_SIZE];
	const char		*phys;

	/* Find bluetooth device name or take local bdaddr */
	sclen = sizeof(local);
	if (getsockname(s->ctrl, (struct sockaddr *) &local, &sclen) == 0 &&
	    bt_devname(devname, &local.l2cap_bdaddr) != 0) {
		len = strnlen(devname, HCI_DEVNAME_SIZE);
		/* cut "hci" ending */
		if (len > 4 && strncmp(devname + len - 3, "hci", 3) == 0)
			devname[len - 3] = 0;
		phys = devname;
	} else
		phys = bt_ntoa(&local.l2cap_bdaddr, NULL);

	/* Set evdev device name and bus/vendor information */
	memset(&uisetup, 0, sizeof(uisetup));
	snprintf(uisetup.name, UINPUT_MAX_NAME_SIZE, "Bluetooth Keyboard (%s)",
	    bt_ntoa(&s->bdaddr, NULL));
	uisetup.id.bustype = BUS_BLUETOOTH;
	uisetup.id.vendor  = 0x0000; /* Dummy value */
	uisetup.id.product = 0x0000; /* Dummy value */
	uisetup.id.version = 0x0000; /* Dummy value */

	/* Advertise key events */
	if (ioctl(s->uinput, UI_SET_EVBIT, EV_KEY) < 0 ||
	    ioctl(s->uinput, UI_SET_EVBIT, EV_LED) < 0 ||
	    ioctl(s->uinput, UI_SET_EVBIT, EV_SYN) < 0 ||
	    ioctl(s->uinput, UI_SET_EVBIT, EV_REP) < 0 ||
	    ioctl(s->uinput, UI_SET_LEDBIT, LED_CAPSL) < 0 ||
	    ioctl(s->uinput, UI_SET_LEDBIT, LED_NUML) < 0 ||
	    ioctl(s->uinput, UI_SET_LEDBIT, LED_SCROLLL) < 0 ||
	    ioctl(s->uinput, UI_SET_PHYS, phys) < 0 ||
	    ioctl(s->uinput, UI_DEV_SETUP, &uisetup) < 0)
		goto bail_out;

	/* Advertise keycodes */
	for (i = 0; i < nitems(keycodes); i++)
		if (ioctl(s->uinput, UI_SET_KEYBIT, keycodes[i]) < 0)
			goto bail_out;

	if (ioctl(s->uinput, UI_DEV_CREATE) >= 0)
		return (0); /* SUCCESS */

bail_out:
	return (-1);
}

/* from sys/dev/evdev/evdev.h */
#define	EVDEV_RCPT_HW_MOUSE	(1<<2)
#define	EVDEV_RCPT_HW_KBD	(1<<3)

static int
uinput_get_rcpt_mask(void)
{
	static struct timespec last = { 0, 0 };
	struct timespec now;
	static int mask = 0xF;
	size_t len;
	time_t elapsed;

	if (clock_gettime(CLOCK_UPTIME_FAST, &now) == -1)
		return mask;

	elapsed = now.tv_sec - last.tv_sec;
	if (now.tv_nsec < last.tv_nsec)
		elapsed--;

#define	MASK_POLL_INTERVAL	5 /* seconds */
	if (elapsed >= MASK_POLL_INTERVAL) {
		len = sizeof(mask);
		sysctlbyname("kern.evdev.rcpt_mask", &mask, &len, NULL, 0);
		last = now;
	}
	return mask;
}

static int
uinput_write_event(int fd, uint16_t type, uint16_t code, int32_t value)
{
	struct input_event ie;

	memset(&ie, 0, sizeof(ie));
	ie.type = type;
	ie.code = code;
	ie.value = value;
	return (write(fd, &ie, sizeof(ie)));
}

int
uinput_report_mouse(bthid_session_p s, struct mouse_info *mi)
{
	size_t i;
	int rcpt_mask, mask, fd;

	rcpt_mask = uinput_get_rcpt_mask();
	if (!(rcpt_mask & EVDEV_RCPT_HW_MOUSE))
		return (0);

	fd = s->uinput;
	if (mi->u.data.x != 0 || mi->u.data.y != 0) {
		if (uinput_write_event(fd, EV_REL, REL_X, mi->u.data.x) < 0)
			return (-1);
		if (uinput_write_event(fd, EV_REL, REL_Y, mi->u.data.y) < 0)
			return (-1);
	}

	if (mi->u.data.z != 0 &&
	    uinput_write_event(fd, EV_REL, REL_WHEEL, -mi->u.data.z) < 0)
		return (-1);

	for (i = 0; i < nitems(mbuttons); i++) {
		mask = 1 << i;
		if ((mi->u.data.buttons & mask) == (s->obutt & mask))
			continue;
		if (uinput_write_event(fd, EV_KEY, mbuttons[i],
		    (mi->u.data.buttons & mask) != 0) < 0)
			return (-1);
	}
	s->obutt = mi->u.data.buttons;

	if (uinput_write_event(fd, EV_SYN, SYN_REPORT, 0) < 0)
		return (-1);

	return (0);
}

/*
 * Translate given keymap and write keyscodes
 */
void
uinput_kbd_write(bitstr_t *m, int32_t fb, int32_t make, int32_t fd)
{
	size_t i;
	uint16_t n;
	int rcpt_mask;

	rcpt_mask = uinput_get_rcpt_mask();
	if (!(rcpt_mask & EVDEV_RCPT_HW_KBD))
		return;

	for (i = fb; i < nitems(keycodes); i++) {
		if (bit_test(m, i)) {
			n = keycodes[i];
			if (n == NONE)
				continue;
			uinput_write_event(fd, EV_KEY, n, make);
			uinput_write_event(fd, EV_SYN, SYN_REPORT, 0);
		}
	}
}

int
uinput_report_leds(int fd, int state, int mask)
{

	if (mask & LED_CAP &&
	    uinput_write_event(fd, EV_LED, LED_CAPSL,
	        state & LED_CAP ? 1 : 0) < 0)
		return (-1);

	if (mask & LED_NUM &&
	    uinput_write_event(fd, EV_LED, LED_NUML,
	        state & LED_NUM ? 1 : 0) < 0)
		return (-1);

	if (mask & LED_NUM &&
	    uinput_write_event(fd, EV_LED, LED_SCROLLL,
	        state & LED_SCR ? 1 : 0) < 0)
		return (-1);

	if (mask & (LED_CAP | LED_NUM | LED_SCR) &&
	    uinput_write_event(fd, EV_SYN, SYN_REPORT, 0) < 0)
		return (-1);

	return (0);
}
