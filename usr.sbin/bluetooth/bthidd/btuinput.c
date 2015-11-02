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
#include <sys/queue.h>

#include <dev/evdev/input.h>
#include <dev/evdev/uinput.h>

#define L2CAP_SOCKET_CHECKED
#include <bluetooth.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <syslog.h>
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

/*
 * Setup uinput device as 8button mouse with wheel
 * TODO: copypaste feature detection code from ums
 */
int
uinput_create_mouse(bthid_session_p const s)
{
	size_t			i, len;
	struct uinput_user_dev	uidev;
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

	/* Advertise events and axes */
	if (ioctl(s->uinput, UI_SET_EVBIT, EV_KEY) < 0 ||
	    ioctl(s->uinput, UI_SET_EVBIT, EV_REL) < 0 ||
	    ioctl(s->uinput, UI_SET_RELBIT, REL_X) < 0 ||
	    ioctl(s->uinput, UI_SET_RELBIT, REL_Y) < 0 ||
	    ioctl(s->uinput, UI_SET_RELBIT, REL_WHEEL) < 0 ||
	    ioctl(s->uinput, UI_SET_PHYS, phys) < 0)
		goto bail_out;

	/* Advertise mouse buttons */
	for (i = 0; i < nitems(mbuttons); i++)
		if (ioctl(s->uinput, UI_SET_KEYBIT, mbuttons[i]) < 0)
			goto bail_out;

	/* Set evdev device name and bus/vendor information */
	memset(&uidev, 0, sizeof(uidev));
	snprintf(uidev.name, UINPUT_MAX_NAME_SIZE, "Bluetooth Mouse (%s)",
		bt_ntoa(&s->bdaddr, NULL));
	uidev.id.bustype = BUS_BLUETOOTH;
	uidev.id.vendor  = 0x0000; /* Dummy value */
	uidev.id.product = 0x0000; /* Dummy value */
	uidev.id.version = 0x0000; /* Dummy value */
	if (write(s->uinput, &uidev, sizeof(uidev)) < 0)
		goto bail_out;

	if (ioctl(s->uinput, UI_DEV_CREATE) >= 0)
		return (0); /* SUCCESS */

bail_out:
	return (-1);
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
	int mask, fd;

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
