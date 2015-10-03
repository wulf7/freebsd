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
#include "uinput.h"

#define	MAX_BUTTONS	3

/*
 * Open uinput device and setup 3button mouse with wheel
 * TODO: copypaste feature detection code from ums
 */
int
uinput_open_mouse(hid_device_p const d)
{
	int fd, i;
	struct uinput_user_dev uidev;

	fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK);
	if (fd < 0)
		return (-1);

#define	UIOCTL(req, val)	if(ioctl(fd, req, val) < 0) goto bail_out;
	UIOCTL(UI_SET_EVBIT, EV_KEY);
	for (i = 0; i < MAX_BUTTONS; i++)
		UIOCTL(UI_SET_KEYBIT, BTN_MOUSE + i);
	UIOCTL(UI_SET_EVBIT, EV_REL);
	UIOCTL(UI_SET_RELBIT, REL_X);
	UIOCTL(UI_SET_RELBIT, REL_Y);
	UIOCTL(UI_SET_RELBIT, REL_WHEEL);

	memset(&uidev, 0, sizeof(uidev));
	snprintf(uidev.name, UINPUT_MAX_NAME_SIZE, "Bluetooth Mouse (%s)",
		bt_ntoa(&d->bdaddr, NULL));
	uidev.id.bustype = BUS_BLUETOOTH;
	uidev.id.vendor  = 0x1; /* Dummy value */
	uidev.id.product = 0x1; /* Dummy value */
	uidev.id.version = 1;

	if (write(fd, &uidev, sizeof(uidev)) < 0)
		goto bail_out;

	if (ioctl(fd, UI_DEV_CREATE) >= 0)
		return (fd); /* SUCCESS */

bail_out:
	close(fd);
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
uinput_report_mouse(int fd, struct mouse_info *mi, int32_t obutt)
{
	int i, mask;

	if (mi->u.data.x != 0 || mi->u.data.y != 0) {
		if (uinput_write_event(fd, EV_REL, REL_X, mi->u.data.x) < 0)
			return (-1);
		if (uinput_write_event(fd, EV_REL, REL_Y, mi->u.data.y) < 0)
			return (-1);
	}

	if (mi->u.data.z != 0 &&
	    uinput_write_event(fd, EV_REL, REL_WHEEL, -mi->u.data.z) < 0)
		return (-1);

	for (i = 0; i < MAX_BUTTONS; i++) {
		switch (i) {
		case 1:
			mask = 1 << 2;
			break;
		case 2:
			mask = 1 << 1;
			break;
		default:
			mask = 1 << i;
		}
		if ((mi->u.data.buttons & mask) == (obutt & mask))
			continue;
		if (uinput_write_event(fd, EV_KEY, BTN_MOUSE + i,
		    (mi->u.data.buttons & mask) != 0) < 0)
			return (-1);
	}

	if (uinput_write_event(fd, EV_SYN, 0, 0) < 0)
		return (-1);

	return (0);
}
