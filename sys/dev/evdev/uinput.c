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

#include <sys/types.h>
#include <sys/systm.h>
#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/conf.h>
#include <sys/uio.h>
#include <sys/proc.h>
#include <sys/poll.h>
#include <sys/selinfo.h>
#include <sys/malloc.h>

#include <dev/evdev/input.h>
#include <dev/evdev/uinput.h>
#include <dev/evdev/evdev.h>

#define	DEBUG
#ifdef DEBUG
#define	debugf(fmt, args...)	printf("evdev: " fmt "\n", ##args);
#else
#define	debugf(fmt, args...)
#endif

static evdev_event_t	uinput_ev_event;

static d_open_t		uinput_open;
static d_read_t		uinput_read;
static d_write_t	uinput_write;
static d_ioctl_t	uinput_ioctl;
static d_poll_t		uinput_poll;
static void uinput_dtor(void *);

static int uinput_setup_provider(struct evdev_dev *, struct uinput_user_dev *);
static int uinput_cdev_create(void);

static struct cdevsw uinput_cdevsw = {
	.d_version = D_VERSION,
	.d_open = uinput_open,
	.d_read = uinput_read,
	.d_write = uinput_write,
	.d_ioctl = uinput_ioctl,
	.d_poll = uinput_poll,
	.d_name = "uinput",
};

static struct evdev_methods uinput_ev_methods = {
	.ev_open = NULL,
	.ev_close = NULL,
	.ev_event = uinput_ev_event,
};

struct uinput_cdev_state
{
	bool			ucs_connected;
	struct evdev_dev *	ucs_evdev;
	struct mtx		ucs_mtx;
};

static void
uinput_ev_event(struct evdev_dev *evdev, void *softc, uint16_t type,
    uint16_t code, int32_t value)
{
	switch (type) {
	case EV_LED:
		evdev_push_event(evdev, EV_LED, code, value);
		break;

	case EV_REP:
		if (code == REP_DELAY) {
			evdev_set_repeat_params(evdev, REP_DELAY, value);
			evdev_push_event(evdev, EV_REP, REP_DELAY, value);
		} else if (code == REP_PERIOD) {
			evdev_set_repeat_params(evdev, REP_PERIOD, value);
			evdev_push_event(evdev, EV_REP, REP_PERIOD, value);
		}
		break;

	default:
		break;
	}
}

static int
uinput_open(struct cdev *dev, int oflags, int devtype, struct thread *td)
{
	struct uinput_cdev_state *state;

	state = malloc(sizeof(struct uinput_cdev_state), M_EVDEV, M_WAITOK | M_ZERO);
	state->ucs_evdev = evdev_alloc();

	devfs_set_cdevpriv(state, uinput_dtor);
	return (0);
}

static void
uinput_dtor(void *data)
{
	struct uinput_cdev_state *state = (struct uinput_cdev_state *)data;

	if (state->ucs_connected)
		evdev_unregister(NULL, state->ucs_evdev);

	evdev_free(state->ucs_evdev);
	free(data, M_EVDEV);
}

static int
uinput_read(struct cdev *dev, struct uio *uio, int ioflag)
{
	struct uinput_cdev_state *state;
	struct evdev_dev *evdev;
	int ret = 0;

	debugf("uinput: read %ld bytes by thread %d", uio->uio_resid,
	    uio->uio_td->td_tid);

	ret = devfs_get_cdevpriv((void **)&state);
	if (ret != 0)
		return (ret);

	evdev = state->ucs_evdev;

	if (uio->uio_resid % sizeof(struct input_event) != 0) {
		debugf("read size not multiple of struct input_event size");
		return (EINVAL);
	}

	return (0);
}

static int
uinput_write(struct cdev *dev, struct uio *uio, int ioflag)
{
	struct uinput_cdev_state *state;
	struct uinput_user_dev userdev;
	struct input_event event;
	int ret = 0;
	
	debugf("uinput: write %ld bytes by thread %d", uio->uio_resid,
	    uio->uio_td->td_tid);

	ret = devfs_get_cdevpriv((void **)&state);
	if (ret != 0)
		return (ret);

	if (!state->ucs_connected) {
		/* Process written struct uinput_user_dev */
		if (uio->uio_resid != sizeof(struct uinput_user_dev)) {
			debugf("write size not multiple of struct uinput_user_dev size");
			return (EINVAL);
		}

		uiomove(&userdev, sizeof(struct uinput_user_dev), uio);
		uinput_setup_provider(state->ucs_evdev, &userdev);
	} else {
		/* Process written event */
		if (uio->uio_resid % sizeof(struct input_event) != 0) {
			debugf("write size not multiple of struct input_event size");
			return (EINVAL);
		}

		while (uio->uio_resid > 0) {
			uiomove(&event, sizeof(struct input_event), uio);
			ret = evdev_inject_event(state->ucs_evdev, event.type,
			    event.code, event.value);

			if (ret != 0)
				return (ret);
		}
	}

	return (0);
}

static int
uinput_setup_provider(struct evdev_dev *evdev, struct uinput_user_dev *udev)
{
	struct input_absinfo absinfo;
	int i;

	debugf("uinput: setup_provider called, udev=%p", udev);

	evdev_set_name(evdev, udev->name);
	memcpy(&evdev->ev_id, &udev->id, sizeof(struct input_id));
	
	bzero(&absinfo, sizeof(struct input_absinfo));
	for (i = 0; i < ABS_CNT; i++) {
		if (!isset(&evdev->ev_abs_flags, i))
			continue;

		absinfo.minimum = udev->absmin[i];
		absinfo.maximum = udev->absmax[i];
		absinfo.fuzz = udev->absfuzz[i];
		absinfo.flat = udev->absflat[i];
		evdev_set_absinfo(evdev, i, &absinfo);
	}

	return (0);
}

static int
uinput_poll(struct cdev *dev, int events, struct thread *td)
{
	int revents = 0;

	/* Always allow write */
	if (events & (POLLOUT | POLLWRNORM))
		revents |= (events & (POLLOUT | POLLWRNORM));

	return (revents);
}

static int
uinput_ioctl(struct cdev *dev, u_long cmd, caddr_t data, int fflag,
    struct thread *td)
{
	struct uinput_cdev_state *state;
	int ret, len;
	char buf[NAMELEN];

	len = IOCPARM_LEN(cmd);

	debugf("uinput: ioctl called: cmd=0x%08lx, data=%p", cmd, data);

	ret = devfs_get_cdevpriv((void **)&state);
	if (ret != 0)
		return (ret);

	switch (IOCBASECMD(cmd)) {
	case UI_GET_SYSNAME(0):
		if (!state->ucs_connected)
			return (ENOENT);
		if (len == 0)
			return (EINVAL);
		snprintf(data, len, "event%d", state->ucs_evdev->ev_unit);
		return (0);
	}

	switch (cmd) {
	case UI_DEV_CREATE:
		evdev_set_methods(state->ucs_evdev, &uinput_ev_methods);
		evdev_set_softc(state->ucs_evdev, state);
		evdev_register(NULL, state->ucs_evdev);
		state->ucs_connected = true;
		return (0);

	case UI_DEV_DESTROY:
		if (!state->ucs_connected)
			return (0);

		evdev_unregister(NULL, state->ucs_evdev);
		state->ucs_connected = false;
		return (0);

	case UI_SET_EVBIT:
		if (*(int *)data == EV_REP)
			return (evdev_support_repeat(state->ucs_evdev,
			    EVDEV_REPEAT));
		else
			return (evdev_support_event(state->ucs_evdev,
			    *(int *)data));

	case UI_SET_KEYBIT:
		return (evdev_support_key(state->ucs_evdev, *(int *)data));

	case UI_SET_RELBIT:
		return (evdev_support_rel(state->ucs_evdev, *(int *)data));

	case UI_SET_ABSBIT:
		return (evdev_support_abs(state->ucs_evdev, *(int *)data));

	case UI_SET_MSCBIT:
		return (evdev_support_msc(state->ucs_evdev, *(int *)data));

	case UI_SET_LEDBIT:
		return (evdev_support_led(state->ucs_evdev, *(int *)data));

	case UI_SET_SNDBIT:
		return (evdev_support_snd(state->ucs_evdev, *(int *)data));

	case UI_SET_FFBIT:
		/* Fake unsupported ioctl */
		return (0);

	case UI_SET_PHYS:
		if (state->ucs_connected)
			return (EINVAL);
		ret = copyinstr(*(void **)data, buf, sizeof(buf), NULL);
		/* Linux returns EINVAL when string does not fit the buffer */
		if (ret == ENAMETOOLONG)
			ret = EINVAL;
		if (ret != 0)
			return (ret);
		evdev_set_phys(state->ucs_evdev, buf);
		return (0);

	case UI_SET_SWBIT:
		return (evdev_support_sw(state->ucs_evdev, *(int *)data));

	case UI_SET_PROPBIT:
		return (evdev_support_prop(state->ucs_evdev, *(int *)data));

	case UI_BEGIN_FF_UPLOAD:
	case UI_END_FF_UPLOAD:
	case UI_BEGIN_FF_ERASE:
	case UI_END_FF_ERASE:
		/* Fake unsupported ioctl */
		return (0);

	case UI_GET_VERSION:
		*(unsigned int *)data = UINPUT_VERSION;
		return (0);
	}

	return (EINVAL);
}

static int
uinput_cdev_create(void)
{
	struct make_dev_args mda;
	struct cdev *cdev;

	make_dev_args_init(&mda);
	mda.mda_devsw = &uinput_cdevsw;
	mda.mda_uid = UID_ROOT;
	mda.mda_gid = GID_WHEEL;
	mda.mda_mode = 0600;

	make_dev_s(&mda, &cdev, "uinput");

	return (0);
}

SYSINIT(uinput, SI_SUB_DRIVERS, SI_ORDER_MIDDLE, uinput_cdev_create, NULL);
