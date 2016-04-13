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
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/conf.h>
#include <sys/malloc.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>
#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>

#include <dev/evdev/input.h>
#include <dev/evdev/evdev.h>

#ifdef DEBUG
#define	debugf(fmt, args...)	printf("evdev: " fmt "\n", ##args)
#else
#define	debugf(fmt, args...)
#endif

MALLOC_DEFINE(M_EVDEV, "evdev", "evdev memory");

static inline void set_bit(unsigned long *, int);
static inline void clr_bit(unsigned long *, int);
static inline void change_bit(unsigned long *, int, int);
static inline int get_bit(unsigned long *, int);
static void evdev_assign_id(struct evdev_dev *);
static void evdev_start_repeat(struct evdev_dev *, uint16_t);
static void evdev_stop_repeat(struct evdev_dev *);
static int evdev_check_event(struct evdev_dev *, uint16_t, uint16_t, int32_t);

static inline void
set_bit(unsigned long *array, int bit)
{
	array[bit / LONG_WIDTH] |= (1LL << (bit % LONG_WIDTH));
}

static inline void
clr_bit(unsigned long *array, int bit)
{
	array[bit / LONG_WIDTH] &= ~(1LL << (bit % LONG_WIDTH));
}

static inline void
change_bit(unsigned long *array, int bit, int value)
{
	if (value)
		set_bit(array, bit);
	else
		clr_bit(array, bit);
}

static inline int
get_bit(unsigned long *array, int bit)
{
	return ((array[bit / LONG_WIDTH] & (1LL << (bit % LONG_WIDTH))) > 0);
}

struct evdev_dev *
evdev_alloc(void)
{

	return malloc(sizeof(struct evdev_dev), M_EVDEV, M_WAITOK | M_ZERO);
}

void
evdev_free(struct evdev_dev *evdev)
{

	free(evdev, M_EVDEV);
}

int
evdev_set_report_size(struct evdev_dev *evdev, size_t report_size)
{
	if (report_size > KEY_CNT + REL_CNT + ABS_CNT + MAX_MT_SLOTS * MT_CNT +
	    MSC_CNT + LED_CNT + SND_CNT + SW_CNT + FF_CNT)
		return (EINVAL);

	evdev->ev_report_size = report_size;
	return (0);
}

static size_t
evdev_estimate_report_size(struct evdev_dev *evdev)
{
	size_t size = 0;
	int16_t i;
	unsigned long type_detected;

	/*
	 * Keyboards generate one event per report but other devices with
	 * buttons like mouses can report events simultaneously
	 */
	type_detected = 0;
	for (i = 0; i < KEY_CNT; i++) {
		if (get_bit(evdev->ev_key_flags, i)) {
			if (i >= BTN_MISC && i < KEY_OK)
				size++;
			else
				type_detected = 1;
		}
	}
	size += (type_detected != 0);

	/* All relative axes can be reported simultaneously */
	for (i = 0; i < REL_CNT; i++)
		if (get_bit(evdev->ev_rel_flags, i))
			size++;

	/*
	 * All absolute axes can be reported simultaneously.
	 * Multitouch axes can be reported ABS_MT_SLOT times
	 */
	for (i = 0; i < ABS_CNT; i++) {
		if (get_bit(evdev->ev_abs_flags, i)) {
			if (ABS_IS_MT(i) || i == ABS_MT_SLOT)
				size += MAX_MT_SLOTS;
			else
				size++;
		}
	}

	/* All misc events can be reported simultaneously */
	for (i = 0; i < MSC_CNT; i++)
		if (get_bit(evdev->ev_msc_flags, i))
			size++;


	/* All leds can be reported simultaneously */
	for (i = 0; i < LED_CNT; i++)
		if (get_bit(evdev->ev_led_flags, i))
			size++;

	/* Assume other events are generated once per report */
	type_detected = 0;
	for (i = 0; i < nlongs(SND_CNT); i++)
		type_detected |= evdev->ev_snd_flags[i];
	size += (type_detected != 0);

	type_detected = 0;
	for (i = 0; i < nlongs(SW_CNT); i++)
		type_detected |= evdev->ev_sw_flags[i];
	size += (type_detected != 0);

	/* XXX: FF part is not implemented yet */

	size++;		/* SYN_REPORT */
	return (size);
}

int
evdev_register(device_t dev, struct evdev_dev *evdev)
{
	int ret, i;

	device_printf(dev, "registered evdev provider: %s <%s>\n",
	    evdev->ev_name, evdev->ev_serial);

	/* Initialize internal structures */
	evdev->ev_dev = dev;
	mtx_init(&evdev->ev_mtx, "evmtx", "evdev", MTX_DEF);
	LIST_INIT(&evdev->ev_clients);

	if (dev != NULL)
		strlcpy(evdev->ev_shortname, device_get_nameunit(dev), NAMELEN);
	
	if (evdev_event_supported(evdev, EV_REP) && !evdev->ev_rep_driver) {
		/* Initialize callout */
		callout_init(&evdev->ev_rep_callout, 1);

		if (evdev->ev_rep[REP_DELAY] == 0 &&
		    evdev->ev_rep[REP_PERIOD] == 0) {
			/* Supply default values */
			evdev->ev_rep[REP_DELAY] = 250;
			evdev->ev_rep[REP_PERIOD] = 33;
		}
	}

	/* Retrieve bus info */
	evdev_assign_id(evdev);

	/* Initialize multitouch protocol type B states */
	for (i = 0; i < MAX_MT_SLOTS; i++)
		evdev->ev_mt_states[i][ABS_MT_INDEX(ABS_MT_TRACKING_ID)] = -1;

	/* Estimate maximum report size */
	if (evdev->ev_report_size == 0) {
		ret = evdev_set_report_size(evdev,
		    evdev_estimate_report_size(evdev));
		if (ret != 0)
			goto bail_out;
	}

	/* Create char device node */
	ret = evdev_cdev_create(evdev);
bail_out:
	if (ret != 0)
		mtx_destroy(&evdev->ev_mtx);

	return (ret);
}

int
evdev_unregister(device_t dev, struct evdev_dev *evdev)
{
	struct evdev_client *client;
	int ret;
	device_printf(dev, "unregistered evdev provider: %s\n", evdev->ev_name);

	EVDEV_LOCK(evdev);
	evdev->ev_cdev->si_drv1 = NULL;
	/* Wake up sleepers */
	LIST_FOREACH(client, &evdev->ev_clients, ec_link) {
		EVDEV_CLIENT_LOCKQ(client);
		evdev_notify_event(client);
		EVDEV_CLIENT_UNLOCKQ(client);
	}
	EVDEV_UNLOCK(evdev);

	if (evdev_event_supported(evdev, EV_REP) && !evdev->ev_rep_driver)
		callout_drain(&evdev->ev_rep_callout);

	/* destroy_dev can sleep so release lock */
	ret = evdev_cdev_destroy(evdev);
	if (ret == 0)
		mtx_destroy(&evdev->ev_mtx);

	return (ret);
}

inline void
evdev_set_name(struct evdev_dev *evdev, const char *name)
{

	snprintf(evdev->ev_name, NAMELEN, "%s", name);
}

inline void
evdev_set_phys(struct evdev_dev *evdev, const char *name)
{

	snprintf(evdev->ev_shortname, NAMELEN, "%s", name);
}

inline void
evdev_set_serial(struct evdev_dev *evdev, const char *serial)
{

	snprintf(evdev->ev_serial, NAMELEN, "%s", serial);
}

inline void
evdev_set_methods(struct evdev_dev *evdev, struct evdev_methods *methods)
{

	evdev->ev_methods = methods;
}

inline void
evdev_set_softc(struct evdev_dev *evdev, void *softc)
{

	evdev->ev_softc = softc;
}

inline int
evdev_support_prop(struct evdev_dev *evdev, uint16_t prop)
{

	if (prop >= INPUT_PROP_CNT)
		return (EINVAL);

	set_bit(evdev->ev_prop_flags, prop);
	return (0);
}

inline int
evdev_support_event(struct evdev_dev *evdev, uint16_t type)
{

	if (type >= EV_CNT)
		return (EINVAL);

	set_bit(evdev->ev_type_flags, type);
	return (0);
}

inline int
evdev_support_key(struct evdev_dev *evdev, uint16_t code)
{

	if (code >= KEY_CNT)
		return (EINVAL);

	set_bit(evdev->ev_key_flags, code);
	return (0);
}

inline int
evdev_support_rel(struct evdev_dev *evdev, uint16_t code)
{

	if (code >= REL_CNT)
		return (EINVAL);

	set_bit(evdev->ev_rel_flags, code);
	return (0);
}

inline int
evdev_support_abs(struct evdev_dev *evdev, uint16_t code)
{

	if (code >= ABS_CNT)
		return (EINVAL);

	set_bit(evdev->ev_abs_flags, code);
	return (0);
}


inline int
evdev_support_msc(struct evdev_dev *evdev, uint16_t code)
{

	if (code >= MSC_CNT)
		return (EINVAL);

	set_bit(evdev->ev_msc_flags, code);
	return (0);
}


inline int
evdev_support_led(struct evdev_dev *evdev, uint16_t code)
{

	if (code >= LED_CNT)
		return (EINVAL);

	set_bit(evdev->ev_led_flags, code);
	return (0);
}

inline int
evdev_support_snd(struct evdev_dev *evdev, uint16_t code)
{

	if (code >= SND_CNT)
		return (EINVAL);

	set_bit(evdev->ev_snd_flags, code);
	return (0);
}

inline int
evdev_support_sw(struct evdev_dev *evdev, uint16_t code)
{
	if (code >= SW_CNT)
		return (EINVAL);

	set_bit(evdev->ev_sw_flags, code);
	return (0);
}

inline int
evdev_support_repeat(struct evdev_dev *evdev, enum evdev_repeat_mode mode)
{

	if (mode != NO_REPEAT)
		evdev_support_event(evdev, EV_REP);

	if (mode == DRIVER_REPEAT)
		evdev->ev_rep_driver = true;

	return (0);
}

bool
evdev_event_supported(struct evdev_dev *evdev, uint16_t type)
{

	if (type >= EV_CNT)
		return (false);

	return (get_bit(evdev->ev_type_flags, type));
}


inline void
evdev_set_absinfo(struct evdev_dev *evdev, uint16_t axis,
    struct input_absinfo *absinfo)
{

	memcpy(&evdev->ev_absinfo[axis], absinfo, sizeof(struct input_absinfo));
}

inline void
evdev_set_repeat_params(struct evdev_dev *evdev, uint16_t property, int value)
{

	KASSERT(property < REP_CNT, ("invalid evdev repeat property"));
	evdev->ev_rep[property] = value;
}

static int
evdev_check_event(struct evdev_dev *evdev, uint16_t type, uint16_t code,
    int32_t value)
{

	/* Allow SYN events implicitly */
	if (type != EV_SYN && !evdev_event_supported(evdev, type))
		return (EINVAL);

	switch (type) {
	case EV_SYN:
		if (code >= SYN_CNT)
			return (EINVAL);
		break;

	case EV_KEY:
		if (code >= KEY_CNT)
			return (EINVAL);
		if (!get_bit(evdev->ev_key_flags, code))
			return (EINVAL);
		break;

	case EV_REL:
		if (code >= REL_CNT)
			return (EINVAL);
		if (!get_bit(evdev->ev_rel_flags, code))
			return (EINVAL);
		break;

	case EV_ABS:
		if (code >= ABS_CNT)
			return (EINVAL);
		if (!get_bit(evdev->ev_abs_flags, code))
			return (EINVAL);
		if (code == ABS_MT_SLOT && value >= MAX_MT_SLOTS)
			return (EINVAL);
		break;

	case EV_MSC:
		if (code >= MSC_CNT)
			return (EINVAL);
		if (!get_bit(evdev->ev_msc_flags, code))
			return (EINVAL);
		break;

	case EV_LED:
		if (code >= LED_CNT)
			return (EINVAL);
		if (!get_bit(evdev->ev_led_flags, code))
			return (EINVAL);
		break;

	case EV_SND:
		if (code >= SND_CNT)
			return (EINVAL);
		if (!get_bit(evdev->ev_snd_flags, code))
			return (EINVAL);
		break;

	case EV_SW:
		if (code >= SW_CNT)
			return (EINVAL);
		if (!get_bit(evdev->ev_sw_flags, code))
			return (EINVAL);
		break;

	case EV_REP:
		if (code >= REP_CNT)
			return (EINVAL);
		break;

	default:
		return (EINVAL);
	}

	return (0);
}

int
evdev_push_event(struct evdev_dev *evdev, uint16_t type, uint16_t code,
    int32_t value)
{
	struct evdev_client *client;
	int32_t postponed_mt_slot = -1;

	if (evdev_check_event(evdev, type, code, value) != 0)
		return (EINVAL);

	debugf("%s pushed event %d/%d/%d",
	    evdev->ev_shortname, type, code, value);

	/*
	 * For certain event types, update device state bits
	 * and convert level reporting to edge reporting
	 */
	switch (type) {
	case EV_KEY:
		if (get_bit(evdev->ev_key_states, code) ==
		    (value != KEY_EVENT_UP)) {
			/* Detect key repeats. */
			if (evdev_event_supported(evdev, EV_REP)
			    && value != KEY_EVENT_UP)
				value = KEY_EVENT_REPEAT;
			else
				return (0);
		} else {
			change_bit(evdev->ev_key_states, code, value);
			if (evdev_event_supported(evdev, EV_REP) &&
			    !evdev->ev_rep_driver) {
				if (value != KEY_EVENT_UP)
					evdev_start_repeat(evdev, code);
				else
					evdev_stop_repeat(evdev);
			}
		}
		break;

	case EV_LED:
		if (get_bit(evdev->ev_led_states, code) == value)
			return (0);
		change_bit(evdev->ev_led_states, code, value);
		break;

	case EV_SND:
		if (get_bit(evdev->ev_snd_states, code) == value)
			return (0);
		change_bit(evdev->ev_snd_states, code, value);
		break;

	case EV_SW:
		if (get_bit(evdev->ev_sw_states, code) == value)
			return (0);
		change_bit(evdev->ev_sw_states, code, value);
		break;

	case EV_REP:
		if (evdev->ev_rep[code] == value)
			return (0);
		evdev_set_repeat_params(evdev, code, value);
		break;

	/* For EV_ABS, save last value in absinfo and ev_mt_states */
	case EV_ABS:
		switch (code) {
		case ABS_MT_SLOT:
			/* Postpone ABS_MT_SLOT till next event */
			evdev->last_reported_mt_slot = value;
			return (0);

		case ABS_MT_FIRST ... ABS_MT_LAST:
			/* Don`t repeat MT protocol type B events */
			if (evdev->ev_mt_states[evdev->last_reported_mt_slot]
			    [ABS_MT_INDEX(code)] == value)
				return (0);
			evdev->ev_mt_states[evdev->last_reported_mt_slot]
			    [ABS_MT_INDEX(code)] = value;
			if (evdev->last_reported_mt_slot !=
			    CURRENT_MT_SLOT(evdev)) {
				CURRENT_MT_SLOT(evdev) =
				    evdev->last_reported_mt_slot;
				postponed_mt_slot = CURRENT_MT_SLOT(evdev);
			}
			break;

		default:
			/* XXX: Do we need MT protocol type A handling ??? */
			if (evdev->ev_absinfo[code].value == value)
				return (0);
			evdev->ev_absinfo[code].value = value;
		}
		break;
	}

	/* Skip empty reports */
	if (type == EV_SYN && code == SYN_REPORT) {
		if (evdev->events_since_last_syn == 0)
			return (0);
		evdev->events_since_last_syn = 0;
	} else {
		evdev->events_since_last_syn++;
	}

	/* Propagate event through all clients */
	EVDEV_LOCK(evdev);
	LIST_FOREACH(client, &evdev->ev_clients, ec_link) {
		if (evdev->ev_grabber != NULL && evdev->ev_grabber != client)
			continue;

		EVDEV_CLIENT_LOCKQ(client);
		/* report postponed ABS_MT_SLOT */
		if (postponed_mt_slot != -1)
			evdev_client_push(client,
			    EV_ABS, ABS_MT_SLOT, postponed_mt_slot);
		evdev_client_push(client, type, code, value);

		if (type == EV_SYN && code == SYN_REPORT)
			evdev_notify_event(client);
		EVDEV_CLIENT_UNLOCKQ(client);
	}
	EVDEV_UNLOCK(evdev);

	return (0);
}

int
evdev_inject_event(struct evdev_dev *evdev, uint16_t type, uint16_t code,
    int32_t value)
{
	int ret = 0;

	switch (type) {
	case EV_REP:
		/* evdev repeats should not be processed by hardware driver */
		if (!evdev->ev_rep_driver)
			goto push;
		/* FALLTHROUGH */
	case EV_LED:
	case EV_MSC:
	case EV_SND:
	case EV_FF:
		if (evdev->ev_methods != NULL &&
		    evdev->ev_methods->ev_event != NULL)
			evdev->ev_methods->ev_event(evdev, evdev->ev_softc,
			    type, code, value);
		/*
		 * Leds and driver repeats should be reported in ev_event
		 * method body to interoperate with kbdmux states and rates
		 * propagation so both ways (ioctl and evdev) of changing it
		 * will produce only one evdev event report to client.
		 */
		if (type == EV_LED || type == EV_REP)
			break;
		/* FALLTHROUGH */
	case EV_SYN:
	case EV_KEY:
	case EV_REL:
	case EV_ABS:
	case EV_SW:
push:
		ret = evdev_push_event(evdev, type,  code, value);
		break;

	default:
		ret = EINVAL;
	}

	return (ret);
}

inline int
evdev_sync(struct evdev_dev *evdev)
{
	
	return (evdev_push_event(evdev, EV_SYN, SYN_REPORT, 1));
}


inline int
evdev_mt_sync(struct evdev_dev *evdev)
{
	
	return (evdev_push_event(evdev, EV_SYN, SYN_MT_REPORT, 1));
}

int
evdev_register_client(struct evdev_dev *evdev, struct evdev_client *client)
{
	int ret = 0;

	debugf("adding new client for device %s", evdev->ev_shortname);

	EVDEV_LOCK_ASSERT(evdev);

	if (LIST_EMPTY(&evdev->ev_clients) && evdev->ev_methods != NULL &&
	    evdev->ev_methods->ev_open != NULL) {
		debugf("calling ev_open() on device %s", evdev->ev_shortname);
		ret = evdev->ev_methods->ev_open(evdev, evdev->ev_softc);
	}
	if (ret == 0)
		LIST_INSERT_HEAD(&evdev->ev_clients, client, ec_link);
	return (ret);
}

void
evdev_dispose_client(struct evdev_dev *evdev, struct evdev_client *client)
{
	debugf("removing client for device %s", evdev->ev_shortname);

	EVDEV_LOCK_ASSERT(evdev);

	LIST_REMOVE(client, ec_link);
	if (LIST_EMPTY(&evdev->ev_clients) && evdev->ev_methods != NULL &&
	    evdev->ev_methods->ev_close != NULL)
		evdev->ev_methods->ev_close(evdev, evdev->ev_softc);
	evdev_release_client(evdev, client);
}

int
evdev_grab_client(struct evdev_dev *evdev, struct evdev_client *client)
{

	EVDEV_LOCK_ASSERT(evdev);

	if (evdev->ev_grabber != NULL)
		return (EBUSY);

	evdev->ev_grabber = client;

	return (0);
}

int
evdev_release_client(struct evdev_dev *evdev, struct evdev_client *client)
{

	EVDEV_LOCK_ASSERT(evdev);

	if (evdev->ev_grabber != client)
		return (EINVAL);

	evdev->ev_grabber = NULL;

	return (0);
}

static void
evdev_assign_id(struct evdev_dev *dev)
{
	device_t parent;
	devclass_t devclass;
	const char *classname;

	if (dev->ev_id.bustype != 0)
		return;

	if (dev->ev_dev == NULL) {
		dev->ev_id.bustype = BUS_VIRTUAL;
		return;
	}

	parent = device_get_parent(dev->ev_dev);
	if (parent == NULL) {
		dev->ev_id.bustype = BUS_HOST;
		return;
	}

	devclass = device_get_devclass(parent);
	classname = devclass_get_name(devclass);

	debugf("parent bus classname: %s", classname);

	if (strcmp(classname, "pci") == 0) {
		dev->ev_id.bustype = BUS_PCI;
		dev->ev_id.vendor = pci_get_vendor(dev->ev_dev);
		dev->ev_id.product = pci_get_device(dev->ev_dev);
		dev->ev_id.version = pci_get_revid(dev->ev_dev);
		return;
	}

	if (strcmp(classname, "uhub") == 0) {
		struct usb_attach_arg *uaa = device_get_ivars(dev->ev_dev);
		dev->ev_id.bustype = BUS_USB;
		dev->ev_id.vendor = uaa->info.idVendor;
		dev->ev_id.product = uaa->info.idProduct;
		return;
	}

	if (strcmp(classname, "atkbdc") == 0) {
		devclass = device_get_devclass(dev->ev_dev);
		classname = devclass_get_name(devclass);
		dev->ev_id.bustype = BUS_I8042;
		if (strcmp(classname, "atkbd") == 0) {
			dev->ev_id.vendor = PS2_KEYBOARD_VENDOR;
			dev->ev_id.product = PS2_KEYBOARD_PRODUCT;
		} else if (strcmp(classname, "psm") == 0) {
			dev->ev_id.vendor = PS2_MOUSE_VENDOR;
			dev->ev_id.product = PS2_MOUSE_GENERIC_PRODUCT;
		}
		return;
	}

	dev->ev_id.bustype = BUS_HOST;
}

static void
evdev_repeat_callout(void *arg)
{
	struct evdev_dev *evdev = (struct evdev_dev *)arg;

	evdev_push_event(evdev, EV_KEY, evdev->ev_rep_key, KEY_EVENT_REPEAT);
	evdev_sync(evdev);

	if (evdev->ev_rep[REP_PERIOD])
		callout_reset(&evdev->ev_rep_callout,
		    evdev->ev_rep[REP_PERIOD] * hz / 1000,
		    evdev_repeat_callout, evdev);
}

static void
evdev_start_repeat(struct evdev_dev *evdev, uint16_t key)
{

	if (evdev->ev_rep[REP_DELAY]) {
		evdev->ev_rep_key = key;
		callout_reset(&evdev->ev_rep_callout,
		    evdev->ev_rep[REP_DELAY] * hz / 1000,
		    evdev_repeat_callout, evdev);
	}
}

static void
evdev_stop_repeat(struct evdev_dev *evdev)
{

	callout_drain(&evdev->ev_rep_callout);
}
